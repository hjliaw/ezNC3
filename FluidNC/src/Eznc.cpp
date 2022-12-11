/*
Eznc.cpp - oled + encoder + switch interface
*/

#include "Protocol.h"
#include "Event.h"
#include "Eznc.h"

#include "Machine/MachineConfig.h"
#include "Machine/Homing.h"
#include "Report.h"         // report_feedback_message
#include "Limits.h"         // limits_get_state, soft_limit
#include "Planner.h"        // plan_get_current_block
#include "MotionControl.h"  // PARKING_MOTION_LINE_NUMBER
#include "Settings.h"       // settings_execute_startup
#include "InputFile.h"      // infile
#include "Logging.h"
#include "Machine/LimitPin.h"

#ifdef INCLUDE_OLED_BASIC
#include "Custom/oled_io.h"  // for calling oled directly
#endif

extern volatile int uimenu_active;
extern volatile int update_dro;
extern volatile int update_menu;

extern char   ui_txt[4][Wchars+1];
extern int    ui_sel, ui_frame;

String ez_select_file();
String ez_gcfn;

char   gbuf[16][Nstr];

unsigned long jog_t0, ez_t0, ez_t1;
char eznc_line[LINE_BUFFER_SIZE];
int cancelJog;
int jog_axis = 0;   // 0..2 for now                                                                                      
int ezJog = 0;
int32_t enc_cnt = 0;
int   jog_stepsize = 1;
float jog_speed = 100;  // mm/min

long run_t0;

// utilities
unsigned int str_length(const char* strp)    // display width
{
    unsigned int len = 0, i;

    for (i=0; i < 256; len++ ){    // max 256 chars
        unsigned char c = (unsigned char) strp[i];
        if( c == 0 ) break;
#ifdef UTF8
        if (      c <= 127 )         i+=1;
        else if ((c & 0xE0) == 0xC0) i+=2;
        else if ((c & 0xF0) == 0xE0) i+=3;
        else if ((c & 0xF8) == 0xF0) i+=4;
        else return 0; //invalid utf8
#else
        i+=1;
#endif
    }
    return len;
}

// offset and len are in display width
String str_substr(const char* strp, unsigned int offset, unsigned int len)
{
    unsigned int i, j, ofs;
    unsigned char c;
    String mStr = "";
        
    for (i=0, ofs = offset; i < 256 && ofs > 0; ofs-- ){  // max= 64..256 chars
        c = (unsigned char) strp[i];
#ifdef UTF8
        if (      c <= 127 )         i+=1;
        else if ((c & 0xE0) == 0xC0) i+=2;
        else if ((c & 0xF0) == 0xE0) i+=3;
        else if ((c & 0xF8) == 0xF0) i+=4;
        //else if ((c & 0xFC) == 0xF8) i+=5;  // unnecessary in 4 byte UTF-8
        //else if ((c & 0xFE) == 0xFC) i+=6;
        else
                    return ""; //invalid utf8
#else
        i++;
#endif
    }

    for (ofs=j=0; j < 128 && ofs < len; ofs++ ){   // implicit 128 chars limit
        c = (unsigned char) strp[i+j];
#ifdef UTF8
        if (      c <= 127         ) j+=1;
        else if ((c & 0xE0) == 0xC0) j+=2;
        else if ((c & 0xF0) == 0xE0) j+=3;
        else if ((c & 0xF8) == 0xF0) j+=4;
        else
            return ""; //invalid utf8
#else
        j++;
#endif
        }

    for( ofs=0; ofs < j; ofs++){
        if( strp[i+ofs] == 0 ) break;
        mStr += strp[i+ofs];
    }
        
    return mStr;
}

// auto-scrolling long string, return non-zero means text has been shifted
int scroll( char *dest, char *src, unsigned int wi )  // non zero wi will initialize
{
    static unsigned int pos, zcnt, len, width;
    static long t0;
    String strx;

    // challenges: char count (length) and displayed width could be different !
    // UTF8 utils abort easily
        
    if( wi ){  // init with variable width, 1=Wchars, otherwise...
        width = ( wi > 1 ) ? wi : Wchars; 
        pos = zcnt = 0;
        t0 = millis();
        len = str_length(src);  

        if( len > width )
            strx = str_substr( src, 0, width );
        else
            strx = str_substr( src, 0, len );

        strx.toCharArray( dest, Nstr);

        // Serial.print( "DBG: " );
        // Serial.print( str_substr( src, 0, len ) );
        // Serial.print( " len=" );
        // Serial.println( len );
        // if( len > width ){
        //      Serial.print( "DBG: " );
        //      Serial.print( strx = str_substr( src, 0, width ) );
        //      Serial.print( " width=" );
        //      Serial.println( width );
        // }

        return 0;
    }

    long t1 = millis();
    if( t1-t0 > Tscroll && len > width ){
        t0 = t1;
        if( (pos || zcnt > Nwaitscr) ) pos++;
        else                           zcnt++;

        if ((pos + width - 2 ) > len ) pos = zcnt = 0;  // less over-scroll

        if( (pos + width) < len )
            strx = str_substr( src, pos, width );
        else
            strx = str_substr( src, pos, len - pos );

        strx.toCharArray( dest, Nstr);

        // Serial.print( "DBG: strx= " );
        // Serial.println( strx );

        return 1;
    }
    return 0;
}

//--------------------------------------------------------------
// spiffs

String gcname[128];
int    Ngcf = 0;

bool has_ending (std::string const &fullString, std::string const &ending) {
        if (fullString.length() >= ending.length()) {
                return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
        } else {
                return false;
        }
}

int bound_int( int x, int min, int max, int flash ){
        int tmp;
        if( min > max ){
                tmp = min;
                min = max;
                max = tmp;
        }
        if (x < min || x > max ) oled_flash();
        if (x < min) return min;
        if (x > max) return max;
        return x;
}

void listLFS(){ // list local file system, save file names in gcname, set Ngcf
    char fname[256];
    Ngcf = 0;

    std::error_code ec;
    FluidPath fpath { "", "spiffs", ec };
    if (ec){
        log_error( "no sd card, " << ec.message() );                
        return;
    }

    auto iter = stdfs::recursive_directory_iterator { fpath, ec };
    if (ec) {
        log_error( ec.message() );
        return;
    }

    for (auto const& dir_entry : iter) {
        if ( ! dir_entry.is_directory() && (
            has_ending( dir_entry.path().filename(),  ".gc") || 
            has_ending( dir_entry.path().filename(), ".ngc") ||
            has_ending( dir_entry.path().filename(),  ".GC") || 
            has_ending( dir_entry.path().filename(), ".NGC")
            )
        ){
            // String is Arduino defined class, filename is std::string
            gcname[Ngcf] = String( dir_entry.path().filename().c_str() );
            if (Ngcf++ >= 128-4) break;  // avoid overflow
        }                        
    }
}

//---------------------------------------------------------------------------------
// eznc jogging
//---------------------------------------------------------------------------------

void ez_jog( int32_t rot ){
    char axis[3] = {'X', 'Y', 'Z'};
    static int32_t odir;
    if( rot == 0 ) return;
    jog_t0 = millis();

    //Serial.print( "[uienc] " ); Serial.print( jog_t0  );                                                             
    //Serial.print( "rot= " );  Serial.println( rot );                                                                 

    if( odir * rot < 0 ){  // reversal                                                                                 
        cancelJog = 1;      //ez_cancel_jog();                                                                         
        // some encoders have big jump on reversal ?                                                                   
        rot = (rot > 0) ? 1 : -1;   // can't just clear, hack it                                                       
    }
    else{
        // TODO: user specified jogging speed and steps
        // two ticks per step
        if( gc_state.modal.units == Units::Mm ){
            sprintf( eznc_line, "$J=G21G91%c%.3fF%.1f", 
                    axis[jog_axis], 0.01*jog_stepsize*rot/2, jog_speed );
        }
        else{
            sprintf( eznc_line, "$J=G21G91%c%.3fF%.1f",
                    axis[jog_axis], 0.0254*jog_stepsize*rot/4, jog_speed );
        }
        //report_status_message(gc_execute_line(eznc_line, Uart0), Uart0);
        gc_execute_line(eznc_line, Uart0);

        //log_info( "ezJog " << eznc_line ); 
        ezJog = 1;                                                                     
    }
    odir = rot;
}

void ez_cancel_jog(){
    if (sys.state == State::Jog ) {
        protocol_send_event(&motionCancelEvent);                                                                   
    }
    ezJog = 0;  // not necessary ?
}

//-------------------------------------------------------------------------

#ifdef INCLUDE_OLED_IO

String ez_select_file(){
    listLFS();

    if( Ngcf == 0 ){
        clearBtn();
        u8g_print(  (char *) "No gc|ngc file",
                    (char *) " upload",
                    (char *) " or rename",
                    (char *) "clck to cont" );
        while( ! btnClickedRlsd() && ! clickCounterSWR );
        clearBtn();
        return "";
    }
        
    // try a new way to do sel menu, as number of file entries can be >16
    int sel =0, smin=0;
    while( ! btnClickedRlsd() && ! clickCounterSWR ){
        for( int i=0; i<4; i++ ) gcname[Ngcf+i] = "  ";
        strncpy( gbuf[0], "Select g-code", Nstr );

        for( int i=1; i<4; i++ )
            // eznc first char='/', but fluidnc removed that
            strncpy( gbuf[i]+1, gcname[smin+i-1].c_str(), Nstr-1 );

        gbuf[1][0] = gbuf[2][0] = gbuf[3][0] = ' ';

        gbuf[sel-smin+1][0] = '>';
        u8g_print( gbuf[0], gbuf[1], gbuf[2], gbuf[3] );

        if( int32_t r =  readEncoder(0) ){
            //char msg[128];
            if( r < 0 ) sel--;
            if( r > 0 ) sel++;
            sel  = bound_int(  sel,     0, Ngcf-1, 1 /*flash*/);
            smin = bound_int( smin, sel-2,    sel, 0);
            smin = bound_int( smin,     0, Ngcf-1, 0);

            //sprintf( msg, "[ez dbg] sel= %d, smin= %d, Ngcf= %d", sel, smin, Ngcf );
            //Serial.println( msg );
        }
    }
    if( clickCounterSWR ) return "";

    clearBtn();

    // TODO
    //if( confirm( (char *)"Run g-code", (char *)gcname[sel].c_str() ))
            return gcname[sel];
    //else
    //        return "";

}

void u8g_print( char *s0, char *s1, char *s2, char *s3, int8_t sel, int8_t frame )
{
    ui_sel = sel;
    ui_frame = frame;
    snprintf(ui_txt[0], Wchars, "%s", s0 );
    snprintf(ui_txt[1], Wchars, "%s", s1 );
    snprintf(ui_txt[2], Wchars, "%s", s2 );
    snprintf(ui_txt[3], Wchars, "%s", s3 );
    update_menu = 1;     // dispatch to oledUI
}

// generic menu drawing with rolling entries
//   keep 1st title line, rotate other items, up to 3 items on screen
//   menu entries at gbuf[], sel= cur selection, *smin= starting entry
// requires  *smin <= sel <= (*smin)+2

void draw_menu(int8_t sel, int8_t *smin ){
    int8_t sm = *smin;

    if( sel >= sm && sel <= sm+2 ){
        u8g_print( gbuf[0], gbuf[sm], gbuf[sm+1], gbuf[sm+2], sel-sm+1 );
    }
    else if( sel < sm ){
        u8g_print( gbuf[0], gbuf[sel], gbuf[sel+1], gbuf[sel+2], 1 );
        *smin = sel;
    }
    else{
        u8g_print( gbuf[0], gbuf[sel-2], gbuf[sel-1], gbuf[sel], 3 );
        *smin = sel-2;
    }
}

void select_from_menu( uint8_t N, char (*ptr)[Nstr], int8_t *sp, int8_t *sminp, void (*f2upd)(char *,int) )
{
    unsigned int i;
    int8_t osv, sv = *sp;
        
    for( i=0; i<N; i++ )
        strncpy( gbuf[i], (const char*)(ptr+i), Nstr );   // bounded

    scroll( gbuf[0], ptr[0], Wchars );  // init, with frame
        
    draw_menu( sv, sminp );
    clearBtn();
    osv = sv;
    while( ! btnClickedRlsd() && ! clickCounterSWR ){
        scroll( gbuf[0], ptr[0], 0 );
        int32_t r = readEncoder(0);  // no accel

        if( r < 0 ) sv--;
        if( r > 0 ) sv++;
        if( sv < 1   ){ sv = 1;   oled_flash(); }
        if( sv > N-1 ){ sv = N-1; oled_flash(); }

        if( sv != osv ){
            osv = sv;
            if( f2upd ) f2upd( ptr[0], sv );
            log_debug( "sv= " << sv << ",txt= " << ptr[0] );
            scroll( gbuf[0], ptr[0], 1 ); // 1=init
        }
        draw_menu( sv, sminp );
    }

    if( clickCounterSWR )
        *sp = *sminp = 1;
    else
        *sp = sv;
    clearBtn();
}

void ez_menu()   // top level ui menu, only title line is auto-scrolled
{
    int8_t sel=1, smin=1, junk=1;
    int progress=0, old_progress=0;

    char menu[6][Nstr] = {
        "ezFluidNC 01234567890",    // HJL: test font and scrolling, to be removed
        "Set Pos",
        "Run G-code",
        "Change Unit",
        "Back to DRO",
        "Reset WebUi"
    };
    char msg[Nstr];

    clearBtn();
    select_from_menu( 6, menu, &sel, &smin );  // blocking
    if( clickCounterSWR ) return;

    switch(sel){
    case 1:
        // todo: start with current position
        //   want a quick way to enter zero of single axis
        log_info( "set pos");
        //sprintf( msg, "Set As (%s)", (EZnc.Unit==INCH) ? "inch" : "mm" );
        //ez_enter_pos( msg, (char *) "click to set" );
        //if( btnClicked() ){
        //    Serial.print( "[MSG:ezSetPos-" );    // add [MSG so that ugs doesn't complain                           
        //    sprintf( eznc_line, "G10L20P1X%fY0Z%f", npos[0], npos[1] );
        //    report_status_message(gc_execute_line(eznc_line, CLIENT_SERIAL), CLIENT_SERIAL);
        //}
        clearBtn();
        return;  // return or stay ?
    case 2:
        ez_gcfn = ez_select_file();
        log_info( "file: " << ez_gcfn );

        // if not return, refresh screen
#if 0                                               
        sprintf( gbuf[0], "Running" );
        strncpy( gbuf[1], ez_gcfn.c_str(), Nstr );
        sprintf( gbuf[2], "" );
        sprintf( gbuf[3], "tchR to abrt" );
        draw_menu(-1, &junk);
        update_menu = 1;
#endif
        run_t0 = millis();
        if (ez_gcfn[0] != 0 ){
                char gcmd[128];
                log_info( "run file " << ez_gcfn  );
                sprintf( gcmd, "$localfs/run=%s", ez_gcfn.c_str() );
                execute_line( gcmd, Uart0, WebUI::AuthenticationLevel::LEVEL_GUEST );
                log_info( "done file " << ez_gcfn  );
        }
        return;
    case 3:  // change unit, todo: ask and make it permament if desired                                                             
        if( gc_state.modal.units == Units::Mm )
            sprintf( eznc_line, "G20" );
        else
            sprintf( eznc_line, "G21" );
        gc_execute_line( eznc_line, Uart0);
        return;
    case 4:
        return;
    case 5:
        // confirm ?                                                                                                
        //SPIFFS.remove("/index.html.gz");
        return;
    }
}

#endif

//-------------------------------------------------------------------------

// todo: disable jogging when running, allow SWR to abort

// currently, eznc_dispatch is really just ez_dro
unsigned int lcnt = 0;
void ez_dro()
{
    //if( (lcnt % 1000) == 0) log_warn( "HJL: ez_dro loop " << lcnt );

    if( sys.state == State::Idle || sys.state == State::Jog ){
        enc_cnt = readEncoder(1);  // 1=no double reads
        if( enc_cnt != 0 ){
            ez_jog( enc_cnt );
        }
        else{
            unsigned long jog_dt = millis() - jog_t0;
            if( jog_dt > 150 && ezJog ){   // avoid multiple calls
                cancelJog = 1;
                jog_t0 += jog_dt;
            }
        }
        if( cancelJog ) ez_cancel_jog();
    }

    if( btnClicked() ){ // todo: individual button clear
  
        if( clickCounterSWL ){
            jog_axis = (jog_axis+1) % 3;
            update_dro = 1;
            log_info( "SWL jog axis " << jog_axis );
        }
            
        if( clickCounterSWR ){
            if( sys.state == State::Idle || sys.state == State::Jog ){
                jog_stepsize = jog_stepsize << 1;
                if( jog_stepsize > 4 ) jog_stepsize = 1;
                update_dro = 1;
                log_info( "SWR jog step " << jog_stepsize );
            }
            if( sys.state == State::Cycle ){
                log_warn( "HJL: SWR abort, loop " << lcnt  );
                sys.abort = true;
            }
        }   

        if( clickCounterSW1 ){  // click main button in DRO enters ui menu mode
            uimenu_active = 1;
            ez_menu();
            uimenu_active = 0;
            update_dro = 1;
            log_warn( "HJL: returned from ez_menu" );
        }
        clearBtn();            
    }
}

void ez_ui()  // NOT used, current ui_menu is blocking
{
//    if( (lcnt % 1000) == 0) log_warn( "HJL: ez_ui, loop " << lcnt );

#ifndef NO_ENCODER
    enc_cnt = readEncoder(0);
    if( enc_cnt != 0 ){
    }
#endif

    if( btnClicked() ){
        if( clickCounterSWL ){
        }
            
        if( clickCounterSWR ){
        }
            
        if( clickCounterSW1 ){
            log_info( "exit ui menu..." );
            uimenu_active = 0;
            update_dro = 1;
        }
    }
    clearBtn();
}

// called by protocal loop, try to be as short as possible
// TODO; disable jogging when running ?

// gee, once gcode file is started fron eznc, this is not called
// bcz (different from esp_grbl) Protocol loop structure

void eznc_dispatch( void )    // top level dispatcher
{
    lcnt++;
    //if( (lcnt % 1000) == 0) log_warn( "HJL: eznc_dispatch, loop " << lcnt );

    if( uimenu_active )
        ez_ui();        
    else
        ez_dro();
}