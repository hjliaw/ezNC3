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

volatile bool ez_check_cancel =0;

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
        clearBtnTouch();
        u8g_print(  (char *) "No gc|ngc file",
                    (char *) "Upload va WiFi",
                    (char *) " ",
                    (char *) "clck to cont" );
        while( ! btnClickedRlsd() && ! touchedR );
        clearBtnTouch();
        return "";
    }
        
    // try a new way to do sel menu, as number of file entries can be >16
    int sel =0, smin=0;
    while( ! btnClickedRlsd() && ! touchedR ){
        for( int i=0; i<4; i++ ) gcname[Ngcf+i] = "  ";
        strncpy( gbuf[0], "Select File", Nstr );

        for( int i=1; i<4; i++ )
            // eznc first char='/', but fluidnc removed that
            strncpy( gbuf[i]+1, gcname[smin+i-1].c_str(), Nstr-1 );

        gbuf[1][0] = gbuf[2][0] = gbuf[3][0] = ' ';

        gbuf[sel-smin+1][0] = '>';
        u8g_print( gbuf[0], gbuf[1], gbuf[2], gbuf[3], -1, 1 );

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
    if( touchedR ) return "";

    clearBtnTouch();

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
    clearBtnTouch();
    osv = sv;
    while( ! btnClickedRlsd() && ! touchedR ){
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

    if( touchedR )
        *sp = *sminp = 1;
    else
        *sp = sv;
    clearBtn();   // preserve touchdR
}

//---------------------------------------------------------------------------------------
// OK/Cancel, with frame, otherwise Yes/No no frame                                                                  
// yes=1 default positive, yes=0 default negative                                                                     
// scroll top two lines at the same time

#define MSG_OK                          "OK"
#define MSG_Cancel                      "Cancel"
#define MSG_Yes                         "Yes"
#define MSG_No                          "No"

bool ask( char *msg1, char *msg2, int ok_or_cancel, int yes  )
{
    unsigned int len[2], max_len, pos = 0, zcnt = 0;
    long t0 = millis();
    String strx;
    char *msgp[2];

    int s;
    if( yes ) s = 2;
    else      s = 3;

    msgp[0] = msg1; msgp[1] = msg2;
    max_len = 0;
    for( int i=0; i<2; i++ ){
        len[i] = str_length( msgp[i] );
        if( len[i] > max_len ) max_len = len[i];

        if( len[i] > Wchars )
            strx = str_substr( msgp[i], 0, Wchars );
        else
            strx = str_substr( msgp[i], 0, len[i] );
        strx.toCharArray( gbuf[i], Nstr);
    }
    sprintf( gbuf[2], ok_or_cancel ? MSG_OK     : MSG_Yes );
    sprintf( gbuf[3], ok_or_cancel ? MSG_Cancel : MSG_No  );

    u8g_print( gbuf[0], gbuf[1], gbuf[2], gbuf[3], s, ok_or_cancel );   // OK/Cancel with frame                       

    clearBtnTouch();
    while( ! btnClickedRlsd() && ! touchedR ){

        long t1 = millis();
        if( t1-t0 > Tscroll && max_len > Wchars ){
            t0 = t1;
            if( (pos || zcnt > Nwaitscr) ) pos++;
            else                           zcnt++;

            if( (pos + Wchars - 2 ) > max_len ) pos = zcnt = 0;

            for( int i=0; i<2; i++ ){
                if( pos + Wchars < len[i] )
                    strx = str_substr( msgp[i], pos, Wchars );
                else{
                    if( len[i] > pos )
                        strx = str_substr( msgp[i], pos, len[i] - pos );
                    else
                        strx = "";
                }
                strx.toCharArray( gbuf[i], Nstr);
            }
        }

        int32_t rot = readEncoder(0);
        if( rot > 0 ) s++;
        if( rot < 0 ) s--;
        if(  s < 2  ){ s = 2; oled_flash(); }
        if(  s > 3  ){ s = 3; oled_flash(); }
        u8g_print( gbuf[0], gbuf[1], gbuf[2], gbuf[3], s, ok_or_cancel );  // last param= frame                       
    }

    if( ok_or_cancel ){
        if( touchedR ) s = 3;            // cancel                                                                    
    }
    else{
        if( touchedR ) s = yes ? 2 : 3;  // cancel means default                                                      
    }
    clearBtnTouch();
    return s == 2;
}

bool confirm( char *msg1, char *msg2, int yes )    // ok or cancel                                                    
{
    return ask( msg1, msg2, 1, yes );
}

bool yes_or_no( char *msg1, char *msg2, int yes )    // Yes or No question, with default answer                       
{
    return ask( msg1, msg2, 0, yes );
}

//-----------------------------------------------------------------------------------------------

float x_to_dx( float x )
{
	float xa  = abs(x);
	if( xa < 0.1 ) return 0.001;
	if( xa < 10  ) return 0.01;
	if( xa < 100 ) return 0.1;
	if( xa < 1e3 ) return 1;
	if( xa < 1e4 ) return 10;
	return 100;
}

// dd = decimal digits 0..4
#define MSG__Old  "Old"
#define MSG__New  "New"

// todo: use left align
// since left button is used to change step size, show step size on the left ? "> d=" 
float set_float( float X, char *s, uint8_t dd, float max, float min, float dx )
{
	float newX = X;
	int   dx_inc = 1;
	char  sdx[Nstr];

	if( dx < 1e-6 ){   // 0.0 for auto set dx
		dx = x_to_dx( X );
		if( dd == 0 && dx < 1   )  dx = 1;   // zero deciaml point
		if( dd == 1 && dx < 0.1 )  dx = 0.1;
		if( dd == 2 && dx < 0.01 ) dx = 0.01;
	}
	
	scroll(  gbuf[0], s, 1 );  // sprintf( gbuf[0], "%s", s );
	switch( dd ){
	case 0:
		sprintf( gbuf[3], "%s %10.0f" , MSG__Old, X );  // NOTE: formating depends on language !  FNC has more chars
		sprintf( sdx, "d=%.0f",  dx  );
		break;
	case 1:
		sprintf( gbuf[3], "%s %10.1f" , MSG__Old, X );
		sprintf( sdx, "d=%.1f",  dx  );
		break;
	case 2:
 		sprintf( gbuf[3],  "%s %10.2f" , MSG__Old, X );
		sprintf( sdx, "d=%.2f",  dx  );
		break;
	case 3:
		sprintf( gbuf[3], "%s %10.3f" , MSG__Old, X );
		sprintf( sdx, "d=%.3f",  dx  );
		break;
	case 4:
		sprintf( gbuf[3], "%s %10.4f" , MSG__Old, X );
		sprintf( sdx, "d=%.4f",  dx  );
		break;
	}
		
	sprintf( gbuf[1], ">%13s", sdx  );  // LANG WISH: arrow to indicate digit being changed
		
	int tchLcnt = 0;
	float savedX = max * 2;
	clearBtnTouch();
	while( ! btnClickedRlsd() && ! touchedR ){

		scroll(  gbuf[0], s, 0 );
		
		if( touchedL ){
			touchedL = 0;
			tchLcnt++;
						
			if (dx_inc) dx *= 10;
			else if( (dd != 0 && dd < 3 && dx > 0.01 ) ||
					 (dd >= 3 && dx > 0.0001 ) || (dd == 0 && dx > 1) ) dx /= 10;

			if( dd == 4 )      sprintf( sdx, "d=%.4f", dx );
			else if( dd == 3 ) sprintf( sdx, "d=%.3f", dx );
			else if( dd == 2 ) sprintf( sdx, "d=%.2f", dx );
			else if( dd == 1 ) sprintf( sdx, "d=%.1f", dx );
			else               sprintf( sdx, "d=%.0f", dx );

			sprintf( gbuf[1], "%14s", sdx  );
			u8g_print( gbuf[0], gbuf[1], gbuf[2], gbuf[3] );
			
			if ( dd < 3 && dx > abs(newX)/10 ) dx_inc = 0;
			if ( dd > 2 && dx > 0.2 ) dx_inc = 0;   // allow 1 degree chnage in taper angle
			
			if ( dd >= 3 && dx < 0.0002 )  dx_inc = 1;
			if ( dd > 0 && dd < 3 && dx < 0.02 )  dx_inc = 1;    // <0.1 has resolution error
			if ( dd == 0 && dx < 2 )  dx_inc = 1;
		}

		int32_t r = readEncoder(16);  // no 2nd read, accel not implemented
		
		if( r ) tchLcnt = 0;  // touchedL N times w/o rotate the click wheel
		                      // to clear value to zero or back to old saved value
		if( tchLcnt > 10 ){   
			if( savedX > max ){
				savedX = newX;
				  newX = 0.0;
			}else{
				newX   = savedX;
				savedX = 2*max;
			}
			tchLcnt = 0;
		}
		else{
			newX += r * dx / 2;   // one click=2
		}
		
		if( newX < min ){ newX= min; oled_flash(); }
		if( newX > max ){ newX= max; oled_flash(); }
		
		if( dd == 4 )      sprintf( gbuf[2], "%s %9.4f" , MSG__New, newX );
		else if( dd == 3 ) sprintf( gbuf[2], "%s %9.3f" , MSG__New, newX );
		else if( dd == 2 ) sprintf( gbuf[2], "%s %9.2f" , MSG__New, newX );
		else if( dd == 1 ) sprintf( gbuf[2], "%s %9.1f" , MSG__New, newX );
		else               sprintf( gbuf[2], "%s %9.0f" , MSG__New, newX );
		u8g_print( gbuf[0], gbuf[1], gbuf[2], gbuf[3] );
	}
	if( btnClicked() ) X = newX;  // touch will keep old value, only click will take new value
	clearBtnTouch();    // touchL=step size, touchR=cancel, can be confusing
	return X;
}

#define INCH2MM    (25.4)
#define MM2INCH    (1.0/25.4)

#define NDDI  3     // number of displayed digits in inch mode
#define NDDM  2     // number of displayed digits in metric mode

float set_float_auto_unit( float Xx, char *s, float max, float min, float dx ){
    char ttl[Nstr];
    sprintf( ttl, "%s %s", s, (gc_state.modal.units == Units::Mm) ? "(mm)" : "(inch)" );    // TODO: language

    if( gc_state.modal.units == Units::Mm  )
        return set_float( Xx,        ttl, NDDM, max, min, dx );
    else
        return set_float( Xx*MM2INCH, ttl, NDDI, max*MM2INCH, min*MM2INCH, dx ) * INCH2MM;
}

//---------------------------------------------------------------------------------------
void ez_set_pos()
{
    log_info( "set pos");
        // todo: start with current position
        //   want a quick way to enter zero of single axis

       //sprintf( msg, "Set As (%s)", (EZnc.Unit==INCH) ? "inch" : "mm" );
        //ez_enter_pos( msg, (char *) "click to set" );
        //if( btnClicked() ){
        //    Serial.print( "[MSG:ezSetPos-" );    // add [MSG so that ugs doesn't complain                           
        //    sprintf( eznc_line, "G10L20P1X%fY0Z%f", npos[0], npos[1] );
        //    report_status_message(gc_execute_line(eznc_line, CLIENT_SERIAL), CLIENT_SERIAL);
        //}

    int8_t sel=1, smin=1;
    char menu[10][Nstr] = {
        "Set Position",
        "Back to DRO",  // missed comma will pass compilier
        "X=Y=Z = 0",
        "X=Y = 0",
        "X = 0",
        "Y = 0",
        "Z = 0",
        "X",
        "Y",
        "Z"
    };
    char msg[Nstr];
    float val, *pos;

    for(;;){
        pos = get_mpos();
        mpos_to_wpos(pos);
        // todo: add position value to menu string

        clearBtnTouch();
        select_from_menu( 10, menu, &sel, &smin );  // blocking
        if( touchedR ) return;

        switch(sel){
            case 1:
                clearBtnTouch();
                return;
            case 2:  // X/Y/Z=0
                if( confirm( (char *)"Set X=Y=Z=0") ){
                     sprintf( eznc_line, "G10L20P1X0Y0Z0");
                     gc_execute_line(eznc_line, Uart0);
                }
                break;
            case 3:  // X/Y=0
                if( confirm( (char *)"Set X=Y=0") ){
                     sprintf( eznc_line, "G10L20P1X0Y0");
                     gc_execute_line(eznc_line, Uart0);
                }
                break;
            case 4:  // X=0
                if( confirm( (char *)"Set X=0") ){
                     sprintf( eznc_line, "G10L20P1X0");
                     gc_execute_line(eznc_line, Uart0);
                }
                break;
            case 5:  // Y=0
                if( confirm( (char *)"Set Y=0") ){
                     sprintf( eznc_line, "G10L20P1Y0");
                     gc_execute_line(eznc_line, Uart0);
                }
                break;
            case 6:  // Z=0
                if( confirm( (char *)"Set Z=0") ){
                     sprintf( eznc_line, "G10L20P1Z0");
                     gc_execute_line(eznc_line, Uart0);
                }
                break;
            case 7:  // X=?
                val = pos[0];
                val = set_float_auto_unit( val, (char *)"Set X= " );                
                sprintf( eznc_line, "G10L20P1X%.4f", val );   // always set
                gc_execute_line(eznc_line, Uart0);
                break;
            case 8:  // Y=?
                val = pos[1];
                val = set_float_auto_unit( val, (char *)"Set Y= " );                
                sprintf( eznc_line, "G10L20P1Y%.4f", val );   // always set
                gc_execute_line(eznc_line, Uart0);
                break;
            case 9:  // Z=?
                val = pos[2];
                val = set_float_auto_unit( val, (char *)"Set Z= " );                
                sprintf( eznc_line, "G10L20P1Z%.4f", val );   // always set
                gc_execute_line(eznc_line, Uart0);
                break;
        }
    }
}

void ez_menu()   // top level ui menu, only title line is auto-scrolled
{
    int8_t sel=1, smin=1, junk=1;
    int progress=0, old_progress=0;

    char menu[6][Nstr] = {
        "ezFluidNC",
        "Set Pos",
        "Run G-code",
        "Change Unit",
        "Back to DRO",
        "Reset WebUi"
    };
    char msg[Nstr];

    clearBtnTouch();
    select_from_menu( 6, menu, &sel, &smin );  // blocking
    if( touchedR ) return;

    switch(sel){
    case 1:
        ez_set_pos(); 
        return;
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
                ez_check_cancel = true; 
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

    if( btnClicked() || touched() ){ // todo: individual button clear
  
        if( touchedL ){
            jog_axis = (jog_axis+1) % 3;
            update_dro = 1;
            log_info( "SWL jog axis " << jog_axis );
        }
            
        if( touchedR ){
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
        clearBtnTouch();            
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

    if( btnClicked() || touched() ){
        if( touchedL ){
        }
            
        if( touchedR ){
        }
            
        if( clickCounterSW1 ){
            log_info( "exit ui menu..." );
            uimenu_active = 0;
            update_dro = 1;
        }
    }
    clearBtnTouch();
}

// called by protocal loop, try to be as short as possible
// TODO; disable jogging when running ?

// gee, once gcode file is started fron eznc, this is not called
// bcz (different from esp_grbl) Protocol loop structure

bool gcode_started = false;

void eznc_dispatch( void )    // top level dispatcher
{    
    lcnt++;
    //if( (lcnt % 1000) == 0) log_warn( "HJL: eznc_dispatch, loop " << lcnt );

    if( ez_check_cancel && sys.state == State::Cycle ) gcode_started = true;
    if( ez_check_cancel && gcode_started && sys.state == State::Idle  ){
        ez_check_cancel = false;
        gcode_started = false;
    }

    if( uimenu_active )
        ez_ui();        
    else
        ez_dro();
}

