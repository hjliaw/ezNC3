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

String ez_select_file( String p );

extern volatile int uimenu_active;
extern volatile int update_dro;
extern volatile int update_menu;

extern char   ui_txt[4][Wchars+1];
extern int    ui_sel, ui_frame;

String ez_select_file();
String ez_gcfn;

volatile bool ez_check_cancel =0;
volatile bool ez_run_pwrfd =0;

char   gbuf[16][Nstr];
unsigned long jog_t0, jog_cancel_t = 0;
char eznc_line[LINE_BUFFER_SIZE];
int cancelJog;
int jog_axis = 0;   // 0..2 for now                                                                                      
int ezJog = 0;
int32_t enc_cnt = 0;

// todo: save parameters permamently (in localfs)

int   jog_stepsize = 1000;
bool jss_inc = false;

long run_t0;

float mark_A[3] = {0, 0, 0};
float mark_B[3] = {0, 0, 0};
float pos_entered[3];

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
// littlefs, with one level of directory navigation 

#define MAX_GCNAME 512
#define MAX_DIRNAME 64
String gcname[MAX_GCNAME];   // can this be unlimited ?
String dirname[MAX_DIRNAME];
int    Ngcf = 0;
int    Ndir = 0;

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

bool is_gcode( std::string const & p ) {
    return 
    ( has_ending( p,  ".gc")  ||
      has_ending( p,  ".ngc") ||
      has_ending( p,  ".GC")  ||
      has_ending( p,  ".NGC") );
}

void listLFS( const char* path);

void listLFS( const char* path ){  // list local file system, save file names in gcname, set Ngcf
    char fname[256];
    Ngcf = Ndir = 0;

    std::error_code ec;
    FluidPath fpath { path, "littlefs", ec };
    if (ec){
        log_error( "no littlefs found, " << ec.message() );                
        return;
    }

    auto iter = stdfs::recursive_directory_iterator { fpath, ec };
    if (ec) {
        log_error( ec.message() );
        return;
    }

    for (auto const& dir_entry : iter) {
        String fname = String( dir_entry.path().c_str() );
        fname.replace( "/littlefs/", "");

        if( strcmp( path, "/") != 0  ){   // 0=equal
            fname.replace( path, "");     // will replace All occurence !
        }

        if ( dir_entry.is_directory() ){
            gcname[Ngcf++] = fname + "/";
        }
        else{
            if( is_gcode(dir_entry.path()) && fname.indexOf("/") < 0 )
                gcname[Ngcf++] = fname;
        }
        if (Ngcf >= MAX_GCNAME-4) break;  // avoid overflow
    }
}

//---------------------------------------------------------------------------------
// eznc jogging
//---------------------------------------------------------------------------------

// try 10x step size

void ez_jog( int32_t rot ){
    char axis[3] = {'X', 'Y', 'Z'};
    static int32_t odir;
    if( rot == 0 ) return;
    jog_t0 = millis();

    if( odir * rot < 0 ){  // reversal                                                                                 
        cancelJog = 1;                                                               
        // some encoders have big jump on reversal                                                                 
        rot = (rot > 0) ? 1 : -1;   // can't just clear, hack it                                                       
    }
    else{
        float jstep;
        if( gc_state.modal.units == Units::Mm )
            jstep =  0.01*jog_stepsize*rot/2.0;     // two ticks per detent
        else
            jstep = 0.0254*jog_stepsize*rot/2.0;

        sprintf( eznc_line, "$J=G21G91%c%.4fF%.1f", axis[jog_axis], jstep, EZnc.jog_speed );

        log_warn( "DBG: " << eznc_line );
        //report_status_message(gc_execute_line(eznc_line, Uart0), Uart0);
        gc_execute_line(eznc_line, Uart0);
        ezJog = 1;                                             
    }
    odir = rot;
}

void ez_cancel_jog(){
    log_warn("DBG: ez cancel jog");
    if (sys.state == State::Jog ) {
        protocol_send_event(&motionCancelEvent);                                                                   
    }
    ezJog = 0;  // not sure necessary
    jog_cancel_t = millis();
}

//-------------------------------------------------------------------------

#ifdef INCLUDE_OLED_IO

String ez_select_file( String p ){

    listLFS( p.c_str() );

    if( Ngcf == 0 ){
        clearBtnTouch();
        u8g_print(  (char *) "No gc|ngc file",    // prepare for scrolling text
                    (char *) "Upload va WiFi",
                    (char *) "",
                    (char *) "clck to cont" );
        while( ! btnClickedRlsd() && ! touchedR );
        clearBtnTouch();
        return "";
    }
        
    // try a new way to do sel menu, as number of file entries can be >16
    int sel =0, smin=0;
    int directory = -1;  // -1=root, 1..Ndir  maps to ndirname[0..Ndir-1]

    while( ! btnClickedRlsd() && ! touchedR ){
        for( int i=0; i<4; i++ ) gcname[Ngcf+i] = "  ";
        //strncpy( gbuf[0], strcat("Gcode ", p.c_str()), Nstr );   // crashed ?
        strncpy( gbuf[0], "Gcode ", Nstr );
        strncpy( gbuf[0]+6, p.c_str(), Nstr-6 );

        for( int i=1; i<4; i++ )
            // eznc first char='/', but is removed here
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

    if( gcname[sel].endsWith("/"))
        return gcname[sel] + ez_select_file( gcname[sel] );
    else
        return gcname[sel];

    // confirm sometimes gets in my way !

    //if( confirm( (char *)"Run g-code", (char *)gcname[sel].c_str() ))
    //        return gcname[sel];
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
            //log_debug( "sv= " << sv << ",txt= " << ptr[0] );
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
// TODO: allow step size to go higher

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
		sprintf( gbuf[1], ">d=%11.0f",  dx  );
		break;
	case 1:
		sprintf( gbuf[3], "%s %10.1f" , MSG__Old, X );
		sprintf( gbuf[1], ">d=%11.1f",  dx  );
		break;
	case 2:
 		sprintf( gbuf[3],  "%s %10.2f" , MSG__Old, X );
		sprintf( gbuf[1], ">d=%11.2f",  dx  );
		break;
	case 3:
		sprintf( gbuf[3], "%s %10.3f" , MSG__Old, X );
		sprintf( gbuf[1], ">d=%11.3f",  dx  );
		break;
	case 4:
		sprintf( gbuf[3], "%s %10.4f" , MSG__Old, X );
		sprintf( gbuf[1], ">d=%11.4f",  dx  );
		break;
	}
		
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

			if( dd == 4 )      sprintf( gbuf[1], ">d=%11.4f", dx );
			else if( dd == 3 ) sprintf( gbuf[1], ">d=%11.3f", dx );
			else if( dd == 2 ) sprintf( gbuf[1], ">d=%11.2f", dx );
			else if( dd == 1 ) sprintf( gbuf[1], ">d=%11.1f", dx );
			else               sprintf( gbuf[1], ">d=%11.0f", dx );

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
		
		if( dd == 4 )      sprintf( gbuf[2], "%s %10.4f" , MSG__New, newX );
		else if( dd == 3 ) sprintf( gbuf[2], "%s %10.3f" , MSG__New, newX );
		else if( dd == 2 ) sprintf( gbuf[2], "%s %10.2f" , MSG__New, newX );
		else if( dd == 1 ) sprintf( gbuf[2], "%s %10.1f" , MSG__New, newX );
		else               sprintf( gbuf[2], "%s %10.0f" , MSG__New, newX );
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


bool ez_enter_XY( char *title, float *x, float *y )
{
    int dd = 100, ddinc=1;      // step size multiplier
    float step, np[2];
    char mstr[Nstr];
    int sel = 0;    // 0=x, 1=y, 2=status/message
    int update;
    uint8_t oled_y_pos;

    update_menu = 0;  // stop oled task, take over
    np[0] = *x;
    np[1] = *y;

    if( gc_state.modal.units == Units::Mm )
        step = 0.01;  // derive from EZnc.NDDM ?
    else
        step = 0.001;

    clearBtnTouch();
    update = 1;  // draw 1st screen
    while( !btnClicked() ){    // touchR is step size now !

        if( touchedL ){
            sel = (sel+1) % 3;
            touchedL = 0;
            update = 1;
        }

        if( touchedR ){
            if( ddinc ) dd *= 10;
            else        dd /= 10;
            if( dd > 1000 ){
                ddinc = 0;  dd = 100;
            }
            if( dd < 1){
                ddinc = 1;  dd = 10;
            }
            touchedR = 0;
            update = 1;
        }

        enc_cnt = readEncoder(1);  // 1=no double reads
        if( enc_cnt != 0 && sel < 2){
            np[sel] += enc_cnt * step * dd / 2;
            update = 1;
        }

        if( gc_state.modal.units == Units::Mm )
            sprintf( mstr, "%.2f %s", step*dd, ddinc ? "<<" : ">>" );
        else
            sprintf( mstr, "%.3f %s", step*dd, ddinc ? "<<" : ">>"  );

        if( update ){
            oled->clear();
            oled->setFont(DejaVu_Sans_Mono_14);

            oled->setTextAlignment(TEXT_ALIGN_LEFT);   // may need smaller font
            oled->drawString(0, 0, title );
            oled->setTextAlignment(TEXT_ALIGN_RIGHT);
            oled->drawString(127, 0, mstr );

            for (uint8_t a = 0; a < 2; a++) {   // fixed for eznc
                oled_y_pos = 18 + a*15;

                String a_name = ((a==sel)? ">":" ") + String(Machine::Axes::_names[a]) + "= ";
                oled->setTextAlignment(TEXT_ALIGN_LEFT);
                oled->drawString(0, oled_y_pos, a_name);

                oled->setTextAlignment(TEXT_ALIGN_RIGHT);
                if( gc_state.modal.units == Units::Mm )
                    snprintf( mstr, 18, "%.2f mm", np[a]);
                else
                    snprintf( mstr, 18, "%.3f in", np[a]);
                oled->drawString( 127, oled_y_pos, mstr);
            }
            // draw last line 
            oled_y_pos = 18 + 2*15;
            if( sel != 2 ) snprintf( mstr, 19, " click to go/set" );
            else           snprintf( mstr, 19, ">click to cancel" );
            oled->setTextAlignment(TEXT_ALIGN_LEFT);
            oled->drawString( 0, oled_y_pos, mstr);
            oled->display();
            update = 0;
        }
    }

    // btn clicked
    update_menu = 1;  // resume oled task
    if( sel == 2 ) return false;
    *x = np[0];  *y = np[1];
    return true;
}

bool ez_enter_Z( char *title, float *z )
{
    int dd = 100, ddinc=1;      // step size multiplier, flag to inc/dec
    float step, nz;
    char mstr[Nstr];
    int sel = 1;
    int update;
    uint8_t oled_y_pos;

    update_menu = 0;  // stop oled task, take over
    nz = *z;

    if( gc_state.modal.units == Units::Mm )       // save as EZnc.step and compute once
        step = 0.01;  // derive from EZnc.NDDM ?
    else
        step = 0.001;

    clearBtnTouch();
    update = 1;  // draw 1st screen
    while( !btnClicked() ){    // touchR is step size now !

        if( touchedL ){
            sel = 1 + sel % 2;  // limit to 1 or 2
            touchedL = 0;
            update = 1;
        }

        if( touchedR ){
            if( ddinc ) dd *= 10;
            else        dd /= 10;
            if( dd > 1000 ){
                ddinc = 0;  dd = 100;
            }
            if( dd < 1){
                ddinc = 1;  dd = 10;
            }
            touchedR = 0;
            update = 1;
        }

        enc_cnt = readEncoder(1);  // 1=no double reads
        if( enc_cnt != 0 && sel < 2){
            nz += enc_cnt * step * dd / 2;
            update = 1;
        }

        if( gc_state.modal.units == Units::Mm )
            sprintf( mstr, "%.2f %s", step*dd, ddinc ? "<<" : ">>" );
        else
            sprintf( mstr, "%.3f %s", step*dd, ddinc ? "<<" : ">>" );

        if( update ){
            oled->clear();
            oled->setFont(DejaVu_Sans_Mono_14);

            oled->setTextAlignment(TEXT_ALIGN_LEFT);   // may need smaller font
            oled->drawString(0, 0, title );
            oled->setTextAlignment(TEXT_ALIGN_RIGHT);
            oled->drawString(127, 0, mstr );

            // z-only, sel=1 or 2
            uint8_t a = 1;
            oled_y_pos = 18 + a*15;

            String a_name = ((a==sel)? ">Z=":" Z=");
            oled->setTextAlignment(TEXT_ALIGN_LEFT);
            oled->drawString(0, oled_y_pos, a_name);

            oled->setTextAlignment(TEXT_ALIGN_RIGHT);
            if( gc_state.modal.units == Units::Mm )
                snprintf( mstr, 18, "%.2f mm", nz);
            else
                snprintf( mstr, 18, "%.3f in", nz);
            oled->drawString( 127, oled_y_pos, mstr);

            // draw last line 
            oled_y_pos = 18 + 2*15;
            if( sel != 2 ) snprintf( mstr, 19, " click to go/set" );
            else           snprintf( mstr, 19, ">click to cancel" );
            oled->setTextAlignment(TEXT_ALIGN_LEFT);
            oled->drawString( 0, oled_y_pos, mstr);
            oled->display();
            update = 0;
        }
    }

    // btn clicked
    update_menu = 1;  // resume oled task
    if( sel == 2 ) return false;
    *z = nz;
    return true;
}

void ez_config()
{
#define Nm 9
    int8_t sel=1, smin=1;
    char menu[Nm][Nstr] = {
        "Configuration",
        "Go Back",
        "Change Unit",
        "Run Speed",
        "Jog Speed",
        "Tool Diameter",
        "Flip Screen",
        "Flip UI encoder",
        "Save Config",
    };
    char msg[Nstr];
    float val, *pos;

  for(;;){
    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );  // blocking
    if( touchedR ) return;

    switch(sel){
        case 1:
            clearBtnTouch();
            return;
        case 2:  // change unit, todo: ask for confirmation ?                                                             
            if( gc_state.modal.units == Units::Mm )
                sprintf( eznc_line, "G20" );
            else
                sprintf( eznc_line, "G21" );
            gc_execute_line( eznc_line, Uart0);
            return;
        case 3:  // run speed
            EZnc.run_speed = set_float( EZnc.run_speed, (char *)"Run Speed (mm/min)", 0, 1500, 10, 10 );
            break;
        case 4:  // jog speed
            EZnc.jog_speed = set_float( EZnc.jog_speed, (char *)"Jog Speed (mm/min)", 0, 1500, 10, 10 );
            break;
        case 5: // tool diameter
            EZnc.tool_dia = set_float( EZnc.tool_dia, (char *)"Tool Dia. (mm)", 3, 100, 0.1, 1 );
            break;
        case 6: // flip screen
            if( confirm( (char *)"Flip Screen ?") ){
                EZnc.FlipScreen = ! EZnc.FlipScreen;
                if( EZnc.FlipScreen )
                    oled->flipScreenVertically();
                else
                    oled->resetOrientation();
            }
            break;
        case 7: // flip ui encoder direction
            if( confirm( (char*) "Flip MPG Encoder ?") ){
                EZnc.UiEncDir = ! EZnc.UiEncDir;
            }
            break;
        case 8: // save config
            if( confirm( (char*) "Save Config ?") )
                save_eznc_eeprom();
            break;
    }
  }
#undef Nm
}

void ez_set_AB()
{
    int8_t sel=1, smin=1;
    float *pos;
#define Nm 4
    char menu[Nm][Nstr] = {
        "Set Pos As",
        "cancel",
        "point B",
        "point A"   };

    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );  // blocking
#undef Nm

    pos = get_mpos();
    mpos_to_wpos(pos);   // B/A are saved in mm

    if( touchedR ) return;
    switch(sel){
        case 1:
            return;
        case 2:   // B first
            for( int i=0; i<3; i++) mark_B[i] = pos[i];
            break;
        case 3:
            for( int i=0; i<3; i++) mark_A[i] = pos[i];
            break;
    }
    return;
}

void ez_set_Zero()
{
    int8_t sel=1, smin=1;
    float *pos;
#define Nm 7 
    char menu[Nm][Nstr] = {
        "Set Pos As",
        "cancel",
        "Xo,Yo,Zo",
        "Xo,Yo",
        "Xo",
        "Yo",
        "Zo"
    };
    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );  // blocking
#undef Nm

    pos = get_mpos();
    mpos_to_wpos(pos);

    if( touchedR ) return;
    switch(sel){
        case 1:
            return;
        case 2:  // X/Y/Z=0
            if( ! confirm( (char *)"Set as X=Y=Z=0") ) return;
            sprintf( eznc_line, "G10L20P1X0Y0Z0");
            break;
        case 3:  // X/Y=0
            if( ! confirm( (char *)"Set as X=Y=0") ) return;
            sprintf( eznc_line, "G10L20P1X0Y0");
            break;
        case 4:  // X=0
            if( ! confirm( (char *)"Set as X=0") ) return;
            sprintf( eznc_line, "G10L20P1X0");
            break;
        case 5:  // Y=0
            if( ! confirm( (char *)"Set as Y=0") ) return;
            sprintf( eznc_line, "G10L20P1Y0");
            break;
        case 6:  // Z=0
            if( ! confirm( (char *)"Set as Z=0") ) return;
            sprintf( eznc_line, "G10L20P1Z0");
            break;
    }
    gc_execute_line(eznc_line, Uart0);

    return;
}

//---------------------------------------------------------------------------------------
void ez_set_pos()
{
    int8_t sel=1, smin=1;

#define Nm 6
    char menu[Nm][Nstr] = {
        "Set As",
        "B or A",
        "Xo/Yo/Zo",
        "XY=",
        "Z=",
        "cancel",
        };
    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );  // blocking
#undef Nm

    if( touchedR ) return;

    float x,y,z;
    float *pos = get_mpos();
    mpos_to_wpos(pos);

    switch(sel){
        case 1:  ez_set_AB();   return;
        case 2:  ez_set_Zero(); return;
        case 3:  //XY
            x = pos[0];  y=pos[1];
            if( gc_state.modal.units == Units::Inches ){ x = pos[0]*MM2INCH; y = pos[1]*MM2INCH; }

            if( ! ez_enter_XY( (char *)"Set As", &x, &y) )  return;
            sprintf( eznc_line, "G10L20P1X%.4fY%.4f", x, y );
            gc_execute_line(eznc_line, Uart0);
            return;
        case 4:  // Z
            z = pos[2];
            if( gc_state.modal.units == Units::Inches ){ z = pos[0]*MM2INCH; }

            if( ! ez_enter_Z( (char *)"Set As", &z) )  return;
            sprintf( eznc_line, "G10L20P1Z%.4f", z );
            gc_execute_line(eznc_line, Uart0);
            return;

        case 5:  return;
    }
}

char pfmsg[4][Nstr];
bool cmd_issued = false;
bool cmd_started = false;
bool cmd_finished = true;   // initial state
int  cmd_cnt = 0;

void ez_pwr_fd_reset()
{
    cmd_finished = true;
    cmd_issued   = false;
    cmd_started  = false;
    cmd_cnt = 0;

    ez_run_pwrfd = false;
    uimenu_active = 0;
    update_dro = 1;
    clearBtnTouch();  // to stay in pwr fd, need to preserve touch
}

void push_gcode( String gc )
{
    if(cmd_cnt < 500 ){
        gcname[cmd_cnt++] = gc;
        log_info( "DBG PF: " << gc );
    }
    else
        log_warn( "too many g-code lines, some are dropped");
}

void dbg_pos()
{
    float* px = get_mpos();
    log_info( "DBG mpos= " <<  px[0] << ", " << px[1] << ", " << px[2] );
    mpos_to_wpos(px);
    log_info( "    wpos= " <<  px[0] << ", " << px[1] << ", " << px[2] );
}

// quirk: run pwr fd after reset, screen will stay in run pwr fd
// sometimes, stay in uimenu ?

// return to origin ?
// 2D: lift Z a little (custom Zlift, Zsafe ?)
// 1D: lift or move away (too many options)

// TODO: add spiral-in, spiral-out option -> pocket cuts

void ez_pwr_fd()        // XY only, move between A/B  1d or 2d
{
    static int cmd_idx;

    if( touchedR ){
        sys.abort = true;
        ez_pwr_fd_reset();
        return;
    }

    if( !ez_run_pwrfd ) return;

    // called by dispatcher repeatedly

    if( cmd_cnt == 0 && sys.state == State::Idle && !cmd_issued ){  // init
        float dx, dy, dA, dB;
        dx = mark_A[0] - mark_B[0];
        dy = mark_A[1] - mark_B[1];

        dbg_pos();

        // where am I, closer to mark-A or mark-B
        float* pos = get_mpos();
        mpos_to_wpos(pos);

        dA =  (pos[0]-mark_A[0])*(pos[0]-mark_A[0])
            + (pos[1]-mark_A[1])*(pos[1]-mark_A[1]);

        dB =  (pos[0]-mark_B[0])*(pos[0]-mark_B[0])
            + (pos[1]-mark_B[1])*(pos[1]-mark_B[1]);

        if( fabs(dx) < 1e-4 || fabs(dy) < 1e-4 ){   // simple x or y feed
            if( fabs(dy) < 1e-4 )
                sprintf( pfmsg[0], "Power Feed X" );
            else
                sprintf( pfmsg[0], "Power Feed Y" );
                        
            if( dB < dA ){
                sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", mark_A[0], mark_A[1], EZnc.run_speed );
                push_gcode( String( eznc_line ) );
                sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", mark_B[0], mark_B[1], EZnc.run_speed );
                push_gcode( String( eznc_line ) );
            }
            else{
                sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", mark_B[0], mark_B[1], EZnc.run_speed );
                push_gcode( String( eznc_line ) );
                sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", mark_A[0], mark_A[1], EZnc.run_speed );
                push_gcode( String( eznc_line ) );
            }
        }
        else{  // 2D power feed, move to nearest corner first
            float p0[2], p1[2];
            sprintf( pfmsg[0], "Power Feed X/Y" );

            for( int i=0; i<2; i++ ){
                dA = fabs(pos[i]-mark_A[i]);
                dB = fabs(pos[i]-mark_B[i]);
                if( dA < dB ){
                    p0[i] = mark_A[i];  p1[i] = mark_B[i];
                } else {
                    p0[i] = mark_B[i];  p1[i] = mark_A[i];
                }
            }
            sprintf( eznc_line, "X/Y  p0=[%.2f, %.2f]  p1=[%.2f, %.2f", p0[0], p0[1], p1[0], p1[1]);
            log_warn( eznc_line );


            // sweep x-first, 
            dx = p1[0] - p0[0];
            dy = p1[1] - p0[1];

            float dd = EZnc.tool_dia * 0.85;  // diameter (with overlap)
            int Ny = 1 + int(fabs(dy/dd));
            float ystep = dy / Ny;

            log_info( "     dx=" << dx );
            log_info( "     dy=" << dy );
            log_info( "  ystep=" << ystep );
            log_info( "     Ny=" << Ny );

            // TODO: new pocket function to allow spiral in/out & fancier options

            sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", p0[0], p0[1], EZnc.run_speed );
            push_gcode( String( eznc_line ) );

            sprintf( eznc_line, "G91G1X%.4fF%.0f", dx, EZnc.run_speed );  // first x-cut
            push_gcode( String( eznc_line ) );

            for( int i=0; i < Ny; i++ ){
                    sprintf( eznc_line, "G91G1Y%.4fF%.0f", ystep, EZnc.run_speed );
                    push_gcode( String( eznc_line ) );
                    sprintf( eznc_line, "G91G1X%.4fF%.0f", (i%2==0) ? -dx:+dx, EZnc.run_speed );
                    push_gcode( String( eznc_line ) );
            }
        }

        // if user clicks pwr feed w/o moving
        if( fabs(dx) < 1e-4 && fabs(dy) < 1e-4 ){
            sprintf( pfmsg[0], "Points A == B" );
            pfmsg[1][0] = 0;
            sprintf( pfmsg[2], "nothing to do" );
            sprintf( pfmsg[3], "touchR to cont." );
            u8g_print( pfmsg[0], pfmsg[1], pfmsg[2], pfmsg[3] );

            clearBtnTouch();   // only touch-R works ? why ?
            while( ! btnClickedRlsd() && !touched()  );
            clearBtnTouch();
            return;
        }

        cmd_idx = 0;
        pfmsg[1][0] = pfmsg[2][0] = 0;
        sprintf( pfmsg[3], "cancel = touchR" );
        u8g_print( pfmsg[0], pfmsg[1], pfmsg[2], pfmsg[3] );

#if 0
        log_info( pfmsg[0] );
        log_info( "cmd list");
        for( int i=0; i<cmd_cnt; i++){
            log_info( "  " << i << "  " << gcname[i] );
        }
#endif
    }

    // try dual cmd, single loop
    // a simple g-code sender here

    if( ! touchedR && sys.state == State::Idle && ! cmd_issued ){
    
        // two commands a time
        if( cmd_idx < cmd_cnt ){
            strncpy( eznc_line, gcname[cmd_idx++].c_str(), 256 );
            gc_execute_line( eznc_line, Uart0);
        }
        if( cmd_idx < cmd_cnt ){
            strncpy( eznc_line, gcname[cmd_idx++].c_str(), 256 );
            gc_execute_line( eznc_line, Uart0);
        }

        cmd_issued = true;
        cmd_started = false;
        cmd_finished = false;
    }

    if( cmd_issued && ! cmd_started && sys.state == State::Cycle ){   // started
        cmd_started = true;
    }

    if( cmd_started && sys.state == State::Idle ){   // ready for more command
        if( cmd_idx >= cmd_cnt ){
            // perhaps, show cancel at upper right corner, repeat at lower left

            dbg_pos();

            sprintf( pfmsg[0], "Power Feed Done" );
            pfmsg[1][0] = 0;
            sprintf( pfmsg[2], "cancel = touchR" );
            sprintf( pfmsg[3], "repeat = click" );
            u8g_print( pfmsg[0], pfmsg[1], pfmsg[2], pfmsg[3] );

            // disable motor to allow user intervene, such as lower z-axis
            // NOT WORKING protocol_disable_steppers();
            config->_axes->set_disable(true);

            clearBtnTouch();
            while( !touchedR && ! btnClicked() ){
                delay(100);
            }

            // touchedR handled by isr
            
            if( btnClicked() ){  // auto repeat by dispatcher
                cmd_finished = true;
                cmd_issued   = false;
                cmd_started  = false;
                cmd_cnt = 0;
            }
        }
        else{ // ?
            cmd_issued = false;
            cmd_started = false;
            cmd_finished = true;
        }
    }
}

void ez_goto_AB()
{
    int8_t sel=1, smin=1;
#define Nm 4
    char menu[Nm][Nstr] = {
        "Goto B or A",
        "cancel",
        "B",
        "A"   };

   // show coordinate, but even 18 chars is not enough to show unit
    if( gc_state.modal.units == Units::Mm ){
        sprintf( menu[2], "B %5.1f, %-5.1f", mark_B[0], mark_B[1] );
        sprintf( menu[3], "A %5.1f, %-5.1f", mark_A[0], mark_A[1] );
    }
    else{
        sprintf( menu[2], "B %5.2f, %-5.2f", mark_B[0]*MM2INCH, mark_B[1]*MM2INCH );
        sprintf( menu[3], "A %5.2f, %-5.2f", mark_A[0]*MM2INCH, mark_A[1]*MM2INCH );
    }

    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );  // blocking
#undef Nm

    if( touchedR ) return;
    switch(sel){
    case 1:
            clearBtnTouch();
        return;
    case 2:   // B first, B/A are saved in mm (unlike eznc)
        if( gc_state.modal.units == Units::Mm )
            sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", mark_B[0], mark_B[1], EZnc.run_speed );
        else
            sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", mark_B[0]*MM2INCH, mark_B[1]*MM2INCH, EZnc.run_speed );
        break;
    case 3:
        if( gc_state.modal.units == Units::Mm )
            sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", mark_A[0], mark_A[1], EZnc.run_speed );
        else
            sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", mark_A[0]*MM2INCH, mark_A[1]*MM2INCH, EZnc.run_speed );
        break;
    }
    gc_execute_line(eznc_line, Uart0);
}

void ez_goto_XYZ0()
{
    int8_t sel=1, smin=1;
#define Nm 7
    char menu[Nm][Nstr] = {   // if stay here, menu can be simplified
        "Goto",
        "cancel/return",
        "Xo",
        "Yo",
        "Xo,Yo",
        "Zo",
        "Xo,Yo,Zo"
    };
    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );
#undef Nm

    if( touchedR ) return;
    switch(sel){
        case 1:
            clearBtnTouch();
            return;
        case 2:  // X=0
            sprintf( eznc_line, "G90G1X0F%.0f", EZnc.run_speed );
            break;
        case 3:  // Y=0
            sprintf( eznc_line, "G90G1Y0F%.0f", EZnc.run_speed );
            break;
        case 4:  // X=Y=0
            sprintf( eznc_line, "G90G1X0Y0F%.0f", EZnc.run_speed );
            break;
        case 5:  // Z=0
            sprintf( eznc_line, "G90G1Z0F%.0f", EZnc.run_speed );
            break;
        case 6:  // X=Y=Z=0
            sprintf( eznc_line, "G90G1X0Y0Z0F%.0f", EZnc.run_speed );
            break;
    }
    gc_execute_line(eznc_line, Uart0);
}

void ez_goto_XY()
{
    float x, y, *p;

#define Nm 4
    int8_t sel=1, smin=1;
    char menu[Nm][Nstr] = { 
        "Goto",
        "cancel/return",  // missed comma will pass compilier
        "XY Relative",
        "XY Absolute"
    };
    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );
#undef Nm

    if( touchedR ) return;
    switch(sel){
        case 1:
            clearBtnTouch();
            return;
        case 2:
            x = y = 0.0;               // Goto is too long
            if( ! ez_enter_XY( (char *)"G1 Rel", &x, &y) )  return;
            sprintf( eznc_line, "G91G1X%.4fY%.4fF%.0f", x, y, EZnc.run_speed );
            break;
        case 3:  // XY absolute  Q: start at (0,0) may be easier to enter ?
            p = get_mpos();
            mpos_to_wpos(p);
            x = p[0];  y=p[1];
            if( gc_state.modal.units == Units::Inches ){ x = p[0]*MM2INCH;  y = p[1]*MM2INCH; }

            if( ! ez_enter_XY( (char *)"G1 Abs", &x, &y) ) return;
            sprintf( eznc_line, "G90G1X%.4fY%.4fF%.0f", x, y, EZnc.run_speed );                
            break;
    }
    gc_execute_line(eznc_line, Uart0);
}

void ez_goto_Z()
{
    float z, *p;
#define Nm 4
    int8_t sel=1, smin=1;
    char menu[Nm][Nstr] = { 
        "Goto",
        "cancel/return",
        "Z Relative",
        "Z Absolute",
    };
    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );  // blocking
#undef Nm

    if( touchedR ) return;
    switch(sel){
        case 1:
            clearBtnTouch();
            return;
        case 2:  // Z rel
            z = 0.0;               // Goto is too long
            if( ! ez_enter_Z( (char *)"G1 Rel", &z) )  return;
            sprintf( eznc_line, "G91G1Z%.4fF%.0f", z, EZnc.run_speed );
            break;
        case 3:
            p = get_mpos();
            mpos_to_wpos(p);
            z = p[2];
            if( ! ez_enter_Z( (char *)"G1 Abs", &z) )  return;
            sprintf( eznc_line, "G90G1Z%.4fF%.0f", z, EZnc.run_speed );
            break;
    }
    gc_execute_line(eznc_line, Uart0);
}

void ez_goto_pos()        // run_speed, perhaps add rapid position
{
    int8_t sel=1, smin=1;

#define Nm 6
    char menu[Nm][Nstr] = {
        "Goto Position",
        "B or A (on XY)",
        "Xo/Yo/Zo",
        "XY",
        "Z",
        "cancel/return",   // sub-menu has return to DRO as 1st choice
    };

    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );  // blocking
#undef Nm

    if( touchedR ) return;
    switch(sel){
        case 1:  ez_goto_AB();   break;
        case 2:  ez_goto_XYZ0(); break;
        case 3:  ez_goto_XY();   break;
        case 4:  ez_goto_Z();    break;
        case 5:  break;
    }
    clearBtnTouch();
    return;
}

void ez_menu()   // top level ui menu, only title line is auto-scrolled
{
    int8_t sel=1, smin=1, junk=1;
    int progress=0, old_progress=0;

    // todo: assign function with title at the same time
#define Nm 7
    char menu[Nm][Nstr] = {
        "ezNC-3",
        "Run   G-code",
        "Power Feed",
        "Goto  Position",
        "Set   Position",
        "Config",
        "Back to DRO",
    };
    char msg[Nstr];

    clearBtnTouch();
    select_from_menu( Nm, menu, &sel, &smin );  // blocking
#undef Nm

    if( touchedR ) return;

    switch(sel){
    case 1:  // run g-code
        ez_gcfn = ez_select_file( "/");
        if (ez_gcfn[0] != 0 ){
                char gcmd[128];
                sprintf( gcmd, "$localfs/run=%s", ez_gcfn.c_str() );
                execute_line( gcmd, Uart0, WebUI::AuthenticationLevel::LEVEL_GUEST );
                ez_check_cancel = true; 
        }
        return;

    case 2: // power feed between points A & B
        if( (fabs(mark_B[0]) + fabs(mark_B[1])) < 1e-4 ){
            float* pos = get_mpos();
            mpos_to_wpos(pos);
            mark_B[0] = pos[0];
            mark_B[1] = pos[1];
        }
        ez_run_pwrfd = true;  // can not block, set flag and return right away
        uimenu_active = 1;    // stops DRO
        clearBtnTouch();
        return;

    case 3: ez_goto_pos(); return;
    case 4: ez_set_pos();  return;
    case 5: ez_config();   return;
    case 6: return;
    }
}

#endif

//-------------------------------------------------------------------------
// currently, eznc_dispatch is really just ez_dro

void ez_dro()
{

    if( sys.state == State::Idle || sys.state == State::Jog ){
        enc_cnt = readEncoder(1);  // no double reads
        if( enc_cnt != 0 && (millis() - jog_cancel_t) > 500 ){
            ez_jog( enc_cnt );
        }
    }

    // TOOD: turn knob to override feed-rate

    if( touched() ){ // todo: individual button clear
  
        if( touchedL ){
            jog_axis = (jog_axis+1) % 3;
            update_dro = 1;
        }
            
        if( touchedR ){
            if( sys.state == State::Jog ){
                ez_cancel_jog();
            }
            else if( sys.state == State::Idle ){
                if( jss_inc )  jog_stepsize *= 10;
                else           jog_stepsize /= 10;

                if( jog_stepsize > 5000 ){     // limit to 1..1000
                    jss_inc = false;   jog_stepsize = 100;
                }
                if( jog_stepsize < 1 ){
                    jss_inc = true;    jog_stepsize = 10;
                }
                update_dro = 1;
            }
            else if( sys.state == State::Cycle )
                sys.abort = true;
        }
    }

    // click while running ? currently has no effect, perhaps do hold/pause

    if( clickCounterSW1 &&  sys.state == State::Idle ){  // click main button in DRO enters ui menu mode
        uimenu_active = 1;
        ez_menu();
        if( ! ez_run_pwrfd ){    // pwr feed wants to control screen
            uimenu_active = 0;
            update_dro = 1;
        }
    }
    clearBtnTouch();            
}

void ez_ui()  // NOT used, current ui_menu is blocking, much easier to implement
{
    enc_cnt = readEncoder(0);
    if( enc_cnt != 0 ){
    }

    if( btnClicked() || touched() ){
        if( touchedL ){
        }
            
        if( touchedR ){
        }
            
        if( clickCounterSW1 ){
            //log_info( "exit ui menu..." );
            uimenu_active = 0;
            update_dro = 1;
        }
    }
    clearBtnTouch();
}

// called by protocal loop, try to be as short as possible

bool gcode_started = false;

void eznc_dispatch( void )    // top level dispatcher
{    
    // tricky to get it right
    // is it really compatible with pwr fd cycles ?

    if( ez_check_cancel && sys.state == State::Cycle )
        gcode_started = true;

    if( ez_check_cancel && gcode_started && sys.state == State::Idle  ){
        // may be tripped right after file started ?  sys.state is not updated quickly
        // solution: process touchR in ISR,    the following may be unnecessary
        //ez_check_cancel = false;
        gcode_started = false;
        ez_gcfn[0] == 0;   // no good way to clear this w/o changing FluidNC
    }

    if( ez_run_pwrfd ){
        ez_pwr_fd();
        return;      // not necessary, but do it anyway
    }

    if( uimenu_active )
        ez_ui();        
    else
        ez_dro();
}

