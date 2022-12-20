// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    OLED display code.

    It is designed to be used with a machine that has no easily accessible serial connection
    It shows basic status and connection information.

    When in alarm mode it will show the current Wifi/BT paramaters and status
    Most machines will start in alarm mode (needs homing)
    If the machine is running a file job it will show the progress
    In other modes it will show state and 3 axis DROs
    Thats All! 
*/
#include "../Config.h"

#include "../Eznc.h"

extern int jog_axis;
extern int jog_stepsize;
extern long run_t0;

volatile int uimenu_active = 0;  // uimenu or dro
volatile int update_dro = 0;
volatile int update_menu = 0;

char   ui_txt[4][Wchars+1];   // todo: change to pointer for scrolling
int    ui_sel, ui_frame;

#ifdef INCLUDE_OLED_BASIC
#    include "oled_io.h"
#    include "../Main.h"  // display_init()

#    include "../Machine/MachineConfig.h"
#    include "WiFi.h"
#    include "../WebUI/WebSettings.h"
#    include "../WebUI/WifiConfig.h"
#    include "../SettingsDefinitions.h"
#    include "../Report.h"
#    include "../Machine/Axes.h"
#    include "../Uart.h"
#    include "../InputFile.h"

static TaskHandle_t oledUpdateTaskHandle = 0;

// wrapper for oled & u8g2

void oled_clear(){
    u8g2->clearBuffer();
}

void oled_setFont(){
    u8g2->setFont(u8g_font_courR10);
//  oled->setFont(ArialMT_Plain_16);
}
void oled_setTextAlignment();

void oled_setFlipMode(){
    u8g2->setFlipMode( EZnc.FlipScreen);
//    if( EZnc.FlipScreen ) oled->flipScreenVertically();
//    else                  oled->resetOrientation();
}

void oled_drawString( uint8_t x, uint8_t y, String s){
    u8g2->drawStr( x, y, s.c_str() );
}

//void oled_drawUTF8( x, y, ui_txt[i]);
void oled_display(){
    u8g2->sendBuffer();
}

void oled_drawRFrame(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t r){
    u8g2->drawRFrame( x1, y1, x2, y2, r);

//        oled->drawHorizontalLine( 2, 0, 128-4);
//        oled->drawHorizontalLine( 2,17, 128-4);
//        oled->drawVerticalLine(   0, 2,  18-4);
//        oled->drawVerticalLine( 127, 2,  18-4);
//        oled->drawLine( 2,  0,  0,  2);
//        oled->drawLine( 0, 15,  2, 17);
//        oled->drawLine( 125,  0,  127,  2);
//        oled->drawLine( 125, 17,  127, 15);
}

// remove static, need to be called from uI
void oledDRO() {
    uint8_t oled_y_pos;
    char msg[16];

    oled_clear();
    //oled_setFont(DejaVu_Sans_Mono_14);
    //oled_setTextAlignment(TEXT_ALIGN_LEFT);
    if( infile ){
        String p = infile->path();    // scroll long file name ?
        p.replace("/littlefs/", "");
        p.replace("/sd/", "");
        oled_drawString(0, 0, p.substring(0, Wchars - 4) );
    }
    else{
        oled_drawString(0, 0, state_name());
    }

    // perhaps, don't show X/Y/Z during infile

    //oled_setTextAlignment(TEXT_ALIGN_RIGHT);
    if( sys.state != State::Idle ){  // infile is not a good indicator, compromise
        if( infile ){
            int progress = infile->percent_complete();
            oled_drawString(126-3*9, 2, String(progress) + "%" );
        }
        else{
            oled_drawString(126-3*9, 2, "..." );
        }
        //log_warn( "progress " << progress << "% " << String(millis()-run_t0) );
    }
    else{
        if( gc_state.modal.units == Units::Mm )
            sprintf( msg, "%.2f %s", jog_stepsize/100.0,  jss_inc ? "<<" : ">>" );
        else
            sprintf( msg, "%.3f %s", jog_stepsize/1000.0, jss_inc ? "<<" : ">>" );

        oled_drawString(126-8*9, 2, msg );  // need to calculate string length
    }

    // todo: invert X/Y/Y when limit switch tripped
    //oled->drawString(80, 14, "L");  // Limit switch

    auto n_axis        = config->_axes->_numberAxis;
    auto ctrl_pins     = config->_control;
    bool prb_pin_state = config->_probe->get_state();

    char axisVal[20];
    float* print_position = get_mpos();
    mpos_to_wpos(print_position);  // same as ezNC

    // only has space for 3-axis ! what about lathe mode ?
    for (uint8_t axis = X_AXIS; axis < n_axis; axis++) {
        oled_y_pos = 19 + (axis * 15);
        // todo: use alignment
        String axis_letter = ((axis==jog_axis)? ">":" ") + String(Machine::Axes::_names[axis]) + "  ";
        //oled_setTextAlignment(TEXT_ALIGN_LEFT);
        oled_drawString(0, oled_y_pos, axis_letter);

        //oled->setTextAlignment(TEXT_ALIGN_RIGHT);
        if( gc_state.modal.units == Units::Mm ){
            snprintf(axisVal, 20 - 1, "%.2f mm", print_position[axis]);
            oled_drawString( 126-8*9, oled_y_pos, axisVal);
        }
        else{
            snprintf(axisVal, 20 - 1, "%.3f in", print_position[axis]/25.4);
            oled_drawString( 126-9*9, oled_y_pos, axisVal);
        }
    }
    oled_display();
}

// a lot of global parameters, todo: make an eznc object ?

void oledUI() {

	uint8_t offs;
	offs = (ui_sel < 0 ) ? 2 : 12;

    oled_clear();
    //oled_setFont(DejaVu_Sans_Mono_14);
    //oled_setTextAlignment(TEXT_ALIGN_LEFT);  // duh, reason for no show

    if( ui_frame > 0){
        oled_drawRFrame( 0, 0, 127, 18, 3);
    }

	if( ui_sel > 0 ){  // what happens when sel=0 ?
        offs = 12;
        oled_drawString( 2, 17 +14*(ui_sel-1), ">");
	}

    for( int i=0; i<4; i++){
        int x, y;
        x = (i==0) ? 4 : offs;
        y = (i==0) ? 1 : (i*14+4);
        #ifdef UTF8
    	  oled->drawUTF8( x, y, ui_txt[i]);
        #else
	      oled_drawString( x, y, ui_txt[i]);
        #endif
    }
    
    oled_display();
}


static void oledUpdate(void* pvParameters) {
    int32_t old_pos[MAX_N_AXIS];

    vTaskDelay(1000);  // wait for flash screen, 600-ms wco is not updated

    uimenu_active=0;
    update_dro = 1;

    for(;;){
        if( uimenu_active ){
            if( update_menu ) oledUI();
            update_menu = 0;
        }
        else{
            if( update_dro ) oledDRO();
            update_dro = 0;
        }

        if( !uimenu_active ){
            int32_t* new_pos = get_motor_steps();
            //for (int a = X_AXIS; a < n_axis; a++){   // crash ? even with y defined ?
            for (int a = 0; a < 3; a++){
                if( new_pos[a] != old_pos[a] ){
                    old_pos[a] = new_pos[a];
                    update_dro = 1;
                }
            }
        }

        if (sys.state == State::Idle)
            vTaskDelay(300);
        else
            vTaskDelay(1500);
    }
}

void display_init() {
#if defined(BRD_DLC32)
    init_oled(0x3c, GPIO_NUM_0, GPIO_NUM_4, GEOMETRY_128_64);
#elif defined(BRD_TINYBEE)
    //init_oled(0x3c, GPIO_NUM_16, GPIO_NUM_17, GEOMETRY_128_64);
    init_oled();
#else
    init_oled(0x3c, GPIO_NUM_21, GPIO_NUM_22, GEOMETRY_128_64);  // ezNC/MPG
#endif

#ifdef UTF8
    u8g2->enableUTF8Print();
    u8g2->setFont(u8g2_font_unifont_t_cyrillic);
#else
    oled_setFont();
#endif
    oled_setFlipMode();

    //oled_setTextAlignment(TEXT_ALIGN_RIGHT);
    //oled->drawString(127, 63-15, "with FluidNC");

    oled_drawString(20, 63-15, "with FluidNC");

    //oled->setTextAlignment(TEXT_ALIGN_LEFT);
    //oled->setFont(ArialMT_Plain_24);
    oled_drawString(10, 10, "ezNC-3");

    oled_display();

    xTaskCreatePinnedToCore(oledUpdate,        // task
                            "oledUpdateTask",  // name for task
                            4096,              // size of task stack
                            NULL,              // parameters
                            1,                 // priority
                            &oledUpdateTaskHandle,
                            //0  //runs fine ?
                            CONFIG_ARDUINO_RUNNING_CORE  // must run the task on same core
    );

}

#endif
