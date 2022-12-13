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

// This displays the status of the ESP32 Radios...BT, WiFi, etc
static void oledRadioInfo() {
    String radio_addr   = "";
    String radio_name   = "";
    String radio_status = "";
    char js[16];

#    ifdef ENABLE_BLUETOOTH
    if (WebUI::bt_enable->get()) {
        radio_name = String("BT: ") + WebUI::bt_name->get();
    }
#    endif
#    ifdef ENABLE_WIFI
    if (radio_name == "") {
        if ((WiFi.getMode() == WIFI_MODE_STA) || (WiFi.getMode() == WIFI_MODE_APSTA)) {
            radio_name = "STA: " + WiFi.SSID();
            radio_addr = WiFi.localIP().toString();
        } else if ((WiFi.getMode() == WIFI_MODE_AP) || (WiFi.getMode() == WIFI_MODE_APSTA)) {
            radio_name = String("AP:") + WebUI::wifi_ap_ssid->get();
            radio_addr = WiFi.softAPIP().toString();
        } else {
            radio_name = "Radio Mode: None";
        }
    }
#    endif

    if (radio_name == "") {
        radio_name = "Radio Mode:Disabled";
    }

    oled->setTextAlignment(TEXT_ALIGN_LEFT);
    oled->setFont(ArialMT_Plain_10);

    if (sys.state == State::Alarm) {  // print below Alarm:
        oled->drawString(0, 18, radio_name);
        oled->drawString(0, 30, radio_addr);

    } else {  // print next to status
#    ifdef ENABLE_BLUETOOTH
        oled->drawString(55, 2, radio_name);
#    else
        oled->drawString(50, 2, radio_addr);
#    endif
        sprintf( js, "s%d", jog_stepsize );
        oled->drawString(115, 2, js );
    }
}

static void draw_checkbox(int16_t x, int16_t y, int16_t width, int16_t height, bool checked) {
    if (checked) {
        oled->fillRect(x, y, width, height);  // If log.0
    } else {
        oled->drawRect(x, y, width, height);  // If log.1
    }
}

// remove static, need to be called from uI
void oledDRO() {
    uint8_t oled_y_pos;
    char msg[16];

    oled->clear();
    oled->setFont(DejaVu_Sans_Mono_14);

    oled->setTextAlignment(TEXT_ALIGN_LEFT);
    if( infile ){
        String p = infile->path();
        p.replace("/littlefs/", "");
        oled->drawString(0, 0, p);
    }
    else{
        oled->drawString(0, 0, state_name());
    }

    // perhaps, don't show X/Y/Z during infile
    
    oled->setTextAlignment(TEXT_ALIGN_RIGHT);
    if( sys.state != State::Idle ){  // infile is not a good indicator, compromise
        int progress = 100;
        if( infile ) progress = infile->percent_complete();
        oled->drawString(126, 2, String(progress) + "%" );
        //log_warn( "progress " << progress << "% " << String(millis()-run_t0) );
    }
    else{
        sprintf( msg, "jog%d", jog_stepsize );
        oled->drawString(126, 2, msg );
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
        oled->setTextAlignment(TEXT_ALIGN_LEFT);
        oled->drawString(0, oled_y_pos, axis_letter);

        oled->setTextAlignment(TEXT_ALIGN_RIGHT);
        if( gc_state.modal.units == Units::Mm ){
            snprintf(axisVal, 20 - 1, "%.2f mm", print_position[axis]);
            oled->drawString( 126, oled_y_pos, axisVal);
        }
        else{
            snprintf(axisVal, 20 - 1, "%.3f in", print_position[axis]/25.4);
            oled->drawString( 126, oled_y_pos, axisVal);
        }
    }
    oled->display();
}

// a lot of global parameters, todo: make an eznc object ?

void oledUI() {

	uint8_t offs;
	offs = (ui_sel < 0 ) ? 2 : 12;

    oled->clear();
    oled->setFont(DejaVu_Sans_Mono_14);
	//oled->setFont(DejaVu_Serif_13);   // ok, but space is much narrower than >

    oled->setTextAlignment(TEXT_ALIGN_LEFT);  // duh, reason for no show

    if( ui_frame > 0){
        //oled->drawRect( 0, 0, 127, 18);
        oled->drawHorizontalLine( 2, 0, 128-4);
        oled->drawHorizontalLine( 2,17, 128-4);
        oled->drawVerticalLine(   0, 2,  18-4);
        oled->drawVerticalLine( 127, 2,  18-4);

        oled->drawLine( 2,  0,  0,  2);
        oled->drawLine( 0, 15,  2, 17);
        oled->drawLine( 125,  0,  127,  2);
        oled->drawLine( 125, 17,  127, 15);
    }

	if( ui_sel > 0 ){  // what happens when sel=0 ?
        offs = 12;
        oled->drawString( 2, 17 +14*(ui_sel-1), ">");
	}

    for( int i=0; i<4; i++){
        int x, y;
        x = (i==0) ? 4 : offs;
        y = (i==0) ? 1 : (i*14+4);
        #ifdef UTF8
    	  oled->drawUTF8( x, y, ui_txt[i]);
        #else
	      oled->drawString( x, y, ui_txt[i]);
        #endif
    }
    
    oled->display();
}


static void oledUpdate(void* pvParameters) {
    int32_t old_pos[MAX_N_AXIS];

    vTaskDelay(600);  // wait for flash screen

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

#if 0
static void oledUpdateOLD(void* pvParameters) {
    TickType_t xOledInterval = 1000;  // in ticks (typically ms)

    while (true) {
        uint16_t file_ticker = 0;
        oled->clear();

        String state_string = "";

        oled->setTextAlignment(TEXT_ALIGN_LEFT);
        oled->setFont(ArialMT_Plain_16);
        oled->drawString(0, 0, state_name());

        if (infile) {
            oled->clear();
            oled->setTextAlignment(TEXT_ALIGN_CENTER);
            oled->setFont(ArialMT_Plain_10);
            state_string = "File";
            for (int i = 0; i < file_ticker % 10; i++) {
                state_string += ".";
            }
            file_ticker++;
            oled->drawString(63, 0, state_string);

            oled->drawString(63, 12, infile->path());

            int progress = infile->percent_complete();
            // draw the progress bar
            oled->drawProgressBar(0, 45, 120, 10, progress);

            // draw the percentage as String
            oled->setFont(ArialMT_Plain_10);
            oled->setTextAlignment(TEXT_ALIGN_CENTER);
            oled->drawString(64, 25, String(progress) + "%");
            xOledInterval = 250;
        } else if (sys.state == State::Alarm) {
            oledRadioInfo();
            xOledInterval = 300;
        } else {
            oledDRO();
            oledRadioInfo();
            xOledInterval = 300;
        }
        oled->display();

        vTaskDelay(xOledInterval);
    }
}
#endif

void display_init() {
#ifdef BRD_DLC32
    init_oled(0x3c, GPIO_NUM_0, GPIO_NUM_4, GEOMETRY_128_64);  // DLC32
    oled->flipScreenVertically();   // no flip for ezNC
#else
    init_oled(0x3c, GPIO_NUM_21, GPIO_NUM_22, GEOMETRY_128_64);  // ezNC
#endif
    oled->setTextAlignment(TEXT_ALIGN_LEFT);
    oled->clear();
    oled->setFont(ArialMT_Plain_16);
    oled->drawString(0, 0, "Starting");
    oled->setFont(ArialMT_Plain_24);
    oled->drawString(0, 20, "ezFluidNC");
    oled->display();

    xTaskCreatePinnedToCore(oledUpdate,        // task
                            "oledUpdateTask",  // name for task
                            4096,              // size of task stack
                            NULL,              // parameters
                            1,                 // priority
                            &oledUpdateTaskHandle,
                            //0  //runs fine
                            CONFIG_ARDUINO_RUNNING_CORE  // must run the task on same core
    );
}


#endif
