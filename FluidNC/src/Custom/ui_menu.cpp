// UI routines, modified from eznc u8g.cpp

#ifdef INCLUDE_OLED_IO

#include <cstring>
#include "../Config.h"
#include "../Logging.h"

#include "oled_io.h"
extern volatile int uimenu_active;
extern volatile int update_dro;
extern volatile int update_menu;

void oled_flash(){
       // not really flash, just clear, next cycle will redraw
#if defined(BRD_TINYBEE)
        u8g2->clearDisplay();
        //delay(20);

#else
	oled->clear();
	oled->display();
#endif
	update_menu = 1;
}		

#endif