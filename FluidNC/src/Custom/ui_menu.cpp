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
	// not really flash, just clear, then ask for redraw
	oled->clear();
	oled->display();
	update_menu = 1;
}		

#endif