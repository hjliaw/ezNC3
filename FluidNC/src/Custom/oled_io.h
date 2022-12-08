#pragma once

#ifdef OLED_096
#include <SSD1306Wire.h>
extern SSD1306Wire* oled;
#else
#include <SH1106Wire.h>
extern SH1106Wire* oled;
#endif

// The SDA and SCL pins must be ordinary GPIOs; mappings to Pin objects do not
// work because the Arduino Wire library performs GPIO setup operations that cannot
// be overridden.
void init_oled(uint8_t address, pinnum_t sda_gpio, pinnum_t scl_gpio, OLEDDISPLAY_GEOMETRY geometry);
