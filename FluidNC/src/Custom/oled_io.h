#pragma once

#include "BoardConfig.h"
// Both driver headers are included unconditionally (rather than via a
// computed OLED_DRIVER_HEADER macro) so PlatformIO's Library Dependency
// Finder can statically detect them and pull in their dependencies (e.g. Wire).
#include <SH1106Wire.h>
#include <SSD1306Wire.h>
extern OLED_DRIVER_TYPE* oled;

extern volatile int updateoled;

//#include <U8g2lib.h>
//extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C *oled;  NOT yes

// The SDA and SCL pins must be ordinary GPIOs; mappings to Pin objects do not
// work because the Arduino Wire library performs GPIO setup operations that cannot
// be overridden.
void init_oled(uint8_t address, pinnum_t sda_gpio, pinnum_t scl_gpio, OLEDDISPLAY_GEOMETRY geometry);
