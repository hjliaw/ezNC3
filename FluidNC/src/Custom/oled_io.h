#pragma once

// TODO: add BRD dependence here

#if defined(BRD_EZMPG) || defined (BRD_DLC32)
#include <SH1106Wire.h>
extern SH1106Wire* oled;    // 1.3"
#elif defined(BRD_TINYBEE)
#include <U8g2lib.h>
extern U8G2_ST7920_128X64_F_SW_SPI *u8g2;
#else
#include <SSD1306Wire.h>
extern SSD1306Wire* oled;
#endif

//#include <U8g2lib.h>
//extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C *oled;  NOT yes

// wrapper
void oled_clear();
void oled_setFont();
void oled_setTextAlignment();

void oled_setFlipMode();

void oled_drawString( uint8_t x, uint8_t y, String s);

//void oled_drawUTF8( x, y, ui_txt[i]);
void oled_display();

void oled_drawRFrame(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t r);

extern volatile int updateoled;

// The SDA and SCL pins must be ordinary GPIOs; mappings to Pin objects do not
// work because the Arduino Wire library performs GPIO setup operations that cannot
// be overridden.

// u8g2 spi hard-coded, to be fixed

void init_oled(); //uint8_t address, pinnum_t sda_gpio, pinnum_t scl_gpio, OLEDDISPLAY_GEOMETRY geometry);
