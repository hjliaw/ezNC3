// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    OLED initialization

    Library Info:
        https://github.com/ThingPulse/esp8266-oled-ssd1306

    Install to PlatformIO with this typed at the terminal
        platformio lib install 2978

    Uncomment the "thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays"
    line in platformio.ini
*/

#include "../Config.h"

#ifdef INCLUDE_OLED_IO
#    include "oled_io.h"
#    include "../Uart.h"

#if defined(BRD_EZMPG) || defined (BRD_DLC32) || defined (BRD_TINYBEE)
SH1106Wire* oled;    // 1.3"
#else
SSD1306Wire* oled;
#endif

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C *oled;  // 1.3" u8g2 TBD

void init_oled(uint8_t address, pinnum_t sda_gpio, pinnum_t scl_gpio, OLEDDISPLAY_GEOMETRY geometry) {
    Uart0 << "[MSG:INFO Init OLED SDA:gpio." << sda_gpio << " SCL:gpio." << scl_gpio << "]\n";

// TODO: should let user define DISPLAY instead of by board

#if defined(BRD_EZMPG)  || defined (BRD_DLC32) || defined (BRD_TINYBEE)
    oled = new SH1106Wire(address, sda_gpio, scl_gpio, geometry, I2C_ONE, 400000);
#else
    oled = new SSD1306Wire(address, sda_gpio, scl_gpio, geometry, I2C_ONE, 400000);
#endif

    //NOT WORKING YET
    // oled = new U8G2_SH1106_128X64_NONAME_F_HW_I2C( U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

    oled->init();
}
#endif
