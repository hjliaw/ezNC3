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

#if defined(BRD_EZMPG) || defined (BRD_DLC32)
SH1106Wire* oled;    // 1.3"
#elif defined(BRD_TINYBEE)
U8G2_ST7920_128X64_F_SW_SPI *u8g2;
#else
SSD1306Wire* oled;
#endif

void init_oled()
{
//Uart0 << "[MSG:INFO Init OLED SDA:gpio." << sda_gpio << " SCL:gpio." << scl_gpio << "]\n";

// TODO: should let user define DISPLAY instead of by board

#if defined(BRD_EZMPG)  || defined (BRD_DLC32)
    oled = new SH1106Wire(address, sda_gpio, scl_gpio, geometry, I2C_ONE, 400000);
#elif defined(BRD_TINYBEE)
    u8g2 = new U8G2_ST7920_128X64_F_SW_SPI(U8G2_R0, 0, 21, 4);  // TODO: pass parameters
#else
    oled = new SSD1306Wire(address, sda_gpio, scl_gpio, geometry, I2C_ONE, 400000);
#endif

#if defined(BRD_TINYBEE)
    u8g2->begin();
#else
    oled->init();
#endif

}
#endif
