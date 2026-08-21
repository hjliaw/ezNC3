#pragma once

// Centralized per-board pin and OLED display configuration.
// This is the single place that maps a BRD_xxx build flag to hardware
// details. To add a new board, add one #elif block here; oled_io.h,
// oled_io.cpp, oled_basic.cpp, and Main.cpp all read from these macros
// instead of keeping their own board-name #ifdef chains.

#if defined(BRD_DLC32)
    #define OLED_DRIVER_TYPE    SH1106Wire
    #define OLED_ADDR           0x3c
    #define OLED_SDA            GPIO_NUM_0
    #define OLED_SCL            GPIO_NUM_4
    #define OLED_GEOMETRY       GEOMETRY_128_64

    #define SW1  GPIO_NUM_36     // X
    #define SWL  GPIO_NUM_35     // Y
    #define SWR  GPIO_NUM_34     // Z
    #define ENCA GPIO_NUM_22     // probe-pin brd v2.1
    #define ENCB GPIO_NUM_33     // LCD_RS
#elif defined(BRD_EZMPG)
    #define OLED_DRIVER_TYPE    SH1106Wire
    #define OLED_ADDR           0x3c
    #define OLED_SDA            GPIO_NUM_21
    #define OLED_SCL            GPIO_NUM_22
    #define OLED_GEOMETRY       GEOMETRY_128_64

    #define SW1  GPIO_NUM_34
    #define SWL  GPIO_NUM_33     // undo old lib mix up
    #define SWR  GPIO_NUM_32
    #define ENCA GPIO_NUM_36
    #define ENCB GPIO_NUM_39
#elif defined(BRD_TINYBEE)
    #define OLED_DRIVER_TYPE    SH1106Wire
    #define OLED_ADDR           0x3c
    #define OLED_SDA            GPIO_NUM_16
    #define OLED_SCL            GPIO_NUM_17
    #define OLED_GEOMETRY       GEOMETRY_128_64

    #define SW1  GPIO_NUM_39    // TB
    #define SWL  GPIO_NUM_36    // TH1
    #define SWR  GPIO_NUM_34    // TH2, need to shift jumper
    #define ENCA GPIO_NUM_14
    #define ENCB GPIO_NUM_12
#elif defined(BRD_RODENT)
    #define OLED_DRIVER_TYPE    SSD1306Wire
    #define OLED_ADDR           0x3c
    #define OLED_SDA            GPIO_NUM_27
    #define OLED_SCL            GPIO_NUM_26
    #define OLED_GEOMETRY       GEOMETRY_128_64

    #define SW1  GPIO_NUM_34    // Y-lim
    #define SWL  GPIO_NUM_33    // Z
    #define SWR  GPIO_NUM_32    // A
    #define ENCA GPIO_NUM_16    // RXD  spindle
    #define ENCB GPIO_NUM_15    // TXD
#elif defined(BRD_EZNC2)
    #define OLED_DRIVER_TYPE    SSD1306Wire
    #define OLED_ADDR           0x3c
    #define OLED_SDA            GPIO_NUM_21
    #define OLED_SCL            GPIO_NUM_22
    #define OLED_GEOMETRY       GEOMETRY_128_64

    #define SW1  GPIO_NUM_34  // eznc test jig or ezNC2
    #define SWL  GPIO_NUM_32  //14 = encoder pin
    #define SWR  GPIO_NUM_33  //13
    #define ENCA GPIO_NUM_36
    #define ENCB GPIO_NUM_39
#else
    #define OLED_DRIVER_TYPE    SSD1306Wire
    #define OLED_ADDR           0x3c
    #define OLED_SDA            GPIO_NUM_21
    #define OLED_SCL            GPIO_NUM_22
    #define OLED_GEOMETRY       GEOMETRY_128_64

    #define SW1  GPIO_NUM_34  // eznc test jig or ezNC2
    #define SWL  GPIO_NUM_32  //14 = encoder pin
    #define SWR  GPIO_NUM_33  //13
    #define ENCA GPIO_NUM_36
    #define ENCB GPIO_NUM_39
#endif
