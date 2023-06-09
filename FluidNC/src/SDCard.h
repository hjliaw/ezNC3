// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */

#include "Configuration/Configurable.h"
#include "WebUI/Authentication.h"
#include "Pin.h"
#include "Error.h"

#include <cstdint>

// XXX This should be a configuration parameter of the SPI bus
const int32_t SPIfreq = 4000000;

class SDCard : public Configuration::Configurable {
public:
    enum class State : uint8_t {
        Idle          = 0,
        NotPresent    = 1,
        Busy          = 2,
        BusyUploading = 3,
        BusyParsing   = 4,
        BusyWriting   = 5,
        BusyReading   = 6,
    };

private:
    State _state;
    Pin   _cardDetect;
    Pin   _cs;

public:
    SDCard();
    SDCard(const SDCard&) = delete;
    SDCard& operator=(const SDCard&) = delete;

    void afterParse() override;

    const char* filename();

    // Initializes pins.
    void init();

    // Configuration handlers.
    void group(Configuration::HandlerBase& handler) override {
        handler.item("cs_pin", _cs);
        handler.item("card_detect_pin", _cardDetect);
    }

    ~SDCard();
};
