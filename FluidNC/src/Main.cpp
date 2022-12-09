// Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#ifndef UNIT_TEST

#    include "Main.h"
#    include "Machine/MachineConfig.h"

#    include "Config.h"
#    include "Report.h"
#    include "Settings.h"
#    include "SettingsDefinitions.h"
#    include "Limits.h"
#    include "Protocol.h"
#    include "System.h"
#    include "Uart.h"
#    include "MotionControl.h"
#    include "Platform.h"
#    include "StartupLog.h"

#    include "WebUI/TelnetServer.h"
#    include "WebUI/InputBuffer.h"

#    include "WebUI/WifiConfig.h"
#    include "Driver/localfs.h"

extern void make_user_commands();

// 20221201 HJL: ezNC code

#ifndef NO_ENCODER
#include <ESP32Encoder.h>
ESP32Encoder encUI; 
#endif

#ifdef BRD_DLC32
  #define SW1  GPIO_NUM_36     // X
  #define SWL  GPIO_NUM_35     // Y
  #define SWR  GPIO_NUM_34     // Z
  #define ENCA GPIO_NUM_22     // probe-pin brd v2.1
  #define ENCB GPIO_NUM_33     // LCD_RS
#else
  #define SW1  GPIO_NUM_34 // encoder-Z (27) 
  #define SWL  GPIO_NUM_14 // was 32, move to encoder pin
  #define SWR  GPIO_NUM_13 // was 33
  #define ENCA GPIO_NUM_36
  #define ENCB GPIO_NUM_39
#endif

volatile int clickCounterSW1 = 0;
volatile int clickCounterSWL = 0;
volatile int clickCounterSWR = 0;

volatile int  sw1_status;
volatile int  swL_status;
volatile int  swR_status;

volatile unsigned long lastReleaseTime = 0;

void IRAM_ATTR handleInterruptSW1() {      // rising only for noisy switch or sensitive i/p
    unsigned long t, dt;
    t = millis();
    dt = t - lastReleaseTime;
    if( dt < 50 ) return;  // increase from 50 & 150 made it worse ?

    sw1_status = digitalRead(SW1);   // triggered on rising, should be high
        
    if( sw1_status == HIGH ){   // btn rlsd
        lastReleaseTime = t;
        clickCounterSW1++;
    }
}

void IRAM_ATTR handleInterruptSWL() {
    unsigned long t, dt;
    t = millis();
    dt = t - lastReleaseTime;
    if( dt < 50 ) return;

    swL_status = digitalRead(SWL);
    if( swL_status == HIGH ){   // btn rlsd
        lastReleaseTime = t;
        clickCounterSWL++;
    }
}

void IRAM_ATTR handleInterruptSWR() {
    unsigned long t, dt;
    t = millis();
    dt = t - lastReleaseTime;
    if( dt < 50 ) return;
    
    swR_status = digitalRead(SWR);
    if( swR_status == HIGH ){   // btn rlsd
        lastReleaseTime = t;
        clickCounterSWR++;
    }
}
int volatile btnClickedRlsd( void ){   // clicked and released, for UI navigation
        delay(50);
        return( (clickCounterSW1 > 0) && digitalRead(SW1) );
}

int volatile btnClicked( void ){  // no delay
        return( clickCounterSW1 || clickCounterSWL || clickCounterSWR);
}

void clearBtn( void ){
        clickCounterSW1 = 0;
        clickCounterSWL = 0;
        clickCounterSWR = 0;
}

int32_t readEncoder(int a)  // ui encoder, a>0, single read
{
    int32_t r = encUI.getCount();

    if( r != 0 && a == 0){    // debounce cheap encoder, works very well
        delay(150);
        r = encUI.getCount();
    }
    encUI.clearCount();

    return r;
}

void setup() {

    ESP32Encoder::useInternalWeakPullResistors= UP;
    pinMode( ENCA, INPUT_PULLDOWN);
    pinMode( ENCB, INPUT_PULLDOWN);

    encUI.clearCount();
    encUI.attachHalfQuad( ENCA, ENCB );  // duh

    pinMode( SW1, INPUT_PULLUP);
    attachInterrupt( SW1, handleInterruptSW1, RISING );

    pinMode( SWL, INPUT_PULLUP);
    attachInterrupt( SWL, handleInterruptSWL, RISING );

    pinMode( SWR, INPUT_PULLUP);
    attachInterrupt( SWR, handleInterruptSWR, RISING );

    try {
        uartInit();       // Setup serial port
        Uart0.println();  // create some white space after ESP32 boot info

        //delay_ms(500);
        
        // Setup input polling loop after loading the configuration,
        // because the polling may depend on the config
        allChannels.init();

        WebUI::WiFiConfig::reset();

        display_init();

        protocol_init();

        // Load settings from non-volatile storage
        settings_init();  // requires config

        log_info("FluidNC " << git_info);
        log_info("Compiled with ESP32 SDK:" << esp_get_idf_version());

        if (localfs_mount()) {
            log_error("Cannot mount a local filesystem");
        } else {
            log_info("Local filesystem type is " << localfsName);
        }

        bool configOkay = config->load();

        make_user_commands();

        if (configOkay) {
            log_info("Machine " << config->_name);
            log_info("Board " << config->_board);

            // The initialization order reflects dependencies between the subsystems
            if (config->_i2so) {
                config->_i2so->init();
            }
            if (config->_spi) {
                config->_spi->init();

                if (config->_sdCard != nullptr) {
                    config->_sdCard->init();
                }
            }

            config->_stepping->init();  // Configure stepper interrupt timers

            plan_init();

            config->_userOutputs->init();

            config->_axes->init();

            config->_control->init();

            config->_kinematics->init();

            auto n_axis = config->_axes->_numberAxis;
            for (size_t axis = 0; axis < n_axis; axis++) {
                set_motor_steps(axis, 0);  // Clear machine position.
            }

            machine_init();  // user supplied function for special initialization
        }

        // Initialize system state.
        if (sys.state != State::ConfigAlarm) {
            if (FORCE_INITIALIZATION_ALARM) {
                // Force ALARM state upon a power-cycle or hard reset.
                sys.state = State::Alarm;
            } else {
                sys.state = State::Idle;
            }

            limits_init();

            // Check for power-up and set system alarm if homing is enabled to force homing cycle
            // by setting alarm state. Alarm locks out all g-code commands, including the
            // startup scripts, but allows access to settings and internal commands. Only a homing
            // cycle '$H' or kill alarm locks '$X' will disable the alarm.
            // NOTE: The startup script will run after successful completion of the homing cycle, but
            // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
            // things uncontrollably. Very bad.
            if (config->_start->_mustHome && Machine::Axes::homingMask) {
                // If there is an axis with homing configured, enter Alarm state on startup
                sys.state = State::Alarm;
            }
            for (auto s : config->_spindles) {
                s->init();
            }
            Spindles::Spindle::switchSpindle(0, config->_spindles, spindle);

            config->_coolant->init();
            config->_probe->init();
        }

    } catch (const AssertionFailed& ex) {
        // This means something is terribly broken:
        log_error("Critical error in main_init: " << ex.what());
        sys.state = State::ConfigAlarm;
    }

    if (!WebUI::wifi_config.begin()) {
        WebUI::bt_config.begin();
    }
    allChannels.deregistration(&startupLog);
}

static void reset_variables() {
    // Reset primary systems.
    system_reset();
    protocol_reset();
    gc_init();  // Set g-code parser to default state
    // Spindle should be set either by the configuration
    // or by the post-configuration fixup, but we test
    // it anyway just for safety.  We want to avoid any
    // possibility of crashing at this point.

    plan_reset();  // Clear block buffer and planner variables

    if (sys.state != State::ConfigAlarm) {
        if (spindle) {
            spindle->stop();
            report_ovr_counter = 0;  // Set to report change immediately
        }
        Stepper::reset();  // Clear stepper subsystem variables
    }

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();
    allChannels.flushRx();
    report_init_message(allChannels);
    mc_init();
}

void loop() {
    static int tries = 0;
    try {
        reset_variables();
        // Start the main loop. Processes program inputs and executes them.
        // This can exit on a system abort condition, in which case run_once()
        // is re-executed by an enclosing loop.  It can also exit via a
        // throw that is caught and handled below.
        protocol_main_loop();
    } catch (const AssertionFailed& ex) {
        // If an assertion fails, we display a message and restart.
        // This could result in repeated restarts if the assertion
        // happens before waiting for input, but that is unlikely
        // because the code in reset_variables() and the code
        // that precedes the input loop has few configuration
        // dependencies.  The safest approach would be to set
        // a "reconfiguration" flag and redo the configuration
        // step, but that would require combining main_init()
        // and run_once into a single control flow, and it would
        // require careful teardown of the existing configuration
        // to avoid memory leaks. It is probably worth doing eventually.
        log_error("Critical error in run_once: " << ex.msg);
        log_error("Stacktrace: " << ex.stackTrace);
        sys.state = State::ConfigAlarm;
    }
    // sys.abort is a user-initiated exit via ^x so we don't limit the number of occurrences
    if (!sys.abort && ++tries > 1) {
        log_info("Stalling due to too many failures");
        while (1) {}
    }
}

void WEAK_LINK machine_init() {}

#    if 0
int main() {
    setup();  // setup()
    while (1) {   // loop()
        loop();
    }
    return 0;
}
#    endif

#endif
