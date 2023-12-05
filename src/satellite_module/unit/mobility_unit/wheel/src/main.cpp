/**
 * @file main.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "wheel_controller/config.hpp"

#include <Arduino.h>

#ifndef SYSTEM_WATCHDOG_ENABLE
#endif
#if SYSTEM_WATCHDOG_ENABLE
#ifdef __AVR__
#include <avr/wdt.h>
#else
#undef SYSTEM_WATCHDOG_ENABLE
#define SYSTEM_WATCHDOG_ENABLE 0
#endif
#endif

#include "maid_robot_system/common/time_check.hpp"
#include "wheel_controller/model_on_wheel_controller.hpp"

mobility_unit::ModelOnWheelController model;

uint8_t mode_led = LOW;

#if SYSTEM_WATCHDOG_ENABLE
volatile bool system_error = false;

void happened_change_mode(ModeList mode)
{
    switch (mode) {
        case ModeList::MODE_NOT_INITIALIZED:
        case ModeList::MODE_RUNNING:
        case ModeList::MODE_FINISHED:
            system_error = false;
            break;
        default:
            system_error = true;
            break;
    }
}

#endif

/**
 * @brief control the LED
 *
 * @param value LED state
 */
void state_led(uint8_t value)
{
    if (mode_led != value) {
        digitalWrite(PIN_BOARD_LED, value);
        mode_led = value;
    }
}

/**
 * @brief control board LED
 *
 * @param mode Wheel controller mode
 */
void state_monitor(ModeList mode)
{
    static TimeCheck interval;
    static ModeList previous_mode = ModeList::MODE_NOT_INITIALIZED;
    static int interval_ms        = 100;

    if (previous_mode != mode) {
        previous_mode = mode;
        switch (mode) {
            case ModeList::MODE_RUNNING:
                interval_ms = 0;
                state_led(HIGH);
                break;
            case ModeList::MODE_NOT_INITIALIZED:
                interval_ms = 100;
                break;
            case ModeList::MODE_ERROR_NOT_COMMUNICATION:
                interval_ms = 250;
                break;
            case ModeList::MODE_ERROR_GENERAL:
            default:
                interval_ms = 1000;
                break;
        }
    }
    if (0 != interval_ms) {
        if (true == interval.check_passing(interval_ms)) {
            state_led(!mode_led);
        }
    }
}

/**
 * @brief setup function
 *
 */
void setup()
{
#if 0
// TODO : 起動時に待つ必要あるのか？
//PWM音 変更
// TCCR1B = (TCCR1B & 0b11111000) | 0x01; //31.37255 [kHz]
// TCCR4B = (TCCR4B & 0b11111000) | 0x01; //31.37255 [kHz] pin 8,7,6
//delay(3000);
#endif

#if SYSTEM_WATCHDOG_ENABLE
    wdt_enable(WDTO_8S);
#endif

    // setup LED
    pinMode(SETTING_PIN_LED, OUTPUT);
    state_led(LOW);

#if 0
    // loop until initialization completes

    do {
        state_monitor(model.get_mode());
    } while (false == model.setup());
#else
    (void)model.setup();
#endif

#if SYSTEM_WATCHDOG_ENABLE
    (void)model.set_callback_change_mode(&happened_change_mode);
#endif

#if OUTPUT_LOG
    LOG_DEBUG("= setup : finish = ");
#endif
}

/**
 * @brief loop function
 *
 */
void loop()
{
#if SYSTEM_WATCHDOG_ENABLE
    static TimeCheck wdt;
    if (true == wdt.check_passing(1000)) {
        if (false == system_error) {
            wdt_reset();
        }
    }
#endif

    // loop function
    model.loop();
    // control board LED
    state_monitor(model.get_mode());
    delay(1);
}
