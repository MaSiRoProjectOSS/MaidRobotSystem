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
#include "lower_body_controller/config.hpp"
#include "lower_body_controller/model_on_lower_body_controller.hpp"
#include "maid_robot_system/common/time_check.hpp"

#include <Arduino.h>

lower_body_unit::ModelOnLowerBodyController model;

uint8_t mode_led = LOW;

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
    // setup LED
    pinMode(SETTING_PIN_LED, OUTPUT);
    state_led(LOW);

    (void)model.setup();

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
    // loop function
    model.loop();
    // control board LED
    state_monitor(model.get_mode());
    delay(1);
}
