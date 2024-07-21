/**
 * @file replica.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#if MODBUS_ADDRESS != 0x00
#define SETTING_LOOP_TIME_SLEEP_DETECT 10

#include "modbus_impl.hpp"

#include <Arduino.h>
#include <M5Atom.h>
ModbusImpl modbus(&Serial1);

void setup()
{
    (void)M5.begin(true, false, true);
    (void)M5.dis.begin();
    (void)M5.dis.fillpix(CRGB::White);
    log_i("========================================");
    log_i("M5Atom initialized.");
    log_i("  - Start Modbus. Address[%d]", MODBUS_ADDRESS);
    bool result = modbus.begin(MODBUS_ADDRESS, (MessageFrame::MODBUS_TYPE)CUSTOM_MODBUS_TYPE);
    log_i("========================================");
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Green);
}

void loop()
{
    (void)M5.update();
    if (true == M5.Btn.wasPressed()) {
        (void)M5.dis.fillpix(CRGB::Yellow);
        char msg_buffer[512];
        UBaseType_t stack_cushy = uxTaskGetStackHighWaterMark(NULL);
        sprintf(msg_buffer, "STACK SIZE : %d", (int)stack_cushy);
        log_i("%s", msg_buffer);
    }

    delay(SETTING_LOOP_TIME_SLEEP_DETECT);
}

#endif
