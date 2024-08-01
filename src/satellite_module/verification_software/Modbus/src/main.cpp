/**
 * @file primary.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#define SETTING_LOOP_TIME_SLEEP_DETECT 10
#define SEND_INTERVAL                  1000

#include "modbus_impl.hpp"

#include <M5Atom.h>
#ifndef MODBUS_TARGET_ADDRESS
#define MODBUS_TARGET_ADDRESS 0x01
#endif

ModbusImpl modbus;

void setup()
{
    (void)M5.begin(true, false, true);
    (void)M5.dis.begin();
    (void)M5.dis.fillpix(CRGB::White);
    log_i("========================================");
    log_i("M5Atom initialized.");
    log_i("  - Start Modbus. Address[%d]", MODBUS_ADDRESS);
    bool result = modbus.begin(&Serial1, MODBUS_ADDRESS, (MessageFrame::MODBUS_TYPE)CUSTOM_MODBUS_TYPE);
    log_i("========================================");
#if MODBUS_ADDRESS == 0x00
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::White);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Black);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::White);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Black);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::White);
#else
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Green);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Black);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Green);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Black);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Green);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Black);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Green);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Black);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Green);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Black);
    delay(200);
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::White);
#endif
}

#if MODBUS_ADDRESS == 0x00
void loop()
{
    const int INTERVAL_MAX    = SEND_INTERVAL / SETTING_LOOP_TIME_SLEEP_DETECT;
    static int interval_count = 0;

    static int address            = MODBUS_TARGET_ADDRESS;
    static int count              = 0;
    static unsigned int data[255] = { 0 };
    (void)M5.update();
    if (true == M5.Btn.wasPressed()) {
        (void)M5.dis.fillpix(CRGB::Yellow);
        char msg_buffer[512];
        UBaseType_t stack_cushy = uxTaskGetStackHighWaterMark(NULL);
        sprintf(msg_buffer, "STACK SIZE : %d", (int)stack_cushy);
        log_i("%s", msg_buffer);
    }

    if (interval_count > INTERVAL_MAX) {
        interval_count = 0;
        char buffer[255];
        switch (count) {
            case 0:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(0x2F, MessageFrame::MODBUS_FUNCTION::FUNCTION_UNKNOWN, data, 3)) {
                    log_w("No response.");
                }
                break;
            case 1:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(address, MessageFrame::MODBUS_FUNCTION::FUNCTION_READ_COILS, data, 3)) {
                    log_w("No response.");
                }
                break;
            case 2:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(address, MessageFrame::MODBUS_FUNCTION::FUNCTION_READ_DISCRETE_INPUTS, data, 3)) {
                    log_w("No response.");
                }
                break;
            case 3:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(address, MessageFrame::MODBUS_FUNCTION::FUNCTION_READ_HOLDING_REGISTERS, data, 3)) {
                    log_w("No response.");
                }
                break;
            default:
                data[0] = 0x00;
                data[1] = 0x00;
                data[2] = 0x12;
                data[3] = 0x34;
                if (false == modbus.send(address, MessageFrame::MODBUS_FUNCTION::FUNCTION_DIAGNOSTICS, data, 4)) {
                    log_w("No response.");
                }
                break;
        }
        count++;
        if (count > 4) {
            count = 0;
        }
    }

    interval_count++;

    delay(SETTING_LOOP_TIME_SLEEP_DETECT);
}
#else
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
