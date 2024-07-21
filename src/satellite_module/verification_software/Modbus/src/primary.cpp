/**
 * @file primary.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#if MODBUS_ADDRESS == 0x00
#define SETTING_LOOP_TIME_SLEEP_DETECT 10

#include "modbus_impl.hpp"

#include <M5Atom.h>

ModbusImpl modbus(&Serial1);

void setup()
{
    (void)M5.begin(true, false, true);
    (void)M5.dis.begin();
    (void)M5.dis.fillpix(CRGB::White);
    log_d("========================================");
    log_d("M5Atom initialized.");
    log_d("  - Start Modbus. Address[%d]", MODBUS_ADDRESS);
    bool result = modbus.begin(MODBUS_ADDRESS, (MessageFrame::MODBUS_TYPE)CUSTOM_MODBUS_TYPE);
    log_d("========================================");
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Blue);
}

void loop()
{
    static int count              = 0;
    static unsigned int data[255] = { 0 };
    (void)M5.update();
    if (true == M5.Btn.wasPressed()) {
        char buffer[255];
        (void)M5.dis.fillpix(CRGB::Yellow);
        switch (count) {
            case 0:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(0x2F, 0x00, data, 3)) {
                    log_w("No response.");
                }
                break;
            case 1:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(0x03, 0x01, data, 3)) {
                    log_w("No response.");
                }
                break;
            case 2:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(0x03, 0x02, data, 3)) {
                    log_w("No response.");
                }
                break;
            case 3:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(0x03, 0x03, data, 3)) {
                    log_w("No response.");
                }
                break;
            default:
                data[0] = 0x00;
                data[1] = 0x00;
                data[2] = 0x12;
                data[3] = 0x34;
                if (false == modbus.send(0x03, 0x08, data, 4)) {
                    log_w("No response.");
                }
                break;
        }
        count++;
        if (count > 4) {
            count = 0;
        }
        (void)M5.dis.fillpix(CRGB::Blue);
        char msg_buffer[512];
        UBaseType_t stack_cushy = uxTaskGetStackHighWaterMark(NULL);
        sprintf(msg_buffer, "STACK SIZE : %d", (int)stack_cushy);
        log_i("%s", msg_buffer);
    }

    delay(SETTING_LOOP_TIME_SLEEP_DETECT);
}

#endif
