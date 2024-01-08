/**
 * @file primary.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
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
    delay(1000);
    log_d("  - Start Modbus. Address[%d]", MODBUS_ADDRESS);
    bool result = modbus.begin(MODBUS_ADDRESS, ModbusImpl::MODBUS_TYPE::MODBUS_TYPE_RTU_EX);
    log_d("========================================");
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Blue);
}

void loop()
{
    static int count = 0;
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
                //Serial1.print(":2F0002BBFF62B6\r\n");
                break;
            case 1:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(0x03, 0x01, data, 3)) {
                    log_w("No response.");
                }
                //Serial1.print(":030102BBFFF28C\r\n");
                break;
            case 2:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(0x03, 0x02, data, 3)) {
                    log_w("No response.");
                }
                //Serial1.print(":030202bbfff2c8\r\n");
                break;
            case 3:
                data[0] = 0x02;
                data[1] = 0xB0 + count;
                data[2] = 0xFF;
                if (false == modbus.send(0x03, 0x03, data, 3)) {
                    log_w("No response.");
                }
                //Serial1.print(":030302BBFFF334\r\n");
                break;
            default:
                data[0] = 0x00;
                data[1] = 0x00;
                data[2] = 0x12;
                data[3] = 0x34;
                if (false == modbus.send(0x03, 0x08, data, 4)) {
                    log_w("No response.");
                }
                //Serial1.print(":030800001234ec9e\r\n");
                break;
        }
        count++;
        if (count > 4) {
            count = 0;
        }
        delay(500);
        (void)M5.dis.fillpix(CRGB::Blue);
    }

    delay(10);
}
