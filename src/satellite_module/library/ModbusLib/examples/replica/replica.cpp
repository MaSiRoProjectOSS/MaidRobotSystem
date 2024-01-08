/**
 * @file replica.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "modbus_impl.hpp"

#include <Arduino.h>
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
    (void)M5.dis.fillpix((false == result) ? CRGB::Red : CRGB::Green);
}

void loop()
{
    delay(1000);
}
