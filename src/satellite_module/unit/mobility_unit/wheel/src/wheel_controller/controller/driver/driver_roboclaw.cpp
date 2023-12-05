/**
 * @file driver_roboclaw.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Operate Roboclaw device
 * @version 0.1
 * @date 2023-02-17
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "wheel_controller/controller/driver/driver_roboclaw.hpp"

#include <Arduino.h>

namespace mobility_unit
{
namespace driver
{

DriverRoboclaw::DriverRoboclaw(HardwareSerial *h_serial, uint8_t pin_enable, uint8_t id)
{
    this->_address    = id;
    this->_pin_enable = pin_enable;
    pinMode(this->_pin_enable, OUTPUT);
    this->roboclaw = new RoboClaw(h_serial, this->SERIAL_TIMEOUT_MS);
}
#ifdef __AVR__
DriverRoboclaw::DriverRoboclaw(SoftwareSerial *s_serial, uint8_t pin_enable, uint8_t id)
{
    this->_address    = id;
    this->_pin_enable = pin_enable;
    pinMode(this->_pin_enable, OUTPUT);
    this->roboclaw = new RoboClaw(s_serial, this->SERIAL_TIMEOUT_MS);
}
#endif
void DriverRoboclaw::enable(bool enable)
{
    digitalWrite(this->_pin_enable, (true == enable) ? HIGH : LOW);
}

bool DriverRoboclaw::setup()
{
    bool result = false;
    if (NULL != this->roboclaw) {
        this->roboclaw->begin(this->BAUD);
        delay(this->COMMAND_INTERVAL_MS);
        this->roboclaw->SetM1VelocityPID(this->_address, this->KD, this->KP, this->KI, this->QPPS);
        delay(this->COMMAND_INTERVAL_MS);
        this->roboclaw->SetM2VelocityPID(this->_address, this->KD, this->KP, this->KI, this->QPPS);
        delay(this->COMMAND_INTERVAL_MS);
        this->roboclaw->SetM1MaxCurrent(this->_address, 5500);
        delay(this->COMMAND_INTERVAL_MS);
        this->roboclaw->SetM2MaxCurrent(this->_address, 5500);
        delay(this->COMMAND_INTERVAL_MS);
        delay(this->COMMAND_INTERVAL_MS);
        this->roboclaw->SetM1EncoderMode(this->_address, 64);
        delay(this->COMMAND_INTERVAL_MS);
        this->roboclaw->SetM2EncoderMode(this->_address, 64);
        result = true;
    }
    return result;
}

uint32_t DriverRoboclaw::ReadSpeedM1(uint8_t *status, bool *valid)
{
    return this->roboclaw->ReadSpeedM1(this->_address, status, valid);
}
uint32_t DriverRoboclaw::ReadSpeedM2(uint8_t *status, bool *valid)
{
    return this->roboclaw->ReadSpeedM1(this->_address, status, valid);
}

bool DriverRoboclaw::SpeedM1(uint32_t speed)
{
    return this->roboclaw->SpeedM1(this->_address, speed);
}
bool DriverRoboclaw::SpeedM2(uint32_t speed)
{
    return this->roboclaw->SpeedM2(this->_address, speed);
}

uint16_t DriverRoboclaw::ReadMainBatteryVoltage()
{
    return this->roboclaw->ReadMainBatteryVoltage(this->_address);
}

} // namespace driver
} // namespace mobility_unit
