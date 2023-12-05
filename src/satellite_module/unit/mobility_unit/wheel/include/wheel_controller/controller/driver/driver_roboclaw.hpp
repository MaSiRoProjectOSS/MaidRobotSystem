/**
 * @file driver_roboclaw.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Operate Roboclaw device
 * @version 0.1
 * @date 2023-02-17
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MOBILITY_UNIT_DRIVER_ROBOCLAW_HPP
#define MOBILITY_UNIT_DRIVER_ROBOCLAW_HPP

#include "RoboClaw.h"

namespace mobility_unit
{
namespace driver
{

/**
 * @brief Operate Roboclaw device
 *
 */
class DriverRoboclaw {
public:
    DriverRoboclaw(HardwareSerial *h_serial, uint8_t pin_enable, uint8_t id);
#ifdef __AVR__
    DriverRoboclaw(SoftwareSerial *s_serial, uint8_t pin_enable, uint8_t id);
#endif
    void enable(bool enable);
    bool setup();
    uint32_t ReadSpeedM1(uint8_t *status, bool *valid);
    uint32_t ReadSpeedM2(uint8_t *status, bool *valid);
    bool SpeedM1(uint32_t speed);
    bool SpeedM2(uint32_t speed);
    uint16_t ReadMainBatteryVoltage();

private:
    RoboClaw *roboclaw;                  /*!<  */
    uint8_t _pin_enable = __UINT8_MAX__; /*!<  */
    uint8_t _address    = 128;           /*!<  */

private:
    const uint32_t SERIAL_TIMEOUT_MS        = 10000; /*!<  */
    const unsigned long COMMAND_INTERVAL_MS = 200;   /*!<  */
    //Velocity PID coefficients.
    const float KP   = 1.0;   /*!<  */
    const float KI   = 0.5;   /*!<  */
    const float KD   = 0.25;  /*!<  */
    const float QPPS = 44000; /*!<  */
    const int BAUD   = 38400; /*!<  */
};

} // namespace driver
} // namespace mobility_unit

#endif
