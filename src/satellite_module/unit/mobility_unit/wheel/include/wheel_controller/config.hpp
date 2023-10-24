/**
 * @file config.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MOBILITY_UNIT_CONFIG_HPP
#define MOBILITY_UNIT_CONFIG_HPP

#include "maid_robot_system/common/log_callback.hpp"

#include <Arduino.h>

////////////////////////////////////////////////////////////////////
// Parameter
////////////////////////////////////////////////////////////////////
#ifndef WHEEL_MOTOR_REVOLUTION_LEFT
#define WHEEL_MOTOR_REVOLUTION_LEFT 1
#endif
const uint8_t CONFIG_WHEEL_MOTOR_REVOLUTION_LEFT = WHEEL_MOTOR_REVOLUTION_LEFT; /*!< wheel revolution [left] */

#ifndef WHEEL_MOTOR_REVOLUTION_RIGHT
#define WHEEL_MOTOR_REVOLUTION_RIGHT 1
#endif
const uint8_t CONFIG_WHEEL_MOTOR_REVOLUTION_RIGHT = WHEEL_MOTOR_REVOLUTION_RIGHT; /*!< wheel revolution [right] */

////////////////////////////////////////////////////////////////////
// Config
////////////////////////////////////////////////////////////////////
#ifndef ROBOCLAW_ID
#define ROBOCLAW_ID 0x80
#endif
const uint8_t CONFIG_ROBOCLAW_ID = ROBOCLAW_ID; /*!<  */

#ifndef GYRO_ADDRESS
#define GYRO_ADDRESS 0x29
#endif
const uint8_t CONFIG_GYRO_ADDRESS = GYRO_ADDRESS; /*!<  */

#ifndef GYRO_SENSOR_ID
#define GYRO_SENSOR_ID -1
#endif
const int32_t CONFIG_GYRO_SENSOR_ID = GYRO_SENSOR_ID; /*!<  */

////////////////////////////////////////////////////////////////////
// PIN Setting
////////////////////////////////////////////////////////////////////
#ifndef PIN_BOARD_LED
#define PIN_BOARD_LED 13
#endif
const uint8_t SETTING_PIN_LED = PIN_BOARD_LED; /*!<  */

#ifndef PIN_WHEEL_ENABLE
#define PIN_WHEEL_ENABLE 38
#endif
const uint8_t SETTING_PIN_WHEEL_ENABLE = PIN_WHEEL_ENABLE; /*!<  */

#ifndef PIN_COMMUNICATION_CAN_CHIP_CONTROL
#define PIN_COMMUNICATION_CAN_CHIP_CONTROL 53
#endif
const uint8_t SETTING_PIN_COMMUNICATION_CAN_CS = PIN_COMMUNICATION_CAN_CHIP_CONTROL; /*!<  */

////////////////////////////////////////////////////////////////////
// FUNCTION ENABLE
////////////////////////////////////////////////////////////////////

#ifndef SYSTEM_WATCHDOG_ENABLE
#define SYSTEM_WATCHDOG_ENABLE 0
#endif

#endif
