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
#ifndef LOWER_BODY_UNIT_CONFIG_HPP
#define LOWER_BODY_UNIT_CONFIG_HPP

#include "maid_robot_system/common/log_callback.hpp"

#include <Arduino.h>

////////////////////////////////////////////////////////////////////
// Parameter
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
// Config
////////////////////////////////////////////////////////////////////
const int WHEEL_MOTOR_REVOLUTION_LEFT  = -1; /*!<  */
const int WHEEL_MOTOR_REVOLUTION_RIGHT = -1; /*!<  */
const int LEG_SENSOR_FW                = 1;  /*!<  */
const int LEG_MOTOR_FW                 = -1; /*!<  */

const int LEG_POSITION_LIMIT[2] = { 650, 840 }; /*!<  */
const int LEG_POSITION_MAX      = 190;          /*!<  */
const int LEG_PWM_LIMIT         = 3000;         /*!<  */

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

#ifndef PIN_WAIST_PITCH
#define PIN_WAIST_PITCH 44
#endif
const uint8_t SETTING_PIN_WAIST_PITCH = PIN_WAIST_PITCH; /*!<  */

#ifndef PIN_STEPPER_SERVO_PLUS
#define PIN_STEPPER_SERVO_PLUS 34
#endif
const uint8_t SETTING_PIN_STEPPER_SERVO_PLUS = PIN_STEPPER_SERVO_PLUS; /*!<  */

#ifndef PIN_STEPPER_SERVO_DIR
#define PIN_STEPPER_SERVO_DIR 35
#endif
const uint8_t SETTING_PIN_STEPPER_SERVO_DIR = PIN_STEPPER_SERVO_DIR; /*!<  */

#ifndef PIN_STEPPER_SERVO_ENABLE
#define PIN_STEPPER_SERVO_ENABLE 36
#endif
const uint8_t SETTING_PIN_STEPPER_SERVO_ENABLE = PIN_STEPPER_SERVO_ENABLE; /*!<  */

#ifndef PIN_STEPPER_SERVO_ERROR
#define PIN_STEPPER_SERVO_ERROR A3
#endif
const uint8_t SETTING_PIN_STEPPER_SERVO_ERROR = PIN_STEPPER_SERVO_ERROR; /*!<  */

#ifndef PIN_LEG_POSITION
#define PIN_LEG_POSITION A2
#endif
const uint8_t SETTING_PIN_LEG_POSITION = PIN_LEG_POSITION; /*!<  */

#ifndef PIN_LEG_PWM_RIGHT
#define PIN_LEG_PWM_RIGHT 6
#endif
const uint8_t SETTING_PIN_LEG_PWM_RIGHT = PIN_LEG_PWM_RIGHT; /*!<  */

#ifndef PIN_LEG_PWM_LEFT
#define PIN_LEG_PWM_LEFT 7
#endif
const uint8_t SETTING_PIN_LEG_PWM_LEFT = PIN_LEG_PWM_LEFT; /*!<  */

#ifndef PIN_COMMUNICATION_CAN_CHIP_CONTROL
#define PIN_COMMUNICATION_CAN_CHIP_CONTROL 53
#endif
const uint8_t SETTING_PIN_COMMUNICATION_CAN_CS = PIN_COMMUNICATION_CAN_CHIP_CONTROL; /*!<  */

////////////////////////////////////////////////////////////////////
// FUNCTION ENABLE
////////////////////////////////////////////////////////////////////

#endif
