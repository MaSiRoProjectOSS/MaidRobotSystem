/**
 * @file config_zlac706_serial.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-13
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef CONFIG_ZLAC706_SERIAL_HPP
#define CONFIG_ZLAC706_SERIAL_HPP

///////////////////////////////////////////////////////////
// WHEEL setting
///////////////////////////////////////////////////////////

#ifndef SETTING_SYSTEM_POSITION_AROUND
#define SETTING_SYSTEM_POSITION_AROUND (4096)
#endif

#ifndef SETTING_SYSTEM_WHEEL_DIAMETER_MM
#define SETTING_SYSTEM_WHEEL_DIAMETER_MM (206)
#endif

#ifndef SETTING_INTERVAL_LEFT
#define SETTING_INTERVAL_LEFT (true)
#endif
#ifndef SETTING_INTERVAL_RIGHT
#define SETTING_INTERVAL_RIGHT (false)
#endif

#ifndef SETTING_MOTOR_ENABLE_LEFT
#define SETTING_MOTOR_ENABLE_LEFT (1)
#endif
#ifndef SETTING_MOTOR_ENABLE_RIGHT
#define SETTING_MOTOR_ENABLE_RIGHT (1)
#endif

#ifndef SETTING_SPEED_ACCELERATION_MS
#define SETTING_SPEED_ACCELERATION_MS (10000)
#endif
#ifndef SETTING_SPEED_DECELERATION_MS
#define SETTING_SPEED_DECELERATION_MS (10000)
#endif
///////////////////////////////////////////////////////////
// LIMIT
///////////////////////////////////////////////////////////

#ifndef SETTING_SYSTEM_TORQUE_LIMIT_MA
#define SETTING_SYSTEM_TORQUE_LIMIT_MA (750)
#endif
#ifndef SETTING_SYSTEM_SPEED_LIMIT_RPM
#define SETTING_SYSTEM_SPEED_LIMIT_RPM (100)
#endif
#ifndef SETTING_SYSTEM_POSITION_LIMIT_RPM
#define SETTING_SYSTEM_POSITION_LIMIT_RPM (100)
#endif

///////////////////////////////////////////////////////////
// GAIN
///////////////////////////////////////////////////////////
#ifndef SETTING_SPEED_PROPORTIONAL_GAIN
#define SETTING_SPEED_PROPORTIONAL_GAIN (30000)
#endif
#ifndef SETTING_SPEED_INTEGRAL_GAIN
#define SETTING_SPEED_INTEGRAL_GAIN (100)
#endif
#ifndef SETTING_SPEED_DIFFERENTIAL_GAIN
#define SETTING_SPEED_DIFFERENTIAL_GAIN (5000)
#endif

#ifndef SETTING_POSITION_PROPORTIONAL_GAIN
#define SETTING_POSITION_PROPORTIONAL_GAIN (3000)
#endif
#ifndef SETTING_POSITION_DIFFERENTIAL_GAIN
#define SETTING_POSITION_DIFFERENTIAL_GAIN (1)
#endif
#ifndef SETTING_POSITION_FEED_FORWARD_GAIN
#define SETTING_POSITION_FEED_FORWARD_GAIN (0)
#endif

#ifndef SETTING_CURRENT_PROPORTIONAL_GAIN
#define SETTING_CURRENT_PROPORTIONAL_GAIN (500)
#endif
#ifndef SETTING_CURRENT_INTEGRAL_GAIN
#define SETTING_CURRENT_INTEGRAL_GAIN (500)
#endif

#ifndef SETTING_ZLAC_SETTING_FILE
/**
 * @brief Specify zlac settings file path
 *
 */
#define SETTING_ZLAC_SETTING_FILE "/config/zlac.ini"
#endif
///////////////////////////////////////////////////////////
// CONTROL BOARD setting
///////////////////////////////////////////////////////////

#ifndef DEBUG_TEST_BOARD
#define SETTING_FLAG_INVERT (true)
#else
#ifndef SETTING_FLAG_INVERT
#define SETTING_FLAG_INVERT (false)
#else
#define SETTING_FLAG_INVERT DEBUG_TEST_BOARD
#endif
#endif

///////////////////////////////////////////////////////////
// Internal define
///////////////////////////////////////////////////////////

#ifndef SETTING_SYSTEM_WHEEL_DIAMETER_MM_X_PI
#define SETTING_SYSTEM_WHEEL_DIAMETER_MM_X_PI (SETTING_SYSTEM_WHEEL_DIAMETER_MM * PI)
#endif

#endif
