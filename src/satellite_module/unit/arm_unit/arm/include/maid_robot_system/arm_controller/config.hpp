/**
 * @file config.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Configuration.
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_CONFIG_HPP
#define ARM_CONTROLLER_CONFIG_HPP

#include "maid_robot_system/common/time_check.hpp"

#include <Arduino.h>

#define JOINT_NUM                (8)
#define DIMENSION_OF_HOMO_MATRIX (4)

/* Glossary of decision */
/* Mode */
const char STOP       = 0;
const char CRUISE     = 1;
const char WAIT       = 2;
const char PC_CONTROL = 3;
const char FOLLOW     = 4;

const char HUG_MODE   = 5;
const char ACTIVE     = -9;
const char DEACTIVATE = -10;

const char RANDOM_MOVE    = 10;
const char FACE_TRACK     = 11;
const char ARM_TRACK      = 12;
const char COMMAND_MOVING = 13;

const char STAND     = 30;
const char SIT       = 31;
const char RISE_KNEE = 32;
const char SITTING   = 33;

const char HANDSHAKE   = 20;
const char GRASPED     = 21;
const char MOVING      = 22;
const char FREE_MOVING = 25;

const char POSE_LOCK = 23;
const char WALKING   = 24;

const char INITIALIZE = 55;
const int FREE        = 777;
const int PASS        = 888;

const int MODE_FOLLOW    = 101;
const int MODE_POSING    = 102;
const int MODE_STOP_POSE = 103;

const int MODE_PWM   = 121;
const int MODE_VEL_O = 122;

const int _ERROR = 8888;

#define PIN_SENSOR_ARM_0         A0
#define PIN_SENSOR_ARM_1         A1
#define PIN_SENSOR_ARM_2         A2
#define PIN_SENSOR_ARM_3         A3
#define PIN_SHOULDER_PUSH_SENSOR A4

const byte PIN_KRS_EN = D3; /* For ICS */

#define PIN_KRS_TX PA9 /* For ICS */
#define PIN_KRS_RX PB7 /* For ICS */

#define PIN_ROS_TX PC10 /* For ROS */
#define PIN_ROS_RX PC11 /* For ROS */

const int START_Z_UPPER = -200;

#ifdef CIRO
const int L_ARM_ID[JOINT_NUM] = { 11, 12, 13, 14, 15, -16, 17, -18 };
const int R_ARM_ID[JOINT_NUM] = { -1, -2, -3, -4, 5, -6, 7, -8 };

const int L_ARM_INI_POS[JOINT_NUM] = { 7663, 6124, 4503, 8863, 3900, 0, 11800, 0 };
const int R_ARM_INI_POS[JOINT_NUM] = { 0, 0, 0, 0, 11200, 0, 2600, 0 };

const float L_ARM_SCALE[JOINT_NUM] = { 1, 1, 1, -1, 1, 0, -1, 0 };
const float R_ARM_SCALE[JOINT_NUM] = { 0, 0, 0, 0, -1, 0, 1, 0 };

#endif

#ifdef CIYA
const int L_ARM_ID[JOINT_NUM] = { -11, -12, -13, -14, 15, -16, 17, -18 };
const int R_ARM_ID[JOINT_NUM] = { 1, 2, 3, 4, 5, -6, 7, -8 };

const int L_ARM_INI_POS[JOINT_NUM] = { 0, 0, 0, 0, 10935, 0, 7388, 0 };
const int R_ARM_INI_POS[JOINT_NUM] = { 7227, 7696, 9411, 10238, 3931, 0, 4050, 0 };

const float L_ARM_SCALE[JOINT_NUM] = { 0, 0, 0, 0, -1, 0, -1, 0 };
const float R_ARM_SCALE[JOINT_NUM] = { -1, 1, -1, -1, 1, 0, 1, 0 };

#endif

#ifdef CIRO
const int SENSOR_ARM_INIT_POS[4]  = { 600, 580, 700, 720 };
const int SENSOR_ARM_DIRECTION[4] = { 1, 1, 1, -1 };
#endif

#ifdef CIYA
const int SENSOR_ARM_INIT_POS[4]  = { 21, 250, 170, 980 };
const int SENSOR_ARM_DIRECTION[4] = { -1, -1, -1, 1 };
#endif

const int NECK_LIMIT_YAW[2]   = { -50, 50 };
const int NECK_LIMIT_PITCH[2] = { -30, 40 };
const int NECK_LIMIT_ROLL[2]  = { -3, 3 };

#endif
