/**
 * @file topic_name.hpp
 * @brief
 * @date 2020-02-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_ROS_TOPIC_NAME_HPP
#define MAID_ROBOT_SYSTEM_ROS_TOPIC_NAME_HPP

#include "../config/project_config.hpp"

/* ------------------------------------------------------------------------ */
// unit : head
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_A MRS_PACKAGES_MAID_ROBOT_SYSTEM "/system/call_shell"
#define MRS_TOPIC_B MRS_PACKAGES_MAID_ROBOT_SYSTEM "/system/call_shell"
#define MRS_TOPIC_C MRS_PACKAGES_MAID_ROBOT_SYSTEM "/system/call_shell"

/* ======================================================================== */
/* ------------------------------------------------------------------------ */
// PKG[masiro_packages]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_CALL_SHELL CONFIG_SKIN_NAME "/system/call_shell"
/* ------------------------------------------------------------------------ */
// PKG[]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_HITOMI_ANGLE          CONFIG_SKIN_NAME "/eye/cmd_angle"
#define MRS_TOPIC_VOICE_ID              CONFIG_SKIN_NAME "/voiceID"
#define MRS_TOPIC_PHYSICAL_DANCING      CONFIG_SKIN_NAME "/physical/dancing"
#define MRS_TOPIC_PHYSICAL_STATE_JOINT  CONFIG_SKIN_NAME "/physical/state_joint"
#define MRS_TOPIC_SIMULATOR_STATE_JOINT CONFIG_SKIN_NAME "/simulator/jointstate"
#define MRS_TOPIC_SIMULATOR_DANCING     CONFIG_SKIN_NAME "/simulator/dancing"
/* ------------------------------------------------------------------------ */
// PKG[fms]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_RECEPTIONIST_QR   CONFIG_SKIN_NAME "/qr_code_scanner"
#define MRS_TOPIC_CAST_SETTING      CONFIG_SKIN_NAME "cast/setting"
#define MRS_TOPIC_FMS_ACTION        CONFIG_SKIN_NAME "/fms/action"
#define MRS_TOPIC_CAST_STATE_MASIRO CONFIG_SKIN_NAME "masiro/will/cast_state"
#define MRS_TOPIC_CAST_STATE_CIRO   CONFIG_SKIN_NAME "ciro/will/cast_state"
#define MRS_TOPIC_CAST_STATE_CIYA   CONFIG_SKIN_NAME "ciya/will/cast_state"

#define MRS_TOPIC_RECEPTIONIST_QR_CODE_SCANNER "qr_code_scanner"
#define MRS_TOPIC_RECEPTIONIST_GATE_KEEPER     "entered"
#define MRS_TOPIC_RECEPTIONIST_ORDERLY         "ring"
#define MRS_TOPIC_RECEPTIONIST_INSTRUCT        "instruct"

//#define  MRS_TOPIC_ACTION_MANAGER_NAVIGATION CONFIG_SKIN_NAME "/action_manager/navi"
//#define  MRS_TOPIC_ACTION_MANAGER_WILL       CONFIG_SKIN_NAME "/action_manager/will"
//#define  MRS_TOPIC_ACTION_MANAGER_NUCLEO     CONFIG_SKIN_NAME "/action_manager/nucleo"
//#define  MRS_TOPIC_ROS_NAVIGATION_NAVIGATION CONFIG_SKIN_NAME "/ros_navigation/navi"
//#define  MRS_TOPIC_WILL                      CONFIG_SKIN_NAME "/will"
//#define  MRS_TOPIC_NUCLEO                    CONFIG_SKIN_NAME "/cocoro/nucleo"

/* ------------------------------------------------------------------------ */
// PKG[]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_CAST_ACTION    CONFIG_SKIN_NAME "/action"
#define MRS_TOPIC_CAST_SENSOR_01 CONFIG_SKIN_NAME "/sensor_01"
#define MRS_TOPIC_CAST_SENSOR_02 CONFIG_SKIN_NAME "/sensor_02"
#define MRS_TOPIC_CAST_SENSOR_03 CONFIG_SKIN_NAME "/sensor_03"
#define MRS_TOPIC_CAST_SENSOR_04 CONFIG_SKIN_NAME "/sensor_04"

/* ------------------------------------------------------------------------ */
// from NUC
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_VOLTAGE       CONFIG_SKIN_NAME "/power_unit/power_monitor/voltage"
#define MRS_TOPIC_LOCATION      "/wheel/now_pose"
#define MRS_TOPIC_RECOGNIZED_AR CONFIG_SKIN_NAME "/eye/recognized_ar"
#define MRS_TOPIC_GOAL_GAZE     "/wheel/goal_gaze"

/* ------------------------------------------------------------------------ */
// from Nucleo
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_PHYSICAL_VITAL CONFIG_SKIN_NAME "/vital"

/* ------------------------------------------------------------------------ */
// PKG[mikumikumoving]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_MIKUMIKUMOVING CONFIG_SKIN_NAME "/mikumikumoving/dancing"

/* ------------------------------------------------------------------------ */
// PKG[nucleo]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_TO_NUCLEO       CONFIG_SKIN_NAME "/action_manager/to_nucleo"
#define MRS_TOPIC_FROM_NUCLEO     CONFIG_SKIN_NAME "/action_manager/from_nucleo"
#define MRS_TOPIC_TO_NAVIGATION   CONFIG_SKIN_NAME "/action_manager/to_navigation"
#define MRS_TOPIC_FROM_NAVIGATION CONFIG_SKIN_NAME "/action_manager/from_navigation"

/* ======================================================================== */

/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_SAMPLE_INPUT  "messaging/input"
#define MRS_TOPIC_SAMPLE_OUTPUT "messaging/output"
/* ------------------------------------------------------------------------ */

#endif // MAID_ROBOT_SYSTEM_ROS_TOPIC_NAME_HPP
