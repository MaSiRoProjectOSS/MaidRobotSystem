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

/* ======================================================================== */
/* ------------------------------------------------------------------------ */
// PKG[masiro_packages]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_NAME_CALL_SHELL CONFIG_SKIN_NAME "/system/call_shell" /*!< TOPIC : ${masiro}/system/call_shell */

/* ------------------------------------------------------------------------ */
// PKG[]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_NAME_HITOMI_ANGLE          CONFIG_SKIN_NAME "/eye/cmd_angle"        /*!< TOPIC : ${masiro}/eye/cmd_angle */
#define MRS_TOPIC_NAME_VOICE_ID              CONFIG_SKIN_NAME "/voiceID"              /*!< TOPIC : ${masiro}/voiceID */
#define MRS_TOPIC_NAME_PHYSICAL_DANCING      CONFIG_SKIN_NAME "/physical/dancing"     /*!< TOPIC : ${masiro}/physical/dancing */
#define MRS_TOPIC_NAME_PHYSICAL_STATE_JOINT  CONFIG_SKIN_NAME "/physical/state_joint" /*!< TOPIC : ${masiro}/physical/state_joint */
#define MRS_TOPIC_NAME_SIMULATOR_STATE_JOINT CONFIG_SKIN_NAME "/simulator/jointstate" /*!< TOPIC : ${masiro}/simulator/jointstate */
#define MRS_TOPIC_NAME_SIMULATOR_DANCING     CONFIG_SKIN_NAME "/simulator/dancing"    /*!< TOPIC : ${masiro}/simulator/dancing */

/* ------------------------------------------------------------------------ */
// PKG[fms]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_NAME_RECEPTIONIST_QR   CONFIG_SKIN_NAME "/qr_code_scanner"       /*!< TOPIC : qr_code_reader */
#define MRS_TOPIC_NAME_CAST_SETTING      CONFIG_SKIN_NAME "cast/setting"           /*!< TOPIC : cast/setting */
#define MRS_TOPIC_NAME_FMS_ACTION        CONFIG_SKIN_NAME "/fms/action"            /*!< TOPIC : ${masiro}/fms/action */
#define MRS_TOPIC_NAME_CAST_STATE_MASIRO CONFIG_SKIN_NAME "masiro/will/cast_state" /*!< TOPIC : masiro/will/cast_state */
#define MRS_TOPIC_NAME_CAST_STATE_CIRO   CONFIG_SKIN_NAME "ciro/will/cast_state"   /*!< TOPIC : ciro/will/cast_state */
#define MRS_TOPIC_NAME_CAST_STATE_CIYA   CONFIG_SKIN_NAME "ciya/will/cast_state"   /*!< TOPIC : ciya/will/cast_state */

#define MRS_TOPIC_NAME_RECEPTIONIST_QR_CODE_SCANNER "qr_code_scanner" /*!< TOPIC : qr_code_scanner */
#define MRS_TOPIC_NAME_RECEPTIONIST_GATE_KEEPER     "entered"         /*!< TOPIC : qr_code_scanner */
#define MRS_TOPIC_NAME_RECEPTIONIST_ORDERLY         "ring"            /*!< TOPIC : ring */
#define MRS_TOPIC_NAME_RECEPTIONIST_INSTRUCT        "instruct"        /*!< TOPIC : instruct */

//#define  MRS_TOPIC_NAME_ACTION_MANAGER_NAVIGATION CONFIG_SKIN_NAME "/action_manager/navi"   /*!< TOPIC : ${masiro}/action_manager/navi */
//#define  MRS_TOPIC_NAME_ACTION_MANAGER_WILL       CONFIG_SKIN_NAME "/action_manager/will"   /*!< TOPIC : ${masiro}/action_manager/will */
//#define  MRS_TOPIC_NAME_ACTION_MANAGER_NUCLEO     CONFIG_SKIN_NAME "/action_manager/nucleo" /*!< TOPIC : ${masiro}/action_manager/nucleo */
//#define  MRS_TOPIC_NAME_ROS_NAVIGATION_NAVIGATION CONFIG_SKIN_NAME "/ros_navigation/navi"   /*!< TOPIC : ${masiro}/ros_navigation/navi */
//#define  MRS_TOPIC_NAME_WILL                      CONFIG_SKIN_NAME "/will"                  /*!< TOPIC : ${masiro}/will */
//#define  MRS_TOPIC_NAME_NUCLEO                    CONFIG_SKIN_NAME "/cocoro/nucleo"         /*!< TOPIC : ${masiro}/cocoro/nucleo */

/* ------------------------------------------------------------------------ */
// PKG[]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_NAME_CAST_ACTION    CONFIG_SKIN_NAME "/action"    /*!< TOPIC : action */
#define MRS_TOPIC_NAME_CAST_SENSOR_01 CONFIG_SKIN_NAME "/sensor_01" /*!< TOPIC : sensor_01 */
#define MRS_TOPIC_NAME_CAST_SENSOR_02 CONFIG_SKIN_NAME "/sensor_02" /*!< TOPIC : sensor_02 */
#define MRS_TOPIC_NAME_CAST_SENSOR_03 CONFIG_SKIN_NAME "/sensor_03" /*!< TOPIC : sensor_03 */
#define MRS_TOPIC_NAME_CAST_SENSOR_04 CONFIG_SKIN_NAME "/sensor_04" /*!< TOPIC : sensor_04 */

/* ------------------------------------------------------------------------ */
// from NUC
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_NAME_VOLTAGE       CONFIG_SKIN_NAME "/power_unit/power_monitor/voltage" /*!< TOPIC : ${masiro}/power_unit/power_monitor/voltage */
#define MRS_TOPIC_NAME_LOCATION      "/wheel/now_pose"                                    /*!< TOPIC : /wheel/now_pose */
#define MRS_TOPIC_NAME_RECOGNIZED_AR CONFIG_SKIN_NAME "/eye/recognized_ar"                /*!< TOPIC : ${masiro}/eye/recognized_ar*/
#define MRS_TOPIC_NAME_GOAL_GAZE     "/wheel/goal_gaze"                                   /*!< TOPIC : /wheel/goal_gaze */

/* ------------------------------------------------------------------------ */
// from Nucleo
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_NAME_PHYSICAL_VITAL CONFIG_SKIN_NAME "/vital" /*!< TOPIC : vital */

/* ------------------------------------------------------------------------ */
// PKG[mikumikumoving]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_NAME_MIKUMIKUMOVING CONFIG_SKIN_NAME "/mikumikumoving/dancing" /*!< TOPIC : /mikumikumoving/dancing */

/* ------------------------------------------------------------------------ */
// PKG[nucleo]
/* ------------------------------------------------------------------------ */
#define MRS_TOPIC_NAME_TO_NUCLEO       CONFIG_SKIN_NAME "/action_manager/to_nucleo"       /*!< TOPIC : /masiro/action_manager/to_nucleo */
#define MRS_TOPIC_NAME_FROM_NUCLEO     CONFIG_SKIN_NAME "/action_manager/from_nucleo"     /*!< TOPIC : /masiro/action_manager/from_nucleo */
#define MRS_TOPIC_NAME_TO_NAVIGATION   CONFIG_SKIN_NAME "/action_manager/to_navigation"   /*!< TOPIC : /masiro/action_manager/to_navigation */
#define MRS_TOPIC_NAME_FROM_NAVIGATION CONFIG_SKIN_NAME "/action_manager/from_navigation" /*!< TOPIC : /masiro/action_manager/from_navigation */

/* ======================================================================== */

#endif // MAID_ROBOT_SYSTEM_ROS_TOPIC_NAME_HPP
