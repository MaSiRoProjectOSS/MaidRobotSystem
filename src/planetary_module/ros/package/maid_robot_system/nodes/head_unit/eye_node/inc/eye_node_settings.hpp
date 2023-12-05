/**
 * @file eye_node_settings.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-10-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_SETTINGS_HPP
#define MRS_EYE_NODE_SETTINGS_HPP

#include "maid_robot_system/emotion.hpp"

namespace maid_robot_system
{
#define DEBUG_OUTPUT_OPEN_GL    1
#define DEBUG_OUTPUT_WIDGET     0
#define DEBUG_OUTPUT_LOAD_IMAGE 0
#define DEBUG_OUTPUT_PARAM_LV   0

#define DEBUG_OUTPUT_BEHAVIOR 1

#define LOGGER_ROS_INFO_OUTPUT_REPORT_TIME   (1000 * 10)
#define LOGGER_ROS_INFO_DETAIL               0
#define LOGGER_ROS_INFO_PARAMETER            0
#define LOGGER_ROS_INFO_SUBSCRIPTION_MRS_EYE 0
#define LOGGER_ROS_INFO_CALL_FUNCTION        0

/* ============================================= */
#define DRAWING_MAX_FPS                      (60)
#define COUNTER_TIMEOUT_QT_THRESHOLD_PERCENT (2)
#define COUNTER_TIMEOUT_QT_TIMES             (3)

#define DRAW_CORNEA_INSIDE  (1)
#define DRAW_CORNEA_OUTSIDE (1)
#define NEXT_EMOTION_INIT   (MIENS::miens_normal)
/* ============================================= */

#define EYEBALL_DIMENSIONS_MIN      (0.5f)
#define EYEBALL_DIMENSIONS_MAX      (1.5f)
#define EYEBALL_DIMENSIONS_DEFAULT  (1.0f)
#define EYEBALL_DIMENSIONS_INCREASE (0.015f)

#define EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_LOWER     (1000.0)
#define EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_UPPER     (2500.0)
#define EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_LOWER (3000.0)
#define EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_UPPER (6000.0)
/* ============================================= */

#define LOST_ROS_MSG_TIMEOUT_SECONDS (1000 * 2.5)
#define EYE_BLINK_TIME_START_TIME_MS (1000 * 5.0)
#define VOICE_MESSAGE_TIMEOUT_MS     (1000 * 3)
/* ============================================= */
} // namespace maid_robot_system

#endif
