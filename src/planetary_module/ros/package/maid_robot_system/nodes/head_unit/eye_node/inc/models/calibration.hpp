/**
 * @file calibration.hpp
 * @brief
 * @date 2020-03-22
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#ifndef MRS_EYE_NODE_MODELS_CALIBRATION_HPP
#define MRS_EYE_NODE_MODELS_CALIBRATION_HPP

#include "maid_robot_system/emotion.hpp"

/* ============================================= */
#define MASIRO_HITOMI_DISPLAY_FPS 60

/* ============================================= */
#define DRAW_PUPIL_INSIDE  1
#define DRAW_PUPIL_OUTSIDE 1
#define NEXT_EMOTION_INIT  miens_normal
/* ============================================= */
/**
 * @brief ROSメッセージ喪失後に前を見るタイムアウト時間
 */
#define LOST_ROS_MSG_TIMEOUT_SECONDS 1.5

/* ============================================= */
#ifdef _WIN32
#define DEBUG_PRINT 1
#define DEBUG_VIEW  1
#else
#define DEBUG_PRINT 0
#define DEBUG_VIEW  0
#endif
/* ============================================= */

namespace maid_robot_system
{
/**
 * @brief 操作対象の目
 *
 */
typedef enum
{
    TARGET_EYE_LEFT,
    TARGET_EYE_RIGHT,

} ENUM_TARGET_EYE;

} // namespace maid_robot_system

#include "calibration_default.hpp"

#endif
