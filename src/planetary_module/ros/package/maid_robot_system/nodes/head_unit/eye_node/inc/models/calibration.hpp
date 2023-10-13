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
#define DRAW_CORNEA_INSIDE  (1)
#define DRAW_CORNEA_OUTSIDE (1)
#define NEXT_EMOTION_INIT   (MIENS::miens_normal)
/* ============================================= */

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

#define LOST_ROS_MSG_TIMEOUT_SECONDS (1.5)
#define EYE_BLINK_TIME_START_TIME_MS (1000 * 10)
#define VOICE_MESSAGE_TIMEOUT_MS     (1000 * 3)
/* ============================================= */

namespace maid_robot_system
{
typedef enum
{
    TARGET_EYE_LEFT,
    TARGET_EYE_RIGHT,

} ENUM_TARGET_EYE;

} // namespace maid_robot_system

#endif
