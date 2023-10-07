/**
 * @file calibration_default.h
 * @brief
 * @date 2020-03-22
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#ifndef MRS_EYE_NODE_MODELS_CALIBRATION_DEFAULT_HPP
#define MRS_EYE_NODE_MODELS_CALIBRATION_DEFAULT_HPP

/**
 * @brief Image path of eyes data
 *
 */
#define WORKSPACECONFIG_PATH_IMG_EYE "/skin/"
#ifndef WORKSPACECONFIG_PATH_IMG_EYE
#define WORKSPACECONFIG_PATH_IMG_EYE
#endif

/* ============================================= */

#if DEBUG_PRINT
#define CTRL_HITOMI_LOG_FPS 10.0
#else
#define CTRL_HITOMI_LOG_FPS 600.0
#endif
#define CTRL_HITOMI_CMD_VOICE_CLEAR 3.0
/* ============================================= */

#define CALIBRATION_L_X          135.0
#define CALIBRATION_L_Y          465.0
#define CALIBRATION_R_X          -46.0
#define CALIBRATION_R_Y          445.0
#define CALIBRATION_L_DISP_ANGLE 0.0
#define CALIBRATION_R_DISP_ANGLE 0.0

#define CALIBRATION_EYEBALL_X     300
#define CALIBRATION_EYEBALL_Y     300
#define CALIBRATION_EYEBALL_ANGLE 0

#define CALIBRATION_EYELID_SIZE_X 1520
#define CALIBRATION_EYELID_SIZE_Y 2190

/* ============================================= */
#define WINDOW_SIZE_X 2880.0
#define WINDOW_SIZE_Y 1440.0
/* ============================================= */

#define EYEBALL_SIZE_X              1250.0
#define EYEBALL_SIZE_Y              1300.0
#define EYEBALL_DIMENSIONS_MIN      0.5f
#define EYEBALL_DIMENSIONS_MAX      1.5f
#define EYEBALL_DIMENSIONS_DEFAULT  1.0f
#define EYEBALL_DIMENSIONS_INCREASE 0.015f
/* ============================================= */
/**
 * @brief 瞬きのデフォルト時間
 */
#define EYE_BLINK_TIME_MILLISECOND_DEFAULT         500.0f
#define EYE_BLINK_TIME_MILLISECOND_DEFAULT_MIN     (EYE_BLINK_TIME_MILLISECOND_DEFAULT - 100)
#define EYE_BLINK_TIME_MILLISECOND_DEFAULT_MAX     (EYE_BLINK_TIME_MILLISECOND_DEFAULT + 100)
#define EYE_BLINK_TIME_MILLISECOND_DEFAULT_LIMITED 15000
#define EYE_BLINK_TIME_MILLISECOND_DEFAULT_OFFSET  0

#define EYE_BLINK_TIME_MILLISECOND_DEFAULT_QUICKLY 150.0f
/* ============================================= */
/**
 * @brief
 */
#define EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_LOWER     1000.0
#define EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_UPPER     2500.0
#define EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_LOWER 3000.0
#define EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_UPPER 6000.0
/* ============================================= */

#define PUPIL_LING_SIZE_OUTSIDE 750
#define PUPIL_LING_SIZE_INSIDE  550
/* ============================================= */

#endif
