/**
 * @file model_structure.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-08-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_MODEL_STRUCTURE_HPP
#define MRS_EYE_NODE_MODEL_STRUCTURE_HPP

#include <string>

#define WORKSPACECONFIG_PATH_IMG_EYE NODE_HITOMI_DIR_PATH "/skin/"

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

/* ============================================= */
#define SET_GLWIDGET 1
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

namespace maid_robot_system
{
class StParameter {
public:
    class StPostion {
    public:
        int width  = 0;
        int height = 0;
    };

public:
    std::string path = "--";
    std::string name = "--";
    StPostion left;
    StPostion right;

    std::string setting_file = "";
    int brightness           = 100;
    int color_r              = 255;
    int color_g              = 255;
    int color_b              = 255;

public:
    double l_x                   = CALIBRATION_L_X;
    double l_y                   = CALIBRATION_L_Y;
    double r_x                   = CALIBRATION_R_X;
    double r_y                   = CALIBRATION_R_Y;
    double r_angle               = CALIBRATION_R_DISP_ANGLE;
    double l_angle               = CALIBRATION_L_DISP_ANGLE;
    double eyeball_position_r_x  = CALIBRATION_EYEBALL_X;
    double eyeball_position_r_y  = CALIBRATION_EYEBALL_Y;
    double eyeball_position_l_x  = CALIBRATION_EYEBALL_X;
    double eyeball_position_l_y  = CALIBRATION_EYEBALL_Y;
    double eyeball_angle         = CALIBRATION_EYEBALL_ANGLE;
    int eyelid_size_x            = CALIBRATION_EYELID_SIZE_X;
    int eyelid_size_y            = CALIBRATION_EYELID_SIZE_Y;
    float eye_blink_time_quickly = EYE_BLINK_TIME_MILLISECOND_DEFAULT_QUICKLY;
    float eye_blink_time_min     = EYE_BLINK_TIME_MILLISECOND_DEFAULT_MIN;
    float eye_blink_time_max     = EYE_BLINK_TIME_MILLISECOND_DEFAULT_MAX;
    float eye_blink_time_limit   = EYE_BLINK_TIME_MILLISECOND_DEFAULT_LIMITED;
    float eye_blink_time_offset  = EYE_BLINK_TIME_MILLISECOND_DEFAULT_OFFSET;

    // TODO
    // delete function
    void skin_a()
    {
        this->l_x                    = 135.0;
        this->l_y                    = 455.0;
        this->r_x                    = -46.0;
        this->r_y                    = 455.0;
        this->r_angle                = 0.0;
        this->l_angle                = 0.0;
        this->eyeball_position_l_x   = -100.0;
        this->eyeball_position_l_y   = 100.0;
        this->eyeball_position_r_x   = 100.0;
        this->eyeball_position_r_y   = 100.0;
        this->eyeball_angle          = 0.0;
        this->eyelid_size_x          = 1520;
        this->eyelid_size_y          = 2190;
        this->eye_blink_time_quickly = 150;
        this->eye_blink_time_min     = 400;
        this->eye_blink_time_max     = 600;
        this->eye_blink_time_limit   = 15000;
        this->eye_blink_time_offset  = 0;
    }
    void skin_b()
    {
        this->l_x                    = 120.0;
        this->l_y                    = 305.0;
        this->r_x                    = -95.0;
        this->r_y                    = 517.0;
        this->r_angle                = 0.0;
        this->l_angle                = 0.0;
        this->eyeball_position_l_x   = -45.0;
        this->eyeball_position_l_y   = 95.0;
        this->eyeball_position_r_x   = 45.0;
        this->eyeball_position_r_y   = 95.0;
        this->eyeball_angle          = 0.0;
        this->eyelid_size_x          = 1480;
        this->eyelid_size_y          = 1350;
        this->eye_blink_time_quickly = 50;
        this->eye_blink_time_min     = 500;
        this->eye_blink_time_max     = 700;
        this->eye_blink_time_limit   = 15000;
        this->eye_blink_time_offset  = 0;
    }

    void skin_c()
    {
        this->l_x                    = 145.0;
        this->l_y                    = 265.0;
        this->r_x                    = -186.0;
        this->r_y                    = 261.0;
        this->r_angle                = -5.3;
        this->l_angle                = -2;
        this->eyeball_position_l_x   = -25.0;
        this->eyeball_position_l_y   = 145.0;
        this->eyeball_position_r_x   = 110.0;
        this->eyeball_position_r_y   = 185.0;
        this->eyeball_angle          = 0.0;
        this->eyelid_size_x          = 1480;
        this->eyelid_size_y          = 1350;
        this->eye_blink_time_quickly = 150;
        this->eye_blink_time_min     = 450;
        this->eye_blink_time_max     = 550;
        this->eye_blink_time_limit   = 15000;
        this->eye_blink_time_offset  = 0;
    }
};

} // namespace maid_robot_system

#endif
