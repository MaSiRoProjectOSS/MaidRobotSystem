/**
 * @file data_structure.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-08-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_DATA_STRUCTURE_HPP
#define MRS_EYE_NODE_DATA_STRUCTURE_HPP

#include "calibration.hpp"
#include "maid_robot_system/common_structure.hpp"

#include <Qt>
#include <string>

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
    StColor color{ 255, 255, 255 };

public:
    double eyeball_size_x              = EYEBALL_SIZE_X;
    double eyeball_size_y              = EYEBALL_SIZE_Y;
    int thinking_next_time_notAccepted = 0;
    // Qt::ImageConversionFlag imageFlag = Qt::NoOpaqueDetection;
    Qt::ImageConversionFlag imageFlag = Qt::OrderedAlphaDither;
    StVector eyeball_center_left{};
    StVector eyeball_center_right{};

    StRectangle screen_size{ 0.0, 0.0, 640.0, 480.0 };

public:
    double l_x     = CALIBRATION_L_X;
    double l_y     = CALIBRATION_L_Y;
    double r_x     = CALIBRATION_R_X;
    double r_y     = CALIBRATION_R_Y;
    double r_angle = CALIBRATION_R_DISP_ANGLE;
    double l_angle = CALIBRATION_L_DISP_ANGLE;
    // St2DPostion eyeball_right{ CALIBRATION_EYEBALL_X, CALIBRATION_EYEBALL_Y, CALIBRATION_EYEBALL_ANGLE };
    // St2DPostion eyeball_left{ CALIBRATION_EYEBALL_X, CALIBRATION_EYEBALL_Y, CALIBRATION_EYEBALL_ANGLE };
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
