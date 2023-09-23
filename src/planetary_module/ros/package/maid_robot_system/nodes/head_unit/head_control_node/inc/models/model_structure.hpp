/**
 * @file model_structure.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-08-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_NODE_MITUMERU_MODEL_STRUCTURE_HPP
#define MRS_NODE_MITUMERU_MODEL_STRUCTURE_HPP

namespace maid_robot_system
{
class ModelStructure {
public:
    enum INPUT_TYPE
    {
        VOICE,
        AR_LEFT,
        AR_RIGHT,
        POSE_LEFT,
        POSE_RIGHT,
    };
};

class StTemporary {
public:
    int emotion;
    int size;
    int x;
    int y;
    int z;
    bool flag_reversal;
    /////
    int distance;
    double angle_x;
    double angle_y;
    double angle_z;
    double CAMERA_ANGLE_X = 140.0;
    double CAMERA_ANGLE_Y = 140.0;
    double offset_x_angle;
    double offset_y_angle;
    //
    bool detected;
    double center_x;
    double center_y;
    double target_roll;
};

class StParam {
public:
    double oculus_center_x_left    = 0.0;
    double oculus_center_y_left    = 0.0;
    double oculus_offset_up_left   = 0.0;
    double oculus_offset_down_left = 0.0;

    double oculus_center_x_right    = 0.0;
    double oculus_center_y_right    = 0.0;
    double oculus_offset_up_right   = 0.0;
    double oculus_offset_down_right = 0.0;

    int neck_pitch_min = 0;
    int neck_pitch_max = 0;
    int neck_yaw_min   = 0;
    int neck_yaw_max   = 0;
    int neck_roll_min  = 0;
    int neck_roll_max  = 0;

    int lip_min = 0;
    int lip_max = 0;

    bool priority_to_the_right = true;
};

} // namespace maid_robot_system

#endif
