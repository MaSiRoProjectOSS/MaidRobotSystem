/**
 * @file model_structure.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-08-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_HEAD_CONTROL_NODE_MODEL_STRUCTURE_HPP
#define MRS_HEAD_CONTROL_NODE_MODEL_STRUCTURE_HPP

namespace maid_robot_system
{
class ModelStructure {
public:
    enum INPUT_TYPE
    {
        UNDEFINED,
        WATTMETER,
        VOICE,
        AR_LEFT,
        AR_RIGHT,
        POSE_LEFT,
        POSE_RIGHT,
    };
};

class StTemporaryOverall {
public:
    int emotion            = 0;
    bool tired             = false;
    bool flag_eyelid_wink  = false;
    bool flag_eyelid_close = false;
    bool flag_eyelid_smile = false;

    int count_continue_command     = 0;
    int count_continue_eyelid_wink = 0;

    const int COUNT_CONTINUE_MAX      = 5;
    const double EYE_DEFAULT_SIZE     = 1.0;
    const double EYE_DEFAULT_DISTANCE = 1.0;
};

class StTemporaryEye {
public:
    typedef enum detected_type_t
    {
        DETECTED_TYPE_NOT_DETECTED,
        DETECTED_TYPE_LEFT_HAND,
        DETECTED_TYPE_RIGHT_HAND,
        DETECTED_TYPE_NOSE
    } DETECTED_TYPE;

public:
    double x           = 0.0;
    double y           = 0.0;
    double z           = 0.0;
    double target_roll = 0.0;

    bool detected                 = false;
    detected_type_t detected_type = StTemporaryEye::detected_type_t::DETECTED_TYPE_NOT_DETECTED;
    double next_seconds           = 0.0;
    // int emotion;
    // int dimensions;
    // bool flag_reversal;
    /////
    // int distance;
    // double angle_x;
    // double angle_y;
    // double angle_z;
    // double CAMERA_ANGLE_X = 140.0;
    // double CAMERA_ANGLE_Y = 140.0;
    // double offset_x_angle;
    // double offset_y_angle;
    //
    // double offset_center_x;
    // double offset_center_y;
    //double offset_angle;
};

class StParam {
public:
    double eye_left_offset_x     = 0.0;
    double eye_left_offset_y     = 0.0;
    double eye_left_offset_angle = 0.0;

    double eye_right_offset_x     = 0.0;
    double eye_right_offset_y     = 0.0;
    double eye_right_offset_angle = 0.0;

    int neck_pitch_min = -20;
    int neck_pitch_max = 20;
    int neck_yaw_min   = -20;
    int neck_yaw_max   = 20;
    int neck_roll_min  = -20;
    int neck_roll_max  = 20;

    int lip_min = 0;
    int lip_max = 100;

    double tiredness = 25.0;

    bool priority_to_the_right = true;

    double timeout_s_received = 1.0;
    double timeout_s_chased   = 10.0;
};

} // namespace maid_robot_system

#endif
