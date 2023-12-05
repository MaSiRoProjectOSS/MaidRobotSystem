/**
 * @file model_implement.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "models/model_implement.hpp"

#include "maid_robot_system_interfaces/angle_calculus.hpp"

namespace maid_robot_system
{
bool ModelImplement::calculate(double seconds)
{
    bool result                = false;
    static double next_seconds = 0.0;

    double dimensions            = this->_temp_overall.EYE_DEFAULT_SIZE;
    double distance              = this->_temp_overall.EYE_DEFAULT_DISTANCE;
    static double target_angle_x = 0.0;
    static double target_angle_y = 0.0;
    static double target_roll    = 0.0;

    StTemporaryEye primary;
    StTemporaryEye secondary;

    if (true == this->param.priority_to_the_right) {
        primary   = this->_temp_right;
        secondary = this->_temp_left;
    } else {
        primary   = this->_temp_left;
        secondary = this->_temp_right;
    }
    if (seconds >= primary.next_seconds) {
        primary.detected = false;
    }
    if (seconds >= secondary.next_seconds) {
        secondary.detected = false;
    }

    // The decision formula was changed.
    // The previous formula was based on the average of two cameras.
    // The formula is now divided into three parts depending on the detection status.
    if ((true == primary.detected) && (true == secondary.detected)) {
        target_angle_x = (primary.x + secondary.x) / 2.0;
        target_angle_y = (primary.y + secondary.y) / 2.0;
        target_roll    = (primary.target_roll + secondary.target_roll) / 2.0;
        result         = true;
#if DEBUG_MODEL_IMPLEMENT
        printf("calculate [ps]\n");
#endif
    } else if (true == primary.detected) {
        target_angle_x = primary.x;
        target_angle_y = primary.y;
        target_roll    = primary.target_roll;
        result         = true;
#if DEBUG_MODEL_IMPLEMENT
        printf("calculate [p-]\n");
#endif
    } else if (true == secondary.detected) {
        target_angle_x = secondary.x;
        target_angle_y = secondary.y;
        target_roll    = secondary.target_roll;
        result         = true;
#if DEBUG_MODEL_IMPLEMENT
        printf("calculate [-s]\n");
#endif
    }

    if (true == result) {
        this->_calculate_eye(target_angle_x, target_angle_y, dimensions, distance);
        this->_calculate_neck(target_angle_x, target_angle_y, target_roll);

        next_seconds = seconds + this->param.timeout_s_chased;
    } else {
        if (seconds >= next_seconds) {
            this->_msg_clear();

            next_seconds = seconds + this->param.timeout_s_chased;
            result       = true;
#if DEBUG_MODEL_IMPLEMENT
            printf("TIMEOUT\n");
#endif
        }
    }

#if DEBUG_MODEL_IMPLEMENT
    static int count = 0;
    count++;
    if (count > 4) {
        this->_output_param();
        count = 0;
    }
#endif

    return result;
}

// =============================
// PRIVATE : msg
// =============================
bool ModelImplement::_msg_clear()
{
    // _msg_eye
    this->_calculate_eye(0, 0, this->_temp_overall.EYE_DEFAULT_SIZE, this->_temp_overall.EYE_DEFAULT_DISTANCE);
    this->_msg_eye.emotions      = maid_robot_system_interfaces::msg::MrsEye::EMOTION_NORMAL;
    this->_msg_eye.cornea_effect = maid_robot_system_interfaces::msg::MrsEye::EFFECT_CORNEA_NORMAL;

    // _msg_neck
    this->_calculate_neck(0, 0, 0);

    // _msg_lip
    this->_msg_lip.percent = 0;

    return true;
}

bool ModelImplement::_tmp_clear()
{
    this->_temp_left.target_roll = this->param.eye_left_offset_angle;
    this->_temp_left.x           = this->param.eye_left_offset_x;
    this->_temp_left.y           = this->param.eye_left_offset_y;

    this->_temp_right.target_roll = this->param.eye_right_offset_angle;
    this->_temp_right.x           = this->param.eye_right_offset_x;
    this->_temp_right.y           = this->param.eye_right_offset_y;

    return true;
}

// =============================
// PUBLIC : Function
// =============================
bool ModelImplement::set_value_pose(ModelStructure::INPUT_TYPE type, const maid_robot_system_interfaces::msg::PoseDetection msg, double seconds)
{
    bool result = false;
    switch (type) {
        case ModelStructure::INPUT_TYPE::POSE_LEFT:
            result = this->_set_value_pose(msg,
                                           this->_temp_left, //
                                           this->param.eye_left_offset_x,
                                           this->param.eye_left_offset_y,
                                           0.0,
                                           this->param.eye_left_offset_angle,
                                           seconds);
            break;
        case ModelStructure::INPUT_TYPE::POSE_RIGHT:
            result = this->_set_value_pose(msg,
                                           this->_temp_right, //
                                           this->param.eye_right_offset_x,
                                           this->param.eye_right_offset_y,
                                           0.0,
                                           this->param.eye_right_offset_angle,
                                           seconds);
            break;
        default:
            break;
    }
#if DEBUG_MODEL_IMPLEMENT
    if (true == result) {
        //printf("ModelImplement::set_value_pose\n");
    }
#endif
    return result;
}

// =============================
// PRIVATE : Function
// =============================
bool ModelImplement::_set_value_pose(const maid_robot_system_interfaces::msg::PoseDetection msg,
                                     StTemporaryEye &temp,
                                     double offset_x,
                                     double offset_y,
                                     double offset_z,
                                     double offset_angle,
                                     double seconds)
{
    bool result = msg.human_detected;

    if (true == result) {
        double x = msg.landmark.nose.x;
        double y = msg.landmark.nose.y;
        double z = msg.landmark.nose.z;

        temp.detected_type = 1;

        if ((true == msg.landmark.right.eye.exist) && (true == msg.landmark.left.eye.exist)) {
            if (msg.landmark.right.eye.x != 0 and msg.landmark.left.eye.x != 0) {
                if (msg.landmark.right.eye.x > msg.landmark.left.eye.x) {
                    float eye_dx     = msg.landmark.right.eye.x - msg.landmark.left.eye.x;
                    float eye_dy     = msg.landmark.right.eye.y - msg.landmark.left.eye.y;
                    temp.target_roll = rad_to_deg(std::atan2(eye_dy, eye_dx)) + offset_angle;
                }
            }
        }

        // Formula for following hands while kneeling
        // Comment out to switch in mode
        //
        // [ATTENTION ]There is no kneecap function now.
        // I would like to make a decision on the knee pillow, including the condition of the legs.
        if (true == msg.landmark.nose.exist) {
            if ((true == msg.landmark.left.shoulder.exist) && (true == msg.landmark.left.index.exist)) {
                if (msg.landmark.nose.y < msg.landmark.left.shoulder.y) {
                    if (msg.landmark.left.index.y < msg.landmark.left.shoulder.y) // Deleted because crossed hands are not judged.
                    {
                        x = msg.landmark.left.index.x;
                        y = msg.landmark.left.index.y;
                        z = msg.landmark.left.index.z;

                        temp.detected_type = 2;
                    }
                }
            }
            if ((true == msg.landmark.right.shoulder.exist) && (true == msg.landmark.right.index.exist)) {
                if (msg.landmark.nose.y < msg.landmark.right.shoulder.y) {
                    if (msg.landmark.right.index.y < msg.landmark.right.shoulder.y) // Deleted because crossed hands are not judged.
                    {
                        x = msg.landmark.right.index.x;
                        y = msg.landmark.right.index.y;
                        z = msg.landmark.right.index.z;

                        temp.detected_type = 3;
                    }
                }
            }
        }

        temp.x = x + offset_x;
        temp.y = y + offset_y;
        temp.z = z + offset_z;
    }

    temp.detected = result;
    if (result != true) {
        temp.detected_type = 0;
    }
    temp.next_seconds = seconds + this->param.timeout_s_received;

    return result;
}

// =============================
// Constructor
// =============================
ModelImplement::ModelImplement()
{
    this->_tmp_clear();
    this->_msg_clear();
}

ModelImplement::~ModelImplement()
{
}

void ModelImplement::_output_param()
{
#if DEBUG_MODEL_IMPLEMENT
#else
    printf("=============================================\n");
    printf("eye_left_offset_x      : %3.3f\n", this->param.eye_left_offset_x);
    printf("eye_left_offset_y      : %3.3f\n", this->param.eye_left_offset_y);
    printf("eye_left_offset_angle  : %3.3f\n", this->param.eye_left_offset_angle);
    printf("eye_right_offset_x     : %3.3f\n", this->param.eye_right_offset_x);
    printf("eye_right_offset_y     : %3.3f\n", this->param.eye_right_offset_y);
    printf("eye_right_offset_angle : %3.3f\n", this->param.eye_right_offset_angle);

    printf("neck_pitch_min  : %d\n", this->param.neck_pitch_min);
    printf("neck_pitch_max  : %d\n", this->param.neck_pitch_max);
    printf("neck_yaw_min    : %d\n", this->param.neck_yaw_min);
    printf("neck_yaw_max    : %d\n", this->param.neck_yaw_max);
    printf("neck_roll_min   : %d\n", this->param.neck_roll_min);
    printf("neck_roll_max   : %d\n", this->param.neck_roll_max);

    printf("lip_min      : %d\n", this->param.lip_min);
    printf("lip_max      : %d\n", this->param.lip_max);
    printf("tiredness    : %d\n", this->param.tiredness);

    printf("priority            : %s\n", this->param.priority_to_the_right ? "right_hand" : "left_hand");
    printf("timeout_s_received  : %3.3f\n", this->param.timeout_s_received);
    printf("timeout_s_chased    : %3.3f\n", this->param.timeout_s_chased);
#endif
}
} // namespace maid_robot_system
