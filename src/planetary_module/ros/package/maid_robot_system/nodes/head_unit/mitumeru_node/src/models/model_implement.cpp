/**
 * @file modeleft.implement.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "models/model_implement.hpp"

namespace maid_robot_system
{
bool ModelImplement::calculate()
{
    bool result           = false;
    double target_angle_x = 0.0;
    double target_angle_y = 0.0;
    double target_roll    = 0.0;
    StTemporary primary;
    StTemporary secondary;
    if (true == this->param.priority_to_the_right) {
        primary   = this->_temp_right;
        secondary = this->_temp_left;
    } else {
        primary   = this->_temp_left;
        secondary = this->_temp_right;
    }

    // The decision formula was changed.
    // The previous formula was based on the average of two cameras.
    // The formula is now divided into three parts depending on the detection status.
    if ((true == primary.detected) && (true == secondary.detected)) {
        target_angle_x = (primary.x + secondary.y) / 2.0;
        target_angle_y = (primary.y + secondary.y) / 2.0;
        target_roll    = (primary.target_roll + secondary.target_roll) / 2.0;
        result         = true;
    } else if (true == primary.detected) {
        target_angle_x = primary.x;
        target_angle_y = primary.y;
        target_roll    = primary.target_roll;
        result         = true;
    } else if (true == secondary.detected) {
        target_angle_x = secondary.x;
        target_angle_y = secondary.y;
        target_roll    = secondary.target_roll;
        result         = true;
    }
    if (true == result) {
        this->_msg_oculus.emotions = 0;
        this->_msg_oculus.size     = 1;
        this->_msg_oculus.distance = 0.98;
        this->_msg_oculus.x        = target_angle_x;
        this->_msg_oculus.y        = target_angle_y;

        // TODO :
        this->_msg_neck.x = target_angle_x;
        this->_msg_neck.y = target_angle_y;
        this->_msg_neck.z = target_roll;
    } else {
        if (false) {
            // TODO :
            // 一定時間経過しても判定がないならば、
            // 位置の初期化
            this->_msg_oculus.emotions = 0;
            this->_msg_oculus.size     = 1;
            this->_msg_oculus.distance = 0.98;
            this->_msg_oculus.x        = 0;
            this->_msg_oculus.y        = 0;
            //
            this->_msg_neck.x = 0;
            this->_msg_neck.y = 0;
            this->_msg_neck.z = 0;
            result            = true;
        }
    }

    return result;
}
// =============================
// PUBLIC : Function
// =============================
bool ModelImplement::set_value_voice(std::string text, int command)
{
    bool result = false;
    switch (this->_mode) {
        case MRS_MODE::MRS_MODE_NONE:
            break;
        default:
            break;
    }
    return result;
}
bool ModelImplement::set_value_ar(ModelStructure::INPUT_TYPE type, int id)
{
    bool result = false;
    switch (type) {
        case ModelStructure::INPUT_TYPE::AR_LEFT:
            result = true;
            break;
        case ModelStructure::INPUT_TYPE::AR_RIGHT:
            result = true;
            break;
        default:
            break;
    }
    return result;
}
bool ModelImplement::set_value_pose(ModelStructure::INPUT_TYPE type, const maid_robot_system_interfaces::msg::PoseDetection msg)
{
    bool result = false;
    switch (type) {
        case ModelStructure::INPUT_TYPE::POSE_LEFT:
            result = this->_calculate_pose(msg, this->_temp_left);
            break;
        case ModelStructure::INPUT_TYPE::POSE_RIGHT:
            result = this->_calculate_pose(msg, this->_temp_right);
            break;
        default:
            break;
    }
    return result;
}

void ModelImplement::get_msg_oculus(maid_robot_system_interfaces::msg::MrsHitomi &msg)
{
    msg.emotions = this->_msg_oculus.emotions;
    msg.size     = this->_msg_oculus.size;
    msg.distance = this->_msg_oculus.distance;
    msg.x        = this->_msg_oculus.x;
    msg.y        = this->_msg_oculus.y;
}
void ModelImplement::get_msg_neck(maid_robot_system_interfaces::msg::MrsKubi &msg)
{
    //  this->param.neck_pitch_min  = 0;
    //  this->param. neck_pitch_max = 0;
    //   this->param.neck_yaw_min    = 0;
    //   this->param.neck_yaw_max    = 0;
    //   this->param. neck_roll_min  = 0;
    //   this->param.neck_roll_max = 0;

    msg.x = this->_msg_neck.x;
    msg.y = this->_msg_neck.y;
    msg.z = this->_msg_neck.z;
    msg.w = this->_msg_neck.w;
}
void ModelImplement::get_msg_lip(maid_robot_system_interfaces::msg::MrsKuchibiru &msg)
{
    msg.percent = std::min(std::max(this->_msg_lip.percent, this->param.lip_min), this->param.lip_max);
}

// =============================
// PRIVATE : Function
// =============================
bool ModelImplement::_calculate_pose(const maid_robot_system_interfaces::msg::PoseDetection msg, StTemporary &temp)
{
    // ターゲットの位置を決定する。
    bool result = msg.human_detected;
    if (true == result) {
        double x = msg.landmark.nose.x;
        double y = msg.landmark.nose.y;
        double z = msg.landmark.nose.z;

        if ((msg.landmark.right.eye.exist == 1) && (msg.landmark.left.eye.exist == 1)) {
            if (msg.landmark.right.eye.x != 0 and msg.landmark.left.eye.x != 0) {
                if (msg.landmark.right.eye.x > msg.landmark.left.eye.x) {
                    float eye_dx     = msg.landmark.right.eye.x - msg.landmark.left.eye.x;
                    float eye_dy     = msg.landmark.right.eye.y - msg.landmark.left.eye.y;
                    temp.target_roll = std::atan2(eye_dy, eye_dx);
                }
            }
        }
#if 0
    // TODO:
    // Formula for following hands while kneeling
    // Comment out to switch in mode
    //
    // [ATTENTION ]There is no kneecap function now.
    // I would like to make a decision on the knee pillow, including the condition of the legs.
    if (true == msg.landmark.nose.exist) {
        if ((true == msg.landmark.left.shoulder.exist) && (true == msg.landmark.left.hand.exist)) {
            if (get_person_data.nose.y < msg.landmark.left.shoulder.y) {
                // if (msg.landmark.left.hand.y < msg.landmark.left.shoulder.y) // Deleted because crossed hands are not judged.
                {
                    x = data.landmark.left.hand.x;
                    y = data.landmark.left.hand.y;
                    z = data.landmark.left.hand.z;
                }
            }
        }
        if ((true == msg.landmark.right.shoulder.exist) && (true == msg.landmark.right.hand.exist)) {
            if (get_person_data.nose.y < msg.landmark.right.shoulder.y) {
                // if (msg.landmark.right.hand.y < msg.landmark.right.shoulder.y) // Deleted because crossed hands are not judged.
                {
                    x = data.landmark.right.hand.x;
                    y = data.landmark.right.hand.y;
                    z = data.landmark.right.hand.z;
                }
            }
        }
    }
#endif

        temp.x = (x - 0.5) * temp.center_x;
        temp.y = (y - 0.5) * temp.center_y;
    }
    temp.detected = result;

    return result;
}

// =============================
// Constructor
// =============================
ModelImplement::ModelImplement()
{
    this->_temp_left.offset_x_angle  = -40.0;
    this->_temp_left.offset_y_angle  = 0.0;
    this->_temp_right.offset_x_angle = -12.0;
    this->_temp_right.offset_y_angle = 0.0;
}

ModelImplement ::~ModelImplement()
{
}

} // namespace maid_robot_system
