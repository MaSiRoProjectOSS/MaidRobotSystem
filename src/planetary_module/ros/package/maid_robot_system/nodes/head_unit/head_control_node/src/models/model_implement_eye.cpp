/**
 * @file model_implement_eye.cpp
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
void ModelImplement::get_msg_eye(maid_robot_system_interfaces::msg::MrsEye &msg)
{
    bool result = false;

    static int emotions = 0;
    static int size     = 0;
    static int distance = 0;
    static int left_y   = 0;
    static int left_z   = 0;
    static int right_y  = 0;
    static int right_z  = 0;
    if (emotions != this->_msg_eye.emotions) {
        result = true;
    }
    if (size != this->_msg_eye.size) {
        result = true;
    }
    if (distance != this->_msg_eye.distance) {
        result = true;
    }
    if (left_y != this->_msg_eye.left_y) {
        result = true;
    }
    if (left_z != this->_msg_eye.left_z) {
        result = true;
    }
    if (right_y != this->_msg_eye.right_y) {
        result = true;
    }
    if (right_z != this->_msg_eye.right_z) {
        result = true;
    }
    if (true == result) {
        emotions = this->_msg_eye.emotions;
        size     = this->_msg_eye.size;
        distance = this->_msg_eye.distance;
        left_y   = this->_msg_eye.left_y;
        left_z   = this->_msg_eye.left_z;
        right_y  = this->_msg_eye.right_y;
        right_z  = this->_msg_eye.right_z;
    }

    msg.emotions = emotions;
    msg.size     = size;
    msg.distance = distance;
    msg.left_y   = left_y;
    msg.left_z   = left_z;
    msg.right_y  = right_y;
    msg.right_z  = right_z;
}
void ModelImplement::_calculate_eye(double x, double y, double size, double distance)
{
    // this->_msg_eye.emotions = maid_robot_system_interfaces::msg::MrsEye::EMOTION_CLOSE_LEFT;
    // this->_msg_eye.emotions = maid_robot_system_interfaces::msg::MrsEye::EMOTION_CLOSE_RIGHT;

    // ===========================================
    // EMOTIONS
    // ===========================================
    if (true == this->_temp_overall.tired) {
        this->_msg_eye.emotions = maid_robot_system_interfaces::msg::MrsEye::EMOTION_CLOSE;
    } else if (true == this->_temp_overall.flag_eyelid_close) {
        this->_msg_eye.emotions = maid_robot_system_interfaces::msg::MrsEye::EMOTION_CLOSE;
    } else if (true == this->_temp_overall.flag_eyelid_smile) {
        this->_msg_eye.emotions = maid_robot_system_interfaces::msg::MrsEye::EMOTION_SMILE;
    } else if (true == this->_temp_overall.flag_eyelid_wink) {
        // TODO :Experimental function
        // How to make a cute wink ?
#if 1
        this->_msg_eye.emotions = maid_robot_system_interfaces::msg::MrsEye::EMOTION_NORMAL;
#else
        this->_msg_eye.emotions = maid_robot_system_interfaces::msg::MrsEye::EMOTION_WINK_LEFT;
        this->_msg_eye.emotions = maid_robot_system_interfaces::msg::MrsEye::EMOTION_WINK_RIGHT;
#endif
        if (this->_temp_overall.count_continue_eyelid_wink <= 0) {
            this->_temp_overall.flag_eyelid_wink = false;
            printf("<WINK> Clear\n");
        } else {
            this->_temp_overall.count_continue_eyelid_wink--;
        }
    } else {
        this->_msg_eye.emotions = maid_robot_system_interfaces::msg::MrsEye::EMOTION_NORMAL;
    }

    // ===========================================
    // EFFECT
    // ===========================================
    if (this->_temp_overall.count_continue_command <= 0) {
        this->_temp_overall.flag_eyelid_wink = false;
        this->_msg_eye.cornea_effect         = maid_robot_system_interfaces::msg::MrsEye::EFFECT_CORNEA_NORMAL;
    } else {
        this->_temp_overall.count_continue_command--;
        this->_msg_eye.cornea_effect = maid_robot_system_interfaces::msg::MrsEye::EFFECT_CORNEA_ORDER;
    }

    // ===========================================
    // Postion and size
    // ===========================================
    this->_msg_eye.left_y   = x;        // Eye position X coordinate (left)
    this->_msg_eye.left_z   = -y;       // Eye position Y coordinate (left)
    this->_msg_eye.right_y  = x;        // Eye position X coordinate (right)
    this->_msg_eye.right_z  = -y;       // Eye position Y coordinate (right)
    this->_msg_eye.size     = size;     // Eyeball size
    this->_msg_eye.distance = distance; // About the distance from the eyeball
}

bool ModelImplement::set_value_tiredness(float data, double seconds)
{
    // TODO
    static float tiredness = 100;
    bool result            = false;
    if (data != tiredness) {
        tiredness = data;
        if (tiredness <= this->param.tiredness) {
            this->_temp_overall.tired = true;
        } else {
            this->_temp_overall.tired = false;
        }
        result = true;
    }

    return result;
}
} // namespace maid_robot_system
