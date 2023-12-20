/**
 * @file model_implement_neck.cpp
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
void ModelImplement::get_msg_neck(maid_robot_system_interfaces::msg::MrsNeck &msg)
{
    bool result    = false;
    static float x = 0.0f;
    static float y = 0.0f;
    static float z = 0.0f;
    static float w = 0.0f;

    if (!this->_is_zero(x - this->_msg_neck.x, std::abs(this->_msg_neck.x) * this->_MINIMUM_LIMIT_OF_FLOAT_VALUE)) {
        result = true;
    }
    if (!this->_is_zero(y - this->_msg_neck.y, std::abs(this->_msg_neck.y) * this->_MINIMUM_LIMIT_OF_FLOAT_VALUE)) {
        result = true;
    }
    if (!this->_is_zero(z - this->_msg_neck.z, std::abs(this->_msg_neck.z) * this->_MINIMUM_LIMIT_OF_FLOAT_VALUE)) {
        result = true;
    }
    if (!this->_is_zero(z - this->_msg_neck.w, std::abs(this->_msg_neck.w) * this->_MINIMUM_LIMIT_OF_FLOAT_VALUE)) {
        result = true;
    }

    if (true == result) {
        x = this->_msg_neck.x;
        y = this->_msg_neck.y;
        z = this->_msg_neck.z;
        w = this->_msg_neck.w;
    }

    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.w = w;
}

void ModelImplement::_calculate_neck(double x, double y, double roll)
{
    // TODO :
    // キャリブレーションが必要
    // x,y は -0.5 ～ 0.5 で、どれくらい動かすべきか
    // 現在の角度を保持する必要がある

    // キャリブレーション案
    // 自分の手を検知して、
    // 左右、上下、かしげの動作を、一定角度で行って
    // x,yの変化量を計測する

    // ゼロになるように、角度計算をする
    this->_msg_neck.x = x;
    this->_msg_neck.y = y;
    this->_msg_neck.z = roll;
    this->_msg_neck.w = 0;
}
} // namespace maid_robot_system
