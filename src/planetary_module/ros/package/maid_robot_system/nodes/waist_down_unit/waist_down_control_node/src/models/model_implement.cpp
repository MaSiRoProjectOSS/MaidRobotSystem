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

//
#include <iostream>
//

namespace maid_robot_system
{
// =============================
// PUBLIC : Function
// =============================
void ModelImplement::set_times(double value)
{
    this->_times = value;
}

void ModelImplement::set_offset(double value)
{
    this->_offset = value;
}

void ModelImplement::set_position_rotation(const geometry_msgs::msg::PoseStamped &msg)
{
    this->_robot_position->x = (float)msg.pose.position.x;
    this->_robot_position->y = (float)msg.pose.position.y;
    this->_robot_position->z = (float)msg.pose.position.z;

    this->_robot_rotation->x = (float)msg.pose.orientation.x;
    this->_robot_rotation->y = (float)msg.pose.orientation.y;
    this->_robot_rotation->z = (float)msg.pose.orientation.z;
    this->_robot_rotation->w = (float)msg.pose.orientation.w;
}

void ModelImplement::set_hand_position(const geometry_msgs::msg::Point &msg)
{
    this->_hand_position->x = (float)msg.x;
    this->_hand_position->y = (float)msg.y;
    this->_hand_position->z = (float)msg.z;
}

bool ModelImplement::calculate()
{
    this->_get_hand_information();

    this->_handshake_follow(this->_hand_r, this->_hand_sita, this->_hand_z);

    return true;
}

double ModelImplement::get_times()
{
    return this->_times;
}
double ModelImplement::get_offset()
{
    return this->_offset;
}
double ModelImplement::get_value()
{
    return 0.0;
}

// =============================
// PRIVATE : Function
// =============================
void ModelImplement::_handshake_follow(float r, float sita, float z)
{
    float v, w;

    this->_mode = WAIST_DOWN_UNIT_MODE_FOLLOW;

    float r_error = this->_target_r - r;
    float s_error = this->_target_s - sita;

    v = this->_constrain(r_error * this->_K_v * (float)this->_R_V_FACTOR, -this->_max_speed * this->_R_V_FACTOR, this->_max_speed * this->_R_V_FACTOR) / (float)this->_R_V_FACTOR;
    w = this->_constrain(s_error * this->_K_w * (float)this->_S_W_FACTOR, -this->_max_turn_speed * this->_S_W_FACTOR, this->_max_turn_speed * this->_S_W_FACTOR)
        / (float)this->_S_W_FACTOR;

    v = v * (1.0f - this->_map(this->_constrain(std::abs(s_error), 0, this->_MAX_S_ERROR_TO_V), 0, this->_MAX_S_ERROR_TO_V, this->_V_MIN, this->_V_MAX) / (float)this->_V_MAX);

    if (z > this->_START_Z_UPPER) { /* If arm height is greater than certain value. */
        this->_z_count++;
    } else {
        this->_z_count -= this->_Z_COUNT_DECREASE_RATE;
    }
    this->_z_count = this->_constrain(this->_z_count, this->_Z_COUNT_MIN, this->_Z_COUNT_MAX);

    if (std::abs(v) > this->_R_MOVING_LIMIT || std::abs(w) > this->_S_MOVING_LIMIT) {
        this->_speed_inertia += this->_SPEED_INERTIA_INCREASE_RATE;
    } else {
        /* If wheel is almost stopped, make it hard to move. */
        this->_speed_inertia -= this->_SPEED_INERTIA_DECREASE_RATE;
    }
    this->_speed_inertia = this->_constrain(this->_speed_inertia * this->_SPEED_INERTIA_CONSTRAIN_MAX, this->_SPEED_INERTIA_CONSTRAIN_MIN, this->_SPEED_INERTIA_CONSTRAIN_MAX)
                           / this->_SPEED_INERTIA_CONSTRAIN_MAX;

    v = v * this->_speed_inertia;
    w = w * this->_speed_inertia;

    if (this->_z_count < 1) {
        v                    = 0;
        w                    = 0;
        this->_speed_inertia = 0;
    }

    if (v < 0.0f) {
        v = v / this->_V_DECREASE_FACTOR;
    }

    this->_wheel_target_v = v;
    this->_wheel_target_w = w;
}

void ModelImplement::_get_hand_information()
{
    this->_hand_z    = this->_hand_position->z;
    this->_hand_sita = std::atan2(this->_hand_position->y, this->_hand_position->x) / this->_PI * this->_SEMI_CIRCLE_DEGREE;
    this->_hand_r    = std::sqrt(this->_hand_position->x * this->_hand_position->x + this->_hand_position->y * this->_hand_position->y);
}

float ModelImplement::_constrain(float value, float min, float max)
{
    return std::fmin(std::fmax(value, min), max);
}

float ModelImplement::_map(float value, float from_low, float from_high, float to_low, float to_high)
{
    float normalized_value = (value - from_low) / (from_high - from_low);

    float mapped_value = to_low + normalized_value * (to_high - to_low);

    return mapped_value;
}

// =============================
// Constructor
// =============================
ModelImplement::ModelImplement()
{
}

ModelImplement ::~ModelImplement()
{
}

} // namespace maid_robot_system
