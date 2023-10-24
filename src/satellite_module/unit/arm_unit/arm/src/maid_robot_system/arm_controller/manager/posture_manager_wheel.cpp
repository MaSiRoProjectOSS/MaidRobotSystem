/**
 * @file posture_manager_wheel.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Menage posture of wheel.
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/manager/posture_manager_wheel.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

PostureManager_Wheel::PostureManager_Wheel()
{
}
PostureManager_Wheel::~PostureManager_Wheel()
{
}

bool PostureManager_Wheel::_begin()
{
    bool result = true;
    return result;
}

bool PostureManager_Wheel::_calculate()
{
    bool result = true;

    float d_yaw       = this->_old_gy_yaw - this->_args->input.wheel_gy.yaw;
    this->_old_gy_yaw = this->_args->input.wheel_gy.yaw;

    if (d_yaw > this->_YAW_CHANGED_LIMIT_MAX) {
        this->_yaw_rotation += 1;
    }
    if (d_yaw < this->_YAW_CHANGED_LIMIT_MIN) {
        this->_yaw_rotation -= 1;
    }

    this->_yaw_control(this->_args);

    return result;
}

void PostureManager_Wheel::move_stop(PostureManagerArguments *args)
{
    this->_mode     = STOP;
    this->_v.target = 0;
    this->_w.target = 0;

    args->input.wheel_target_v = 0;
    args->input.wheel_target_w = 0;

    this->_v.now = this->_v.target;
    this->_w.now = this->_w.target;
}

void PostureManager_Wheel::set_follow_target(PostureManagerArguments *args, float r, float sita, float z)
{
    this->_follow_target_data[0] = r;
    this->_follow_target_data[1] = sita;
    this->_follow_target_data[2] = z;
}

void PostureManager_Wheel::handshake_follow(PostureManagerArguments *args)
{
    this->_handshake_follow(args, this->_follow_target_data[0], this->_follow_target_data[1], this->_follow_target_data[2]);
}

void PostureManager_Wheel::_handshake_follow(PostureManagerArguments *args, float r, float sita, float z)
{
    float v, w;

    this->_mode = FOLLOW;

    float r_error = this->_target_r - r;
    float s_error = this->_target_s - sita;

    v = constrain(r_error * this->_K_v * (float)this->_R_V_FACTOR, -this->_max_speed * this->_R_V_FACTOR, this->_max_speed * this->_R_V_FACTOR) / (float)this->_R_V_FACTOR;
    w = constrain(s_error * this->_K_w * (float)this->_S_W_FACTOR, -this->_max_turn_speed * this->_S_W_FACTOR, this->_max_turn_speed * this->_S_W_FACTOR)
      / (float)this->_S_W_FACTOR;

    v = v * (1.0 - map(constrain(abs(s_error), 0, this->_MAX_S_ERROR_TO_V), 0, this->_MAX_S_ERROR_TO_V, this->_V_MIN, this->_V_MAX) / (float)this->_V_MAX);

    if (true == this->_timer_check_inertia.check_passing(50)) {
        if (z > START_Z_UPPER) { /* If arm height is greater than certain value. */
            this->_z_count++;
        } else {
            this->_z_count -= this->_Z_COUNT_DECREASE_RATE;
        }
        this->_z_count = constrain(this->_z_count, this->_Z_COUNT_MIN, this->_Z_COUNT_MAX);

        if (abs(v) > this->_R_MOVING_LIMIT || abs(w) > this->_S_MOVING_LIMIT) {
            this->_speed_inertia += this->_SPEED_INERTIA_INCREASE_RATE;
        } else {
            /* If wheel is almost stopped, make it hard to move. */
            this->_speed_inertia -= this->_SPEED_INERTIA_DECREASE_RATE;
        }
        this->_speed_inertia = constrain(this->_speed_inertia * this->_SPEED_INERTIA_CONSTRAIN_MAX, this->_SPEED_INERTIA_CONSTRAIN_MIN, this->_SPEED_INERTIA_CONSTRAIN_MAX)
                             / this->_SPEED_INERTIA_CONSTRAIN_MAX;
    }
    v = v * this->_speed_inertia;
    w = w * this->_speed_inertia;

    if (this->_z_count < 1) {
        v                    = 0;
        w                    = 0;
        this->_speed_inertia = 0;
    }

    if (v < 0) {
        v = v / this->_V_DECREASE_FACTOR;
    }

    args->input.wheel_target_v = v;
    args->input.wheel_target_w = w;

    this->_R_target_pwm = -(args->input.wheel_target_v + args->input.wheel_target_w * this->_wheel_tread) / (this->_wheel_diameter * this->_PI) * this->_WHEEL_TO_PWM_FACTOR;
    this->_L_target_pwm = -(args->input.wheel_target_v - args->input.wheel_target_w * this->_wheel_tread) / (this->_wheel_diameter * this->_PI) * this->_WHEEL_TO_PWM_FACTOR;
}

void PostureManager_Wheel::_yaw_control(PostureManagerArguments *args)
{
    if (this->_flag_yaw_control == 1) {
        float now_theta = args->input.wheel_gy.yaw + this->_yaw_rotation * this->_ONE_REVOLUTION_DEG;

        this->_mode = COMMAND_MOVING;

        float s_error         = this->_target_theta - now_theta;
        this->_max_turn_speed = this->_YAW_CONTROL_MAX_TURN_SPEED;
        float w = constrain(s_error * this->_K_w * this->_W_CONSTRAIN_FACTOR, -this->_max_turn_speed * this->_W_CONSTRAIN_FACTOR, this->_max_turn_speed * this->_W_CONSTRAIN_FACTOR)
                / this->_W_CONSTRAIN_FACTOR;

        args->input.wheel_target_w = w;
    }
}

void PostureManager_Wheel::set_target_theta(float target_theta)
{
    this->_target_theta = target_theta;
}

float PostureManager_Wheel::get_target_theta()
{
    return this->_target_theta;
}

int PostureManager_Wheel::get_target_r()
{
    return this->_target_r;
}

int PostureManager_Wheel::get_target_s()
{
    return this->_target_s;
}

void PostureManager_Wheel::set_flag_yaw_control(bool flag_yaw_control)
{
    this->_flag_yaw_control = flag_yaw_control;
}

void PostureManager_Wheel::set_posture_address(PostureManagerArguments *args)
{
    this->_args = args;
}

bool PostureManager_Wheel::_end()
{
    return true;
}

} // namespace arm_unit
} // namespace maid_robot_system
