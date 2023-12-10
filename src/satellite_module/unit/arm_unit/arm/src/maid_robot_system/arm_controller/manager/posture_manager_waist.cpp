/**
 * @file posture_manager_waist.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Manage posture of waist.
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/manager/posture_manager_waist.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace arm_unit
{

PostureManager_Waist::PostureManager_Waist()
{
}
PostureManager_Waist::~PostureManager_Waist()
{
}

bool PostureManager_Waist::_begin()
{
    bool result = true;
    return result;
}

bool PostureManager_Waist::_calculate()
{
    /***/
    float link_neck_yaw  = (this->_args->output.neck_angle.yaw) / this->_NECK_TO_LINK_NECK_YAW_FACTOR;
    float link_neck_roll = (this->_args->output.neck_angle.roll) * this->_NECK_TO_LINK_NECK_ROLL_FACTOR;
    /***/

    bool result = true;

    float breath_delta_yaw
            = this->SIN_CYCLE_TO_BREATH_DELTA_YAW_FACTOR * this->_args->states.vital_timer.get_sin_cycle(this->_args->states.breath_speed * this->_BREATH_SPEED_FACTOR);
    float breath_delta_roll
            = this->SIN_CYCLE_TO_BREATH_DELTA_ROLL_FACTOR * this->_args->states.vital_timer.get_sin_cycle(this->_args->states.breath_speed * this->_BREATH_SPEED_FACTOR);

    float dt                = (micros() - this->_resident_time) / this->_SEC_TO_MICRO_SEC_FACTOR;
    float send_target_roll  = this->_args->states.waist_target.roll + breath_delta_roll + link_neck_roll; //+ walking_roll;
    float send_target_yaw   = this->_args->states.waist_target.yaw + breath_delta_yaw + link_neck_yaw;    //+ walking_yaw;
    float send_target_pitch = this->_args->states.waist_target.pitch;                                     //+ walking_pitch ;

    this->_now_pitch += this->_args->states.waist_speed_gain * (send_target_pitch - this->_now_pitch) * dt;
    this->_now_yaw += this->_args->states.waist_speed_gain * (send_target_yaw - this->_now_yaw) * dt;
    this->_now_roll += this->_args->states.waist_speed_gain * (send_target_roll - this->_now_roll) * dt;

    this->_now_roll = constrain(this->_now_roll, this->_limit_roll[0], this->_limit_roll[1]);
    this->_now_yaw  = constrain(this->_now_yaw, this->_limit_yaw[0], this->_limit_yaw[1]);

    this->_resident_time = micros();

    /***/
    this->_args->output.waist_angle.set(this->_now_pitch, this->_now_roll, this->_now_yaw);
    /***/

    return result;
}

void PostureManager_Waist::set_posture_address(PostureManagerArguments *args)
{
    this->_args = args;
}

bool PostureManager_Waist::_end()
{
    return true;
}

} // namespace arm_unit
} // namespace maid_robot_system
