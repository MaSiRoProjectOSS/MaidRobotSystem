/**
 * @file posture_manager_leg.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief PostureManager_Arm
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/manager/posture_manager_leg.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

PostureManager_Leg::PostureManager_Leg()
{
    this->_center_pos = this->_default_standup_pos;
}
PostureManager_Leg::~PostureManager_Leg()
{
}

bool PostureManager_Leg::_begin()
{
    bool result = true;
    return result;
}

void PostureManager_Leg::_reset(PostureManagerArguments *args)
{
    if (args->input.leg_communication_ok == true) {
        if (args->input.leg_pos >= 0) {
            this->_center_pos = args->input.leg_pos;
            if (args->input.leg_pos > this->_LEG_STAND_LIMIT) {
                args->states.leg_mode = STAND;
            } else {
                args->states.leg_mode = SIT;
            }
        }
    }
}

bool PostureManager_Leg::_calculate()
{
    bool result = true;

    if (this->_args->input.leg_step_percentage == 0) {
        this->_args->output.leg_send_pos
                = this->_target_pos + this->_center_pos + this->_SIN_CYCLE_POS_FACTOR * this->_args->states.vital_timer.get_sin_cycle(this->_args->states.breath_speed);
    } else {
        this->_args->output.leg_send_pos = this->_target_pos + this->_center_pos;
        this->_args->states.vital_timer.update();
    }
    this->_args->output.leg_send_pos = constrain(this->_args->output.leg_send_pos, this->_LEG_SEND_POS_MIN, this->_LEG_SEND_POS_MAX);

    if (this->_args->states.leg_mode == INITIALIZE) {
        this->_args->output.leg_send_flag = false;
        this->_reset(this->_args);
    } else {
        this->_args->output.leg_send_flag = true;

        if (this->_args->states.leg_mode == STAND) {
            this->_center_pos   = this->_default_standup_pos;
            int speed           = map(this->_args->input.shoulder_left_push * this->_SHOULDER_LEFT_SPEED_FACTOR, 0, 100, this->_upper_speed, this->_down_speed);
            this->_target_speed = speed;

            if (this->_args->input.shoulder_left_push > 0.7) {
                this->_args->states.leg_mode = SITTING;
            }
        }
        if (this->_args->states.leg_mode == SITTING) {
            int speed           = map(this->_args->input.shoulder_left_push * 100.0, this->_SPEED_X_MIN, this->_SPEED_X_MAX, this->_upper_speed, this->_down_speed);
            this->_target_speed = speed;

            if (this->_args->input.leg_pos < this->_default_sit_pos + this->_SIT_POS_OFFSET) {
                this->_args->states.leg_mode = SIT;
            }
            if (this->_args->input.leg_pos > this->_default_standup_pos - this->_STAND_POS_OFFSET) {
                this->_args->states.leg_mode = STAND;
            }
        }

        if (this->_args->states.leg_mode == SIT) {
            this->_center_pos   = this->_default_sit_pos;
            this->_target_speed = this->_down_speed / this->_DOWN_TO_TARGET_SPEED_FACTOR;

            if (this->_args->states.sensor_arm_hand_z > this->_HAND_Z_RISE_KNEE_LIMIT) {
                this->_args->states.leg_mode = RISE_KNEE;
            }
        }
        if (this->_args->states.leg_mode == RISE_KNEE) {
            if (this->_args->input.leg_pos < this->_default_standup_pos - this->_DEFAULT_STANDUP_OFFSET) {
                this->_target_speed = this->_upper_speed;

            } else {
                this->_args->states.leg_mode = STAND;
            }
        }

        if (true == this->_timer_dt.check_passing(10)) {
            if (this->_target_speed > this->_now_speed) {
                if (this->_now_speed > 0) {
                    this->_now_speed += this->_ONE_STEP_SPEED_INCREMENT_FOR_POSITIVE;
                } else {
                    this->_now_speed += this->_ONE_STEP_SPEED_INCREMENT_FOR_NEGATIVE;
                }
            }
            if (this->_target_speed < this->_now_speed) {
                this->_now_speed -= this->_ONE_STEP_SPEED_DECREMENT_FOR_LARGE_SPEED;
            }
        }

        if (this->_now_speed < 0) {
            this->_center_pos = this->_default_sit_pos;
        } else {
            this->_center_pos = this->_default_standup_pos;
        }
        this->_args->output.leg_send_speed = abs(this->_now_speed);
    }
    return result;
}

void PostureManager_Leg::set_posture_address(PostureManagerArguments *args)
{
    this->_args = args;
}

bool PostureManager_Leg::_end()
{
    return true;
}

void PostureManager_Leg::set_target_pos(float target_pos)
{
    this->_target_pos = target_pos;
}

} // namespace arm_unit
} // namespace maid_robot_system
