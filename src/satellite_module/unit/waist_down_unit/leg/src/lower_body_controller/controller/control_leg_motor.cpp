/**
 * @file control_leg_motor.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief control the leg
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "lower_body_controller/controller/control_leg_motor.hpp"

#include "maid_robot_system/common/log_callback.hpp"

#include <Arduino.h>

namespace lower_body_unit
{
namespace controller
{

ControlLegMotor::ControlLegMotor(int pin_left,
                                 int pin_right,
                                 int pin_sensor,
                                 int pin_servo_plus,
                                 int pin_servo_dir,
                                 int pin_servo_enable,
                                 int pin_servo_error,
                                 int pwm_limit,
                                 int position_max,
                                 const int position_limit[2],
                                 char fw_rw_sensor,
                                 char fw_rw_motor)
{
    this->_leg_position_max      = position_max;
    this->_leg_position_limit[0] = position_limit[0];
    this->_leg_position_limit[1] = position_limit[1];

    this->_mt = new driver::DriverStepperMotor(pin_left, //
                                               pin_right,
                                               pin_sensor,
                                               pin_servo_plus,
                                               pin_servo_dir,
                                               pin_servo_enable,
                                               pin_servo_error,
                                               pwm_limit,
                                               position_max,
                                               position_limit,
                                               fw_rw_sensor,
                                               fw_rw_motor);
}

bool ControlLegMotor::_begin()
{
    bool result = true;
    /////////////////////////////
    this->_mt->setup();
    this->_mt->set_PID(70, 0, 0);
    this->_mt->target_rps  = 0;
    this->_mt->motor_power = 1;
    this->leg_target_pos   = 950;
    /////////////////////////////
    return result;
}

bool ControlLegMotor::_end()
{
    bool result = true;
    return result;
}

bool ControlLegMotor::_calculate()
{
    bool result = true;
    ///
    this->_mt->target_pos = map(this->leg_target_pos + this->_step_height, 0, 1000, 0, this->_leg_position_max);
    this->_mt->resident(this->is_enable());

    if (this->_mt->now_pos > 450) {
        this->center_pitch_pos = map(this->_mt->now_pos, 450, this->_leg_position_limit[1], 7220, 6955);
    } else if (this->_mt->now_pos > 360) {
        this->center_pitch_pos = map(this->_mt->now_pos, 360, 450, 6050, 7220);
    } else if (this->_mt->now_pos > 307) {
        this->center_pitch_pos = map(this->_mt->now_pos, 307, 360, 5172, 6050);
    } else {
        this->center_pitch_pos = map(this->_mt->now_pos, this->_leg_position_limit[0], 356, 5813, 5172);
    }

    return result;
}

bool ControlLegMotor::_is_error()
{
    return false;
}

void ControlLegMotor::set_data(float step_height)
{
    this->_step_height = step_height;
}

int ControlLegMotor::get_height_percentage()
{
    return this->_mt->now_percentage;
}

void ControlLegMotor::set_data(int flag_smart_move, int position, int speed)
{
    this->_mt->smart_move_flag  = flag_smart_move;
    this->_mt->leg_target_speed = speed;

    this->leg_target_pos = position;

    this->_heartbeat.update();
}

bool ControlLegMotor::is_alive()
{
    return !this->_heartbeat.check_time_over(this->TIMEOUT_MS);
}

} // namespace controller
} // namespace lower_body_unit
