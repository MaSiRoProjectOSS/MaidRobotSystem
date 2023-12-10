/**
 * @file speed_servo.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/controller/driver/speed_servo.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

SpeedServo::SpeedServo()
{
}

void SpeedServo::setup(int servo_port, int ini_pos, int direction)
{
    this->_direction = direction;
    this->_servo.attach(servo_port);
    this->_center_pos = ini_pos;
    this->servo_write_deg(0);

    this->_servo_send_time = micros();
}

void SpeedServo::detach()
{
    this->_servo.detach();
}

float SpeedServo::get_pos()
{
    return this->_now_pos;
}

int SpeedServo::move_along_trajectory(int tar_deg)
{
    return this->move_along_trajectory(tar_deg, this->_speed);
}

int SpeedServo::move_along_trajectory(float tar_deg, int tar_speed)
{
    this->_target_pos        = tar_deg * this->_deg_to_pwm;
    this->_speed             = tar_speed;
    unsigned long delta_time = micros() - this->_servo_send_time;

    if (delta_time > _MAX_DELTA_TIME) {
        delta_time = _SUB_MAX_DELTA_TIME;
    }
    float time_offset = delta_time / (float)_SUB_MAX_DELTA_TIME;
    if (tar_deg > this->get_pos()) {
        this->_true_tar_pos = this->_true_tar_pos + map(this->_speed, _SPEED_MIN, _SPEED_MAX, _POSITION_MIN, _POSITION_MAX) / _PERCENT_TO_RATE * time_offset;
    } else if (tar_deg < this->get_pos()) {
        this->_true_tar_pos = this->_true_tar_pos - map(this->_speed, _SPEED_MIN, _SPEED_MAX, _POSITION_MIN, _POSITION_MAX) / _PERCENT_TO_RATE * time_offset;
    }

    this->servo_write_deg(this->_true_tar_pos);
    this->_servo_send_time = micros();

    return this->_now_pos;
}

float SpeedServo::servo_write_deg(float deg)
{
    int scale      = _DIRECTION_SCALE * _direction;
    int deg_map    = (deg + this->_center_pos) * scale;
    int servo_send = map(deg_map, -_SERVO_SCALE_FACTOR * scale, _SERVO_SCALE_FACTOR * scale, _SERVO_SEND_MIN, _SERVO_SEND_MAX);

    _servo.writeMicroseconds(servo_send);

    return this->_now_pos = deg;
}

int SpeedServo::move_p(int tar_pos)
{
    this->_target_pos = tar_pos;

    return this->move_p();
}

int SpeedServo::move_p()
{
    this->get_pos();
    int p_speed = abs(this->_target_pos - this->_now_pos) * this->_p_gain;
    p_speed     = constrain(p_speed, _MOVE_P_SPEED_MIN, _MOVE_P_SPEED_MAX);

    return this->move_along_trajectory(this->_target_pos, p_speed);
}

} // namespace arm_unit
} // namespace maid_robot_system
