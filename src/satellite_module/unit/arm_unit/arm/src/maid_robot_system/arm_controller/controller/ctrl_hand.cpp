/**
 * @file ctrl_hand.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/controller/ctrl_hand.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

CtrlHand::CtrlHand()
{
}

void CtrlHand::test(int set)
{
    this->_servo.move_along_trajectory(map(set, this->_TEST_DEG_X_MIN, this->_TEST_DEG_X_MAX, this->_TEST_DEG_Y_MIN, this->_TEST_DEG_Y_MAX), _TEST_SPEED);
}

void CtrlHand::move_along_trajectory(float tar_deg, int tar_speed)
{
    this->_servo.move_along_trajectory(tar_deg, tar_speed);
}

bool CtrlHand::_begin()
{
    this->_servo.setup(this->_SERVO_PORT, this->_INITIAL_POSITION, this->_INITIAL_DIRECTION);

    return true;
}

bool CtrlHand::_end()
{
    return true;
}

bool CtrlHand::_calculate()
{
    return true;
}

} // namespace arm_unit
} // namespace maid_robot_system
