/**
 * @file posture_manager_arm.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Manager posture of arm
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/manager/posture_manager_arm.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

PostureManager_Arm::PostureManager_Arm()
{
    this->left  = new PostureManager_ArmLeft();
    this->right = new PostureManager_ArmRight();
}
PostureManager_Arm::~PostureManager_Arm()
{
}

bool PostureManager_Arm::_begin()
{
    bool result = true;
    this->left->begin();
    this->right->begin();

    return result;
}

bool PostureManager_Arm::_calculate()
{
    bool result = true;

    this->left->set_posture_address(this->_args);
    this->left->calculate();

    this->right->set_posture_address(this->_args);
    this->right->calculate();

    return result;
}

void PostureManager_Arm::set_posture_address(PostureManagerArguments *args)
{
    this->_args = args;
}

bool PostureManager_Arm::_end()
{
    return true;
}

} // namespace arm_unit
} // namespace maid_robot_system
