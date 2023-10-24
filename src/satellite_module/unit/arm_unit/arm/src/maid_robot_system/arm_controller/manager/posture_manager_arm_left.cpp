/**
 * @file posture_manager_arm_left.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/manager/posture_manager_arm_left.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

PostureManager_ArmLeft::PostureManager_ArmLeft()
{
}

PostureManager_ArmLeft::~PostureManager_ArmLeft()
{
}

bool PostureManager_ArmLeft::_begin()
{
    bool result = true;
    return result;
}

bool PostureManager_ArmLeft::_calculate()
{
    bool result = true;
    return result;
}

void PostureManager_ArmLeft::set_posture_address(PostureManagerArguments *args)
{
    this->_args = args;
}

bool PostureManager_ArmLeft::_end()
{
    return true;
}

} // namespace arm_unit
} // namespace maid_robot_system
