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
    this->_position->x = (float)msg.pose.position.x;
    this->_position->y = (float)msg.pose.position.y;
    this->_position->z = (float)msg.pose.position.z;

    this->_rotation->x = (float)msg.pose.orientation.x;
    this->_rotation->y = (float)msg.pose.orientation.y;
    this->_rotation->z = (float)msg.pose.orientation.z;
    this->_rotation->w = (float)msg.pose.orientation.w;
}

bool ModelImplement::calculate()
{
    std::cout << "x: " << this->_position->x << std::endl;

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
// Constructor
// =============================
ModelImplement::ModelImplement()
{
}

ModelImplement ::~ModelImplement()
{
}

} // namespace maid_robot_system
