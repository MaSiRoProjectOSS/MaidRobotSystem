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
void ModelImplement::set_value(double value)
{
    this->_value = value;
}
double ModelImplement::calculate()
{
    return (this->_value * this->_times) + this->_offset;
}

double ModelImplement::get_times()
{
    return this->_times;
}
double ModelImplement::get_offset()
{
    return this->_offset;
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
