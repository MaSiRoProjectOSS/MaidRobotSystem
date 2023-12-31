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

bool ModelImplement::is_text()
{
    return false;
}

std::string ModelImplement::pop()
{
    return "";
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
