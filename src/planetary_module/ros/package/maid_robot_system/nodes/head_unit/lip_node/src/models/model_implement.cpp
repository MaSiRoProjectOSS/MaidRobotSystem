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
bool ModelImplement::calculate(int percent)
{
    static int per = -1;
    bool result    = false;
    if (per != percent) {
        per    = percent;
        result = true;
    }
    return result;
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
