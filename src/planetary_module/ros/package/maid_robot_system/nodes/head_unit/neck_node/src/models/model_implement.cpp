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
bool ModelImplement::calculate(int x, int y, int z, int w)
{
    static int _x = -1;
    static int _y = -1;
    static int _z = -1;
    static int _w = -1;
    bool result   = false;
    if (_x != x) {
        _x     = x;
        result = true;
    }
    if (_y != y) {
        _y     = y;
        result = true;
    }
    if (_z != z) {
        _z     = z;
        result = true;
    }
    if (_w != w) {
        _w     = w;
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
