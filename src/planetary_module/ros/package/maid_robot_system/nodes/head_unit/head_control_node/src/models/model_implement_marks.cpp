/**
 * @file model_implement_marks.cpp
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
bool ModelImplement::set_value_ar(ModelStructure::INPUT_TYPE type, int id, double seconds)
{
    // TODO
    bool result = false;
    switch (type) {
        case ModelStructure::INPUT_TYPE::AR_LEFT:
            result = true;
            break;
        case ModelStructure::INPUT_TYPE::AR_RIGHT:
            result = true;
            break;
        default:
            break;
    }
#if DEBUG_MODEL_IMPLEMENT
    if (true == result) {
        printf("ModelImplement::set_value_ar\n");
    }
#endif
    return result;
}
} // namespace maid_robot_system
