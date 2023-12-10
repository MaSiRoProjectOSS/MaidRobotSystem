/**
 * @file model_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_NECK_NODE_MODEL_IMPLEMENT_HPP
#define MRS_NECK_NODE_MODEL_IMPLEMENT_HPP

namespace maid_robot_system
{
class ModelImplement {
public:
    // =============================
    // PUBLIC : Function
    // =============================
    bool calculate(int x, int y, int z, int w);

private:
    // =============================
    // PRIVATE : Function
    // =============================

private:
    // =============================
    // PRIVATE : Variable
    // =============================

public:
    // =============================
    // Constructor
    // =============================
    ModelImplement();
    ~ModelImplement();
};

} // namespace maid_robot_system

#endif
