/**
 * @file model_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef SAMPLE_NODE_MODEL_IMPLEMENT_HPP
#define SAMPLE_NODE_MODEL_IMPLEMENT_HPP

#include <string>

namespace maid_robot_system
{
class ModelImplement {
public:
    // =============================
    // PUBLIC : Function
    // =============================
    bool is_text();
    std::string pop();

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
