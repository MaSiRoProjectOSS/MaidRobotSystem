/**
 * @file model_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef TEMPLATE_NODE_MODEL_IMPLEMENT_HPP
#define TEMPLATE_NODE_MODEL_IMPLEMENT_HPP

namespace maid_robot_system
{

class ModelImplement {
public:
    // =============================
    // PUBLIC : Function
    // =============================
    void set_times(double value);
    void set_offset(double value);
    double calculate(double value);

    double get_times();
    double get_offset();

private:
    // =============================
    // PRIVATE : Function
    // =============================

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    double _offset = 0;
    double _times  = 1;

public:
    // =============================
    // Constructor
    // =============================
    ModelImplement();
    ~ModelImplement();
};

} // namespace maid_robot_system

#endif
