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

#include "../../../../../maid_robot_system_interfaces/include/maid_robot_system_interfaces/angle_calculus.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace maid_robot_system
{
class ModelImplement {
public:
    // =============================
    // PUBLIC : Function
    // =============================
    void set_times(double value);
    void set_offset(double value);
    void set_position_rotation(const geometry_msgs::msg::PoseStamped &msg);
    bool calculate();

    double get_times();
    double get_offset();
    double get_value();

private:
    // =============================
    // PRIVATE : Function
    // =============================

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    double _offset = 200;
    double _times  = 300;

    Vector3 *_position    = new Vector3(0.0f, 0.0f, 0.0f);
    Quaternion *_rotation = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);

public:
    // =============================
    // Constructor
    // =============================
    ModelImplement();
    ~ModelImplement();
};

} // namespace maid_robot_system

#endif
