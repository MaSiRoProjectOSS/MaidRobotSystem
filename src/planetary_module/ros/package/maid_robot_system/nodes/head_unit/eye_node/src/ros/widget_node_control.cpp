/**
 * @file widget_node_control.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-10-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "ros/widget_node.hpp"

namespace maid_robot_system
{
// =============================
// PUBLIC : Control
// =============================
void WidgetNode::effect_cornea_order()
{
    this->_widget->cornea_order();
}

void WidgetNode::emotion(MIENS value)
{
    this->_widget->emotion(value);
}
void WidgetNode::dimensions(float dimensions)
{
    this->_widget->dimensions(dimensions);
}

void WidgetNode::stare(float distance, float left_y, float left_z, float right_y, float right_z)
{
    this->_widget->stare(distance, left_y, left_z, right_y, right_z);
}

} // namespace maid_robot_system
