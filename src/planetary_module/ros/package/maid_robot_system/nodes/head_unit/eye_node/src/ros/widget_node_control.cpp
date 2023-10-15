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

void WidgetNode::set_msg_eye(float size, float distance, float left_x, float left_y, float right_x, float right_y)
{
    this->_widget->stare(size, distance, left_x, left_y, right_x, right_y);
}

} // namespace maid_robot_system
