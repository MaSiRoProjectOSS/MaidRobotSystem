/**
 * @file widget_node_setter.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "models/widgets/eye_widget.hpp"
#include "ros/widget_node.hpp"

#include <filesystem>

namespace maid_robot_system
{
void WidgetNode::effect_cornea_order()
{
    if (nullptr != this->_widget) {
        this->_widget->cornea_order();
    }
}

std::string WidgetNode::output_message()
{
    if (nullptr != this->_widget) {
        return this->_widget->output_message();
    } else {
        return "The widget is not started.";
    }
}

void WidgetNode::emotion(MIENS value)
{
    if (nullptr != this->_widget) {
        this->_widget->emotion(value);
    }
}

void WidgetNode::set_msg_eye(float size, float distance, float left_x, float left_y, float right_x, float right_y)
{
    if (nullptr != this->_widget) {
        this->_widget->stare(size, distance, left_x, left_y, right_x, right_y);
    }
}

bool WidgetNode::set_setting_file(std::string value)
{
    return this->_widget->set_setting_file(value);
}

bool WidgetNode::set_brightness(int value)
{
    this->_widget->param.brightness = value;
    return this->_widget->reload_param();
}

bool WidgetNode::set_eyelid_color_r(int value)
{
    this->_widget->param.eyelid_color.r = value;
    return this->_widget->reload_param();
}
bool WidgetNode::set_eyelid_color_g(int value)
{
    this->_widget->param.eyelid_color.g = value;
    return this->_widget->reload_param();
}
bool WidgetNode::set_eyelid_color_b(int value)
{
    this->_widget->param.eyelid_color.b = value;
    return this->_widget->reload_param();
}

bool WidgetNode::set_ciliary_color_r(int value)
{
    this->_widget->param.ciliary_color.r = value;
    return this->_widget->reload_param();
}
bool WidgetNode::set_ciliary_color_g(int value)
{
    this->_widget->param.ciliary_color.g = value;
    return this->_widget->reload_param();
}
bool WidgetNode::set_ciliary_color_b(int value)
{
    this->_widget->param.ciliary_color.b = value;
    return this->_widget->reload_param();
}

} // namespace maid_robot_system
