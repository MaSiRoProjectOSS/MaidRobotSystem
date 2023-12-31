/**
 * @file widget_node_setter.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "ros/widget_node.hpp"

#include <filesystem>

namespace maid_robot_system
{
// =============================
// PUBLIC : Setter
// =============================
bool WidgetNode::set_setting_file(std::string value)
{
    return this->_widget->set_setting_file(value);
}

bool WidgetNode::set_brightness(int value)
{
    if (true == this->_within_range(value, 0, 100)) {
        this->_widget->param.brightness = value;
        return this->_widget->reload_param();
    } else {
        return false;
    }
}

bool WidgetNode::set_eyelid_color_r(int value)
{
    if (true == this->_within_range(value)) {
        this->_widget->param.eyelid_color.r = value;
        return this->_widget->reload_param();
    } else {
        return false;
    }
}
bool WidgetNode::set_eyelid_color_g(int value)
{
    if (true == this->_within_range(value)) {
        this->_widget->param.eyelid_color.g = value;
        return this->_widget->reload_param();
    } else {
        return false;
    }
}
bool WidgetNode::set_eyelid_color_b(int value)
{
    if (true == this->_within_range(value)) {
        this->_widget->param.eyelid_color.b = value;
        return this->_widget->reload_param();
    } else {
        return false;
    }
}

bool WidgetNode::set_ciliary_color_r(int value)
{
    if (true == this->_within_range(value)) {
        this->_widget->param.left_eye.ciliary_color.r  = value;
        this->_widget->param.right_eye.ciliary_color.r = value;
        return this->_widget->reload_param();
    } else {
        return false;
    }
}
bool WidgetNode::set_ciliary_color_g(int value)
{
    if (true == this->_within_range(value)) {
        this->_widget->param.left_eye.ciliary_color.g  = value;
        this->_widget->param.right_eye.ciliary_color.g = value;
        return this->_widget->reload_param();
    } else {
        return false;
    }
}
bool WidgetNode::set_ciliary_color_b(int value)
{
    if (true == this->_within_range(value)) {
        this->_widget->param.left_eye.ciliary_color.b  = value;
        this->_widget->param.right_eye.ciliary_color.b = value;
        return this->_widget->reload_param();
    } else {
        return false;
    }
}

// =============================
// PRIVATE : Function
// =============================
bool WidgetNode::_within_range(int value, int min, int max)
{
    if (value > max) {
        return false;
    } else if (value < min) {
        return false;
    } else {
        return true;
    }
}

} // namespace maid_robot_system
