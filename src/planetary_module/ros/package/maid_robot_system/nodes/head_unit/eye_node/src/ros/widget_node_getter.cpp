/**
 * @file widget_node_getter.cpp
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
// PUBLIC : Getter
// =============================
bool WidgetNode::is_start()
{
    this->_app->processEvents();
    return this->_widget->is_start();
}
bool WidgetNode::request_update()
{
    return this->_widget->request_update();
}
int WidgetNode::running_exec()
{
    return this->_app->exec();
}

std::string WidgetNode::output_message(bool verbose)
{
    if (nullptr != this->_widget) {
        return this->_widget->output_message(verbose);
    } else {
        return "The widget is not started.";
    }
}
std::string WidgetNode::get_setting_file()
{
    return this->_widget->param.setting_file;
}

int WidgetNode::get_brightness()
{
    return this->_widget->param.brightness;
}

int WidgetNode::get_eyelid_color_r()
{
    return this->_widget->param.eyelid_color.r;
}
int WidgetNode::get_eyelid_color_g()
{
    return this->_widget->param.eyelid_color.g;
}
int WidgetNode::get_eyelid_color_b()
{
    return this->_widget->param.eyelid_color.b;
}

int WidgetNode::get_ciliary_color_r()
{
    return this->_widget->param.left_eye.ciliary_color.r;
}
int WidgetNode::get_ciliary_color_g()
{
    return this->_widget->param.left_eye.ciliary_color.g;
}
int WidgetNode::get_ciliary_color_b()
{
    return this->_widget->param.left_eye.ciliary_color.b;
}

} // namespace maid_robot_system
