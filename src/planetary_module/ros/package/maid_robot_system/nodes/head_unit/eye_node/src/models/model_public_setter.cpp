/**
 * @file model_implement.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "models/model_implement.hpp"
#include "models/widgets/eye_widget.hpp"

#include <filesystem>

namespace maid_robot_system
{
void ModelImplement::effect_pupil_order()
{
    if (nullptr != this->_widget) {
        this->_widget->pupil_order();
    }
}

std::string ModelImplement::output_message()
{
    if (nullptr != this->_widget) {
        return this->_widget->output_message();
    } else {
        return "";
    }
}

void ModelImplement::emotion(MIENS value)
{
}

void ModelImplement::set_msg_eye(int emotions, int pupil_effect, float size, float distance, float left_x, float left_y, float right_x, float right_y)
{
    if (nullptr != this->_widget) {
        this->_widget->cmd_eye_input(emotions, pupil_effect, size, distance, left_x, left_y, right_x, right_y);
    }
}

bool ModelImplement::set_setting_file(std::string value)
{
    return this->_widget->set_setting_file(value);
}

bool ModelImplement::set_brightness(int value)
{
    this->_widget->param.brightness = value;
    return this->_widget->reload_param();
}
bool ModelImplement::set_color_r(int value)
{
    this->_widget->param.color_r = value;
    return this->_widget->reload_param();
}
bool ModelImplement::set_color_g(int value)
{
    this->_widget->param.color_g = value;
    return this->_widget->reload_param();
}
bool ModelImplement::set_color_b(int value)
{
    this->_widget->param.color_b = value;
    return this->_widget->reload_param();
}

} // namespace maid_robot_system
