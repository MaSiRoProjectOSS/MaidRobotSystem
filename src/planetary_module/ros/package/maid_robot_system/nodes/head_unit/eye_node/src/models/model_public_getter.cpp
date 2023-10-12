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
std::string ModelImplement::get_setting_file()
{
    return this->_widget->param.setting_file;
}
int ModelImplement::get_brightness()
{
    return this->_widget->param.brightness;
}
int ModelImplement::get_color_r()
{
    return this->_widget->param.color.r;
}
int ModelImplement::get_color_g()
{
    return this->_widget->param.color.g;
}
int ModelImplement::get_color_b()
{
    return this->_widget->param.color.b;
}

} // namespace maid_robot_system
