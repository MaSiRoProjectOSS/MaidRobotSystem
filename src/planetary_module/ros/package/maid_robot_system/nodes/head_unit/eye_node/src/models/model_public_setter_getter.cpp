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
// =============================
// PUBLIC : Function
// =============================
void ModelImplement::set_msg_eye(int emotions, //
                                 int pupil_effect,
                                 float size,
                                 float distance,
                                 float left_x,
                                 float left_y,
                                 float right_x,
                                 float right_y)
{
}

std::string ModelImplement::get_lap_time()
{
    return "";
}

int ModelImplement::get_brightness()
{
    return this->_param.brightness;
}
int ModelImplement::get_color_r()
{
    return this->_param.color_r;
}
int ModelImplement::get_color_g()
{
    return this->_param.color_g;
}
int ModelImplement::get_color_b()
{
    return this->_param.color_b;
}

bool ModelImplement::set_brightness(int value)
{
    this->_param.brightness = value;
    return true;
}
bool ModelImplement::set_color_r(int value)
{
    this->_param.color_r = value;
    return true;
}
bool ModelImplement::set_color_g(int value)
{
    this->_param.color_g = value;
    return true;
}
bool ModelImplement::set_color_b(int value)
{
    this->_param.color_b = value;
    return true;
}

std::string ModelImplement::get_setting_file()
{
    return this->_param.setting_file;
}

bool ModelImplement::set_setting_file(std::string value)
{
    bool result = false;
    if (true == std::filesystem::is_regular_file(value)) {
        result = this->_set_param(value);
    }

    return result;
}

} // namespace maid_robot_system
