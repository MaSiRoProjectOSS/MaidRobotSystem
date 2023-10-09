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
bool ModelImplement::set_setting_file(std::string value)
{
    bool result = false;
    if (true == std::filesystem::is_regular_file(value)) {
        result = this->_parse_param(value);
        if (true == true) {
            this->_set_param();
        }
    }

    return result;
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

} // namespace maid_robot_system
