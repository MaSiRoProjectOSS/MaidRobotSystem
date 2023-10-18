/**
 * @file eye_widget_private.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "widgets/eye_widget.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace maid_robot_system
{
std::string EyeWidget::_read_file(const std::string &path)
{
    auto ss = std::ostringstream{};
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        char message[255];
        sprintf(message, "Could not open the file : '%s'", path.c_str());
        throw new std::logic_error(message);
        return "{}";
    }
    ss << input_file.rdbuf();
    return ss.str();
}
} // namespace maid_robot_system
