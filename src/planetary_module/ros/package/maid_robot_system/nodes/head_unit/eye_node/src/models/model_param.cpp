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

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace maid_robot_system
{
#define DEBUG_OUTPUT 1

std::string ModelImplement::_read_file(const std::string &path)
{
    auto ss = std::ostringstream{};
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        std::cerr << "Could not open the file : '" << path << "'" << std::endl;
        return "{}";
    }
    ss << input_file.rdbuf();
    return ss.str();
}

bool ModelImplement::_parse_param(std::string json_file)
{
    bool result = false;
    try {
        nlohmann::json settings   = nlohmann::json::parse(this->_read_file(json_file));
        this->_widget->param.path = settings.value("path", this->_widget->param.path);
        this->_widget->param.name = settings.value("name", this->_widget->param.name);

        if (true == settings.contains("position")) {
            if (true == settings["position"].contains("left")) {
                this->_widget->param.left.width  = settings["position"]["left"].value("width", this->_widget->param.left.width);
                this->_widget->param.left.height = settings["position"]["left"].value("height", this->_widget->param.left.height);
            }
            if (true == settings["position"].contains("right")) {
                this->_widget->param.right.width  = settings["position"]["right"].value("width", this->_widget->param.right.width);
                this->_widget->param.right.height = settings["position"]["right"].value("height", this->_widget->param.right.height);
            }
        }
        this->_widget->param.setting_file = json_file;
#if DEBUG_OUTPUT
        printf("%s\n", "=========================================================");
        printf("file: %s\n", this->_widget->param.setting_file.c_str());
#endif
        result = true;
    } catch (...) {
        result = false;
    }

#if DEBUG_OUTPUT
    if (true == result) {
        printf("%s%s\n", "path: ", this->_widget->param.path.c_str());
        printf("%s%s\n", "name: ", this->_widget->param.name.c_str());
        printf("%s\n", "position:");
        printf("%s\n", "  left:");
        printf("%s%d\n", "    width: ", this->_widget->param.left.width);
        printf("%s%d\n", "    height: ", this->_widget->param.left.height);
        printf("%s\n", "  right:");
        printf("%s%d\n", "    width: ", this->_widget->param.right.width);
        printf("%s%d\n", "    height: ", this->_widget->param.right.height);
        printf("%s\n", "display:");
        printf("%s%d\n", "  brightness: ", this->_widget->param.brightness);
        printf("%s\n", "  color:");
        printf("%s%d\n", "    r: ", this->_widget->param.color_r);
        printf("%s%d\n", "    g: ", this->_widget->param.color_g);
        printf("%s%d\n", "    b: ", this->_widget->param.color_b);
    }
    printf("%s\n", "=========================================================");
#endif

    return result;
}

} // namespace maid_robot_system
