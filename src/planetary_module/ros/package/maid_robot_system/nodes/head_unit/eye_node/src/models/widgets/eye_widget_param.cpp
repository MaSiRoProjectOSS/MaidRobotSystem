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

#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace maid_robot_system
{
#define DEBUG_OUTPUT 1

std::string EyeWidget::_read_file(const std::string &path)
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

bool EyeWidget::set_setting_file(std::string json_file)
{
    bool result = false;
    try {
        if (true == std::filesystem::is_regular_file(json_file)) {
            nlohmann::json settings = nlohmann::json::parse(this->_read_file(json_file));
            this->param.path        = settings.value("path", this->param.path);
            this->param.name        = settings.value("name", this->param.name);

            if (true == settings.contains("position")) {
                if (true == settings["position"].contains("left")) {
                    this->param.left.width  = settings["position"]["left"].value("width", this->param.left.width);
                    this->param.left.height = settings["position"]["left"].value("height", this->param.left.height);
                }
                if (true == settings["position"].contains("right")) {
                    this->param.right.width  = settings["position"]["right"].value("width", this->param.right.width);
                    this->param.right.height = settings["position"]["right"].value("height", this->param.right.height);
                }
            }
            this->param.setting_file = json_file;
#if DEBUG_OUTPUT
            printf("%s\n", "=========================================================");
            printf("file: %s\n", this->param.setting_file.c_str());
#endif
            result = true;
        }
    } catch (...) {
        result = false;
    }

    if (true == result) {
        this->reload_param();
#if DEBUG_OUTPUT
        printf("%s%s\n", "path: ", this->param.path.c_str());
        printf("%s%s\n", "name: ", this->param.name.c_str());
        printf("%s\n", "position:");
        printf("%s\n", "  left:");
        printf("%s%d\n", "    width: ", this->param.left.width);
        printf("%s%d\n", "    height: ", this->param.left.height);
        printf("%s\n", "  right:");
        printf("%s%d\n", "    width: ", this->param.right.width);
        printf("%s%d\n", "    height: ", this->param.right.height);
        printf("%s\n", "display:");
        printf("%s%d\n", "  brightness: ", this->param.brightness);
        printf("%s\n", "  color:");
        printf("%s%d\n", "    r: ", this->param.color_r);
        printf("%s%d\n", "    g: ", this->param.color_g);
        printf("%s%d\n", "    b: ", this->param.color_b);
        printf("%s\n", "=========================================================");
#endif
    }

    return result;
}

} // namespace maid_robot_system
