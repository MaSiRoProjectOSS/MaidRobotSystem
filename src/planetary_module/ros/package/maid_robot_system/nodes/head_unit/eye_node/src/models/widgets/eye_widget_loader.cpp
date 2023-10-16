/**
 * @file eye_widget_loader.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace maid_robot_system
{
#ifndef DEBUG_OUTPUT_PARAM_LV
#define DEBUG_OUTPUT_PARAM_LV 0
#endif

bool EyeWidget::load()
{
    bool result = false;
    this->eyelid->load(this->param);
    this->eyeball->load(this->param);
    return result;
}

bool EyeWidget::reload_param()
{
    bool result = false;
    if (true == this->_flag_initialized) {
        StVector eye_size;
        this->param.screen_size.width  = (double)(this->param.view_size.width / this->param.screen_resolution);
        this->param.screen_size.height = (double)(this->param.view_size.height / this->param.screen_resolution);
        eye_size.x                     = this->param.screen_size.width * 0.5;
        eye_size.y                     = this->param.screen_size.height;

        // calculate
        this->param.left_eye.eyelid.width   = (int)std::max(1.0, eye_size.x * this->param.left_eye.eyelid_scale.x);
        this->param.left_eye.eyelid.height  = (int)std::max(1.0, eye_size.y * this->param.left_eye.eyelid_scale.y);
        this->param.right_eye.eyelid.width  = (int)std::max(1.0, eye_size.x * this->param.right_eye.eyelid_scale.x);
        this->param.right_eye.eyelid.height = (int)std::max(1.0, eye_size.y * this->param.right_eye.eyelid_scale.y);

        this->param.left_eye.eyeball.width   = (int)std::max(1.0, eye_size.x * this->param.left_eye.eyeball_scale.x);
        this->param.left_eye.eyeball.height  = (int)std::max(1.0, eye_size.y * this->param.left_eye.eyeball_scale.y);
        this->param.right_eye.eyeball.width  = (int)std::max(1.0, eye_size.x * this->param.right_eye.eyeball_scale.x);
        this->param.right_eye.eyeball.height = (int)std::max(1.0, eye_size.y * this->param.right_eye.eyeball_scale.y);

        // set
        this->eyelid->set_param(this->param);
        this->param.left_eye.eyeball_center.x  = this->eyelid->left_eye.pos_center.x + this->param.left_eye.eyeball.x;
        this->param.left_eye.eyeball_center.y  = this->eyelid->left_eye.pos_center.y + this->param.left_eye.eyeball.y;
        this->param.right_eye.eyeball_center.x = this->eyelid->right_eye.pos_center.x + this->param.right_eye.eyeball.x;
        this->param.right_eye.eyeball_center.y = this->eyelid->right_eye.pos_center.y + this->param.right_eye.eyeball.y;
        this->eyeball->set_param(this->param);

#if DEBUG_OUTPUT_PARAM_LV > 0
        printf("=========================================================\n");
        printf("file: %s\n", this->param.setting_file.c_str());
        printf("path: %s\n", this->param.path.c_str());
        printf("name: %s\n", this->param.name.c_str());

        printf("eyelid:\n");
#if DEBUG_OUTPUT_PARAM_LV > 2
        printf("  color:\n");
        printf("    r: %d\n", this->param.eyelid_color.r);
        printf("    g: %d\n", this->param.eyelid_color.g);
        printf("    b: %d\n", this->param.eyelid_color.b);
#endif
        printf("  left_eye:\n");
        printf("    x: %d\n", this->param.left_eye.eyelid.x);
        printf("    y: %d\n", this->param.left_eye.eyelid.y);
        printf("    width: %d\n", this->param.left_eye.eyelid.width);
        printf("    height: %d\n", this->param.left_eye.eyelid.height);
        printf("    angle: %3.1f\n", this->param.left_eye.eyelid.angle);
        printf("  right_eye:\n");
        printf("    x: %d\n", this->param.right_eye.eyelid.x);
        printf("    y: %d\n", this->param.right_eye.eyelid.y);
        printf("    width: %d\n", this->param.right_eye.eyelid.width);
        printf("    height: %d\n", this->param.right_eye.eyelid.height);
        printf("    angle: %3.1f\n", this->param.right_eye.eyelid.angle);

        printf("eyeball:\n");
        printf("  left_eye:\n");
        printf("    x: %d\n", this->param.left_eye.eyeball.x);
        printf("    y: %d\n", this->param.left_eye.eyeball.y);
        printf("    width: %d\n", this->param.left_eye.eyeball.width);
        printf("    height: %d\n", this->param.left_eye.eyeball.height);
        printf("    angle: %3.1f\n", this->param.left_eye.eyeball.angle);
        printf("    center:\n");
        printf("      x: %3.1f\n", this->param.left_eye.eyeball_center.x);
        printf("      y: %3.1f\n", this->param.left_eye.eyeball_center.y);
        printf("  right_eye:\n");
        printf("    x: %d\n", this->param.right_eye.eyeball.x);
        printf("    y: %d\n", this->param.right_eye.eyeball.y);
        printf("    width: %d\n", this->param.right_eye.eyeball.width);
        printf("    height: %d\n", this->param.right_eye.eyeball.height);
        printf("    angle: %3.1f\n", this->param.right_eye.eyeball.angle);
        printf("    center:\n");
        printf("      x: %3.1f\n", this->param.right_eye.eyeball_center.x);
        printf("      y: %3.1f\n", this->param.right_eye.eyeball_center.y);

#if DEBUG_OUTPUT_PARAM_LV > 1
        printf("cornea:\n");
        printf("  left_eye:\n");
        printf("    outside:\n");
        printf("      enable: %s\n", this->param.left_eye.cornea_outside.enable ? "True" : "False");
        printf("      speed: %3.1f\n", this->param.left_eye.cornea_outside.speed);
        printf("      scale:\n");
        printf("        x: %3.1f\n", this->param.left_eye.cornea_outside.scale.x);
        printf("        y: %3.1f\n", this->param.left_eye.cornea_outside.scale.y);
        printf("      alpha: %d\n", this->param.left_eye.cornea_outside.alpha);
        printf("    inside:\n");
        printf("      enable: %s\n", this->param.left_eye.cornea_inside.enable ? "True" : "False");
        printf("      speed: %3.1f\n", this->param.left_eye.cornea_inside.speed);
        printf("      scale:\n");
        printf("        x: %3.1f\n", this->param.left_eye.cornea_inside.scale.x);
        printf("        y: %3.1f\n", this->param.left_eye.cornea_inside.scale.y);
        printf("      alpha: %d\n", this->param.left_eye.cornea_inside.alpha);
        printf("  right_eye:\n");
        printf("    outside:\n");
        printf("      enable: %s\n", this->param.right_eye.cornea_outside.enable ? "True" : "False");
        printf("      speed: %3.1f\n", this->param.right_eye.cornea_outside.speed);
        printf("      scale:\n");
        printf("        x: %3.1f\n", this->param.right_eye.cornea_outside.scale.x);
        printf("        y: %3.1f\n", this->param.right_eye.cornea_outside.scale.y);
        printf("      alpha: %d\n", this->param.right_eye.cornea_outside.alpha);
        printf("    inside:\n");
        printf("      enable: %s\n", this->param.right_eye.cornea_inside.enable ? "True" : "False");
        printf("      speed: %3.1f\n", this->param.right_eye.cornea_inside.speed);
        printf("      scale:\n");
        printf("        x: %3.1f\n", this->param.right_eye.cornea_inside.scale.x);
        printf("        y: %3.1f\n", this->param.right_eye.cornea_inside.scale.y);
        printf("      alpha: %d\n", this->param.right_eye.cornea_inside.alpha);
#endif

#if DEBUG_OUTPUT_PARAM_LV > 2
        printf("ciliary:\n");
        printf("  color:\n");
        printf("    r: %d\n", this->param.ciliary_color.r);
        printf("    g: %d\n", this->param.ciliary_color.g);
        printf("    b: %d\n", this->param.ciliary_color.b);
#endif

#if DEBUG_OUTPUT_PARAM_LV > 1
        printf("eye_blink:\n");
        printf("  quickly: %3.1f\n", this->param.blink_time_quickly);
        printf("  min: %3.1f\n", this->param.blink_time_min);
        printf("  max: %3.1f\n", this->param.blink_time_max);
        printf("  limit: %3.1f\n", this->param.blink_time_limit);
        printf("  offset: %3.1f\n", this->param.blink_time_offset);
#endif
        printf("display:\n");
#if DEBUG_OUTPUT_PARAM_LV > 2
        printf("  brightness: %d\n", this->param.brightness);
#endif
        printf("  width: %d\n", this->param.screen_size.width);
        printf("  height: %d\n", this->param.screen_size.height);
        printf("  resolution: %3.3f\n", (1.0 / this->param.screen_resolution));
#if DEBUG_OUTPUT_PARAM_LV > 1
        printf("image_summary:\n");
        printf("  size:\n");
        printf("    left:\n");
        printf("      eyelid:\n");
        if (0 < this->param.left_eye.image.eyelid.size()) {
            for (size_t i = 0; i < this->param.left_eye.image.eyelid.size(); ++i) {
                printf("        %s: %d\n", this->param.left_eye.image.eyelid[i].name.c_str(), (int)this->param.left_eye.image.eyelid[i].files.size());
            }
        } else {
            printf("        none: %d\n", (int)this->param.left_eye.image.eyelid.size());
        }
        printf("      eyeball: %d\n", (int)this->param.left_eye.image.eyeball.size());
        printf("      cornea_outside: %d\n", (int)this->param.left_eye.image.cornea_outside.size());
        printf("      cornea_inside: %d\n", (int)this->param.left_eye.image.cornea_inside.size());
        printf("    right:\n");
        printf("      eyelid:\n");
        if (0 < this->param.right_eye.image.eyelid.size()) {
            for (size_t i = 0; i < this->param.right_eye.image.eyelid.size(); ++i) {
                printf("        %s: %d\n", this->param.right_eye.image.eyelid[i].name.c_str(), (int)this->param.right_eye.image.eyelid[i].files.size());
            }
        } else {
            printf("        none: %d\n", (int)this->param.right_eye.image.eyelid.size());
        }
        printf("      eyeball: %d\n", (int)this->param.right_eye.image.eyeball.size());
        printf("      cornea_outside: %d\n", (int)this->param.right_eye.image.cornea_outside.size());
        printf("      cornea_inside: %d\n", (int)this->param.right_eye.image.cornea_inside.size());
#endif

#if DEBUG_OUTPUT_PARAM_LV > 4
        printf("  size:\n");
        printf("    left:\n");
        printf("      eyelid:\n");
        if (0 < this->param.left_eye.image.eyelid.size()) {
            for (size_t i = 0; i < this->param.left_eye.image.eyelid.size(); ++i) {
                printf("        %s:\n", this->param.left_eye.image.eyelid[i].name.c_str());
                for (size_t j = 0; j < this->param.left_eye.image.eyelid[i].files.size(); ++j) {
                    printf("          %s\n", this->param.left_eye.image.eyelid[i].files[j].c_str());
                }
            }
        } else {
            printf("        --\n");
        }
        printf("      eyeball:\n");
        if (0 < this->param.left_eye.image.eyeball.size()) {
            for (size_t i = 0; i < this->param.left_eye.image.eyeball.size(); ++i) {
                printf("        %s:\n", this->param.left_eye.image.eyeball[i].name.c_str());
                for (size_t j = 0; j < this->param.left_eye.image.eyeball[i].files.size(); ++j) {
                    printf("          %s\n", this->param.left_eye.image.eyeball[i].files[j].c_str());
                }
            }
        } else {
            printf("        --\n");
        }
        printf("      cornea_outside:\n");
        if (0 < this->param.left_eye.image.cornea_outside.size()) {
            for (size_t i = 0; i < this->param.left_eye.image.cornea_outside.size(); ++i) {
                printf("        %s:\n", this->param.left_eye.image.cornea_outside[i].name.c_str());
                for (size_t j = 0; j < this->param.left_eye.image.cornea_outside[i].files.size(); ++j) {
                    printf("          %s\n", this->param.left_eye.image.cornea_outside[i].files[j].c_str());
                }
            }
        } else {
            printf("        --\n");
        }
        printf("      cornea_inside:\n");
        if (0 < this->param.left_eye.image.cornea_inside.size()) {
            for (size_t i = 0; i < this->param.left_eye.image.cornea_inside.size(); ++i) {
                printf("        %s:\n", this->param.left_eye.image.cornea_inside[i].name.c_str());
                for (size_t j = 0; j < this->param.left_eye.image.cornea_inside[i].files.size(); ++j) {
                    printf("          %s\n", this->param.left_eye.image.cornea_inside[i].files[j].c_str());
                }
            }
        } else {
            printf("        --\n");
        }
        printf("    right:\n");
        printf("      eyelid:\n");
        if (0 < this->param.right_eye.image.eyelid.size()) {
            for (size_t i = 0; i < this->param.right_eye.image.eyelid.size(); ++i) {
                printf("        %s:\n", this->param.right_eye.image.eyelid[i].name.c_str());
                for (size_t j = 0; j < this->param.right_eye.image.eyelid[i].files.size(); ++j) {
                    printf("          %s\n", this->param.right_eye.image.eyelid[i].files[j].c_str());
                }
            }
        } else {
            printf("        --\n");
        }
        printf("      eyeball:\n");
        if (0 < this->param.right_eye.image.eyeball.size()) {
            for (size_t i = 0; i < this->param.right_eye.image.eyeball.size(); ++i) {
                printf("        %s:\n", this->param.right_eye.image.eyeball[i].name.c_str());
                for (size_t j = 0; j < this->param.right_eye.image.eyeball[i].files.size(); ++j) {
                    printf("          %s\n", this->param.right_eye.image.eyeball[i].files[j].c_str());
                }
            }
        } else {
            printf("        --\n");
        }
        printf("      cornea_outside:\n");
        if (0 < this->param.right_eye.image.cornea_outside.size()) {
            for (size_t i = 0; i < this->param.right_eye.image.cornea_outside.size(); ++i) {
                printf("        %s:\n", this->param.right_eye.image.cornea_outside[i].name.c_str());
                for (size_t j = 0; j < this->param.right_eye.image.cornea_outside[i].files.size(); ++j) {
                    printf("          %s\n", this->param.right_eye.image.cornea_outside[i].files[j].c_str());
                }
            }
        } else {
            printf("        --\n");
        }
        printf("      cornea_inside:\n");
        if (0 < this->param.right_eye.image.cornea_inside.size()) {
            for (size_t i = 0; i < this->param.right_eye.image.cornea_inside.size(); ++i) {
                printf("        %s:\n", this->param.right_eye.image.cornea_inside[i].name.c_str());
                for (size_t j = 0; j < this->param.right_eye.image.cornea_inside[i].files.size(); ++j) {
                    printf("          %s\n", this->param.right_eye.image.cornea_inside[i].files[j].c_str());
                }
            }
        } else {
            printf("        --\n");
        }
#endif
        printf("=========================================================\n");
#endif
        result = true;
    }
    return result;
}

bool EyeWidget::set_setting_file(std::string json_file)
{
    bool result      = false;
    std::string step = "";
    try {
        step = "Verify that the file exists";
        if (true == std::filesystem::exists(json_file)) {
            nlohmann::json settings = nlohmann::json::parse(this->_read_file(json_file));
            if (true == settings.contains("path")) {
                step             = "Get value / path";
                this->param.path = settings.value("path", this->param.path);
            }
            if (true == settings.contains("name")) {
                step             = "Get value / name";
                this->param.name = settings.value("name", this->param.name);
            }
            if (true == settings.contains("display")) {
                step                          = "Get value / display.resolution";
                double resolution             = settings["display"].value("resolution", this->param.screen_resolution);
                this->param.screen_resolution = 1.0 / std::max(0.001, resolution);
            }

            if (true == settings.contains("rectangle")) {
                ////////// left
                if (true == settings["rectangle"].contains("left")) {
                    if (true == settings["rectangle"]["left"].contains("eyelid")) {
                        if (true == settings["rectangle"]["left"]["eyelid"].contains("size_rate")) {
                            step                                = "Get value / left.eyelid.size_rate";
                            this->param.left_eye.eyelid_scale.x = settings["rectangle"]["left"]["eyelid"]["size_rate"].value("width", this->param.left_eye.eyelid_scale.x);
                            this->param.left_eye.eyelid_scale.y = settings["rectangle"]["left"]["eyelid"]["size_rate"].value("height", this->param.left_eye.eyelid_scale.y);
                        }
                        if (true == settings["rectangle"]["left"]["eyelid"].contains("offset")) {
                            step                              = "Get value / left.eyelid.offset";
                            this->param.left_eye.eyelid.x     = settings["rectangle"]["left"]["eyelid"]["offset"].value("y", this->param.left_eye.eyelid.x);
                            this->param.left_eye.eyelid.y     = -settings["rectangle"]["left"]["eyelid"]["offset"].value("z", this->param.left_eye.eyelid.y);
                            this->param.left_eye.eyelid.angle = -settings["rectangle"]["left"]["eyelid"]["offset"].value("angle", this->param.left_eye.eyelid.angle);
                        }
                    }
                    if (true == settings["rectangle"]["left"].contains("eyeball")) {
                        if (true == settings["rectangle"]["left"]["eyeball"].contains("size_rate")) {
                            step                                 = "Get value / left.eyeball.size_rate";
                            this->param.left_eye.eyeball_scale.x = settings["rectangle"]["left"]["eyeball"]["size_rate"].value("width", this->param.left_eye.eyeball_scale.x);
                            this->param.left_eye.eyeball_scale.y = settings["rectangle"]["left"]["eyeball"]["size_rate"].value("height", this->param.left_eye.eyeball_scale.y);
                        }
                        if (true == settings["rectangle"]["left"]["eyeball"].contains("offset")) {
                            step                               = "Get value / left.eyeball.offset";
                            this->param.left_eye.eyeball.x     = settings["rectangle"]["left"]["eyeball"]["offset"].value("y", this->param.left_eye.eyeball.x);
                            this->param.left_eye.eyeball.y     = -settings["rectangle"]["left"]["eyeball"]["offset"].value("z", this->param.left_eye.eyeball.y);
                            this->param.left_eye.eyeball.angle = -settings["rectangle"]["left"]["eyeball"]["offset"].value("angle", this->param.left_eye.eyeball.angle);
                        }
                    }
                    if (true == settings["rectangle"]["left"].contains("cornea")) {
                        if (true == settings["rectangle"]["left"]["cornea"].contains("outside")) {
                            step                                       = "Get value / left.cornea.outside";
                            this->param.left_eye.cornea_outside.enable = //
                                    settings["rectangle"]["left"]["cornea"]["outside"].value("enable", this->param.left_eye.cornea_outside.enable);
                            this->param.left_eye.cornea_outside.speed = //
                                    settings["rectangle"]["left"]["cornea"]["outside"].value("speed", this->param.left_eye.cornea_outside.speed);
                            this->param.left_eye.cornea_outside.alpha = //
                                    settings["rectangle"]["left"]["cornea"]["outside"].value("alpha", this->param.left_eye.cornea_outside.alpha);
                            if (true == settings["rectangle"]["left"]["cornea"]["outside"].contains("size_rate")) {
                                this->param.left_eye.cornea_outside.scale.x = //
                                        settings["rectangle"]["left"]["cornea"]["outside"]["size_rate"].value("width", this->param.left_eye.cornea_outside.scale.x);
                                this->param.left_eye.cornea_outside.scale.y = //
                                        settings["rectangle"]["left"]["cornea"]["outside"]["size_rate"].value("height", this->param.left_eye.cornea_outside.scale.y);
                            }
                        }
                    }
                    if (true == settings["rectangle"]["left"].contains("cornea")) {
                        if (true == settings["rectangle"]["left"]["cornea"].contains("inside")) {
                            step                                      = "Get value / left.cornea.inside";
                            this->param.left_eye.cornea_inside.enable = //
                                    settings["rectangle"]["left"]["cornea"]["inside"].value("enable", this->param.left_eye.cornea_inside.enable);
                            this->param.left_eye.cornea_inside.speed = //
                                    settings["rectangle"]["left"]["cornea"]["inside"].value("speed", this->param.left_eye.cornea_inside.speed);
                            this->param.left_eye.cornea_inside.alpha = //
                                    settings["rectangle"]["left"]["cornea"]["inside"].value("alpha", this->param.left_eye.cornea_inside.alpha);
                            if (true == settings["rectangle"]["left"]["cornea"]["inside"].contains("size_rate")) {
                                this->param.left_eye.cornea_inside.scale.x = //
                                        settings["rectangle"]["left"]["cornea"]["inside"]["size_rate"].value("width", this->param.left_eye.cornea_inside.scale.x);
                                this->param.left_eye.cornea_inside.scale.y = //
                                        settings["rectangle"]["left"]["cornea"]["inside"]["size_rate"].value("height", this->param.left_eye.cornea_inside.scale.y);
                            }
                        }
                    }
                }
                ////////// right
                if (true == settings["rectangle"].contains("right")) {
                    if (true == settings["rectangle"]["right"].contains("eyelid")) {
                        if (true == settings["rectangle"]["right"]["eyelid"].contains("size_rate")) {
                            step                                 = "Get value / right.eyelid.size_rate";
                            this->param.right_eye.eyelid_scale.x = settings["rectangle"]["right"]["eyelid"]["size_rate"].value("width", this->param.right_eye.eyelid_scale.x);
                            this->param.right_eye.eyelid_scale.y = settings["rectangle"]["right"]["eyelid"]["size_rate"].value("height", this->param.right_eye.eyelid_scale.y);
                        }
                        if (true == settings["rectangle"]["right"]["eyelid"].contains("offset")) {
                            step                               = "Get value / right.eyelid.offset";
                            this->param.right_eye.eyelid.x     = settings["rectangle"]["right"]["eyelid"]["offset"].value("y", this->param.right_eye.eyelid.x);
                            this->param.right_eye.eyelid.y     = -settings["rectangle"]["right"]["eyelid"]["offset"].value("z", this->param.right_eye.eyelid.y);
                            this->param.right_eye.eyelid.angle = -settings["rectangle"]["right"]["eyelid"]["offset"].value("angle", this->param.right_eye.eyelid.angle);
                        }
                    }
                    if (true == settings["rectangle"]["right"].contains("eyeball")) {
                        if (true == settings["rectangle"]["right"]["eyeball"].contains("size_rate")) {
                            step                                  = "Get value / right.eyeball.size_rate";
                            this->param.right_eye.eyeball_scale.x = settings["rectangle"]["right"]["eyeball"]["size_rate"].value("width", this->param.right_eye.eyeball_scale.x);
                            this->param.right_eye.eyeball_scale.y = settings["rectangle"]["right"]["eyeball"]["size_rate"].value("height", this->param.right_eye.eyeball_scale.y);
                        }
                        if (true == settings["rectangle"]["right"]["eyeball"].contains("offset")) {
                            step                                = "Get value / right.eyeball.offset";
                            this->param.right_eye.eyeball.x     = settings["rectangle"]["right"]["eyeball"]["offset"].value("y", this->param.right_eye.eyeball.x);
                            this->param.right_eye.eyeball.y     = -settings["rectangle"]["right"]["eyeball"]["offset"].value("z", this->param.right_eye.eyeball.y);
                            this->param.right_eye.eyeball.angle = -settings["rectangle"]["right"]["eyeball"]["offset"].value("angle", this->param.right_eye.eyeball.angle);
                        }
                    }
                    if (true == settings["rectangle"]["right"].contains("cornea")) {
                        if (true == settings["rectangle"]["right"]["cornea"].contains("outside")) {
                            step                                        = "Get value / right.cornea.outside";
                            this->param.right_eye.cornea_outside.enable = //
                                    settings["rectangle"]["right"]["cornea"]["outside"].value("enable", this->param.right_eye.cornea_outside.enable);
                            this->param.right_eye.cornea_outside.speed = //
                                    settings["rectangle"]["right"]["cornea"]["outside"].value("speed", this->param.right_eye.cornea_outside.speed);
                            this->param.right_eye.cornea_outside.alpha = //
                                    settings["rectangle"]["right"]["cornea"]["outside"].value("alpha", this->param.right_eye.cornea_outside.alpha);
                            if (true == settings["rectangle"]["right"]["cornea"]["outside"].contains("size_rate")) {
                                this->param.right_eye.cornea_outside.scale.x = //
                                        settings["rectangle"]["right"]["cornea"]["outside"]["size_rate"].value("width", this->param.right_eye.cornea_outside.scale.x);
                                this->param.right_eye.cornea_outside.scale.y = //
                                        settings["rectangle"]["right"]["cornea"]["outside"]["size_rate"].value("height", this->param.right_eye.cornea_outside.scale.y);
                            }
                        }
                    }
                    if (true == settings["rectangle"]["right"].contains("cornea")) {
                        if (true == settings["rectangle"]["right"]["cornea"].contains("inside")) {
                            step                                       = "Get value / right.cornea.inside";
                            this->param.right_eye.cornea_inside.enable = //
                                    settings["rectangle"]["right"]["cornea"]["inside"].value("enable", this->param.right_eye.cornea_inside.enable);
                            this->param.right_eye.cornea_inside.speed = //
                                    settings["rectangle"]["right"]["cornea"]["inside"].value("speed", this->param.right_eye.cornea_inside.speed);
                            this->param.right_eye.cornea_inside.alpha = //
                                    settings["rectangle"]["right"]["cornea"]["inside"].value("alpha", this->param.right_eye.cornea_inside.alpha);
                            if (true == settings["rectangle"]["right"]["cornea"]["inside"].contains("size_rate")) {
                                this->param.right_eye.cornea_inside.scale.x = //
                                        settings["rectangle"]["right"]["cornea"]["inside"]["size_rate"].value("width", this->param.right_eye.cornea_inside.scale.x);
                                this->param.right_eye.cornea_inside.scale.y = //
                                        settings["rectangle"]["right"]["cornea"]["inside"]["size_rate"].value("height", this->param.right_eye.cornea_inside.scale.y);
                            }
                        }
                    }
                }
            }
            if (true == settings.contains("blink_time")) {
                step                           = "Get value / blink_time";
                this->param.blink_time_quickly = settings["blink_time"].value("quickly_ms", this->param.blink_time_quickly);
                this->param.blink_time_min     = settings["blink_time"].value("min_ms", this->param.blink_time_min);
                this->param.blink_time_max     = settings["blink_time"].value("max_ms", this->param.blink_time_max);
                this->param.blink_time_limit   = settings["blink_time"].value("limit_ms", this->param.blink_time_limit);
                this->param.blink_time_offset  = settings["blink_time"].value("offset_ms", this->param.blink_time_offset);
            }
            if (true == settings.contains("image")) {
                // left
                if (true == settings["image"].contains("left")) {
                    if (true == settings["image"]["left"].contains("eyelid")) {
                        step = "Get value / image.left.eyelid";
                        this->param.left_eye.image.eyelid.clear();
                        for (nlohmann::json::iterator it = settings["image"]["left"]["eyelid"].begin(); it != settings["image"]["left"]["eyelid"].end(); ++it) {
                            const nlohmann::json &item = *it;
                            if (true == item.contains("id")) {
                                if (true == item.contains("files")) {
                                    std::string it_name = "";
                                    bool it_mirror      = false;
                                    if (true == item.contains("name")) {
                                        it_name = item["name"];
                                    }
                                    if (true == item.contains("mirror")) {
                                        it_mirror = (item["mirror"] ? true : false);
                                    }
                                    StParameter::StImageInfo info((int)item["id"], it_name, it_mirror);
                                    info.files.clear();
                                    step = "Get value / image.left.eyelid.file";
                                    for (auto &element : item["files"]) {
                                        const nlohmann::json &r_path = element;
                                        std::filesystem::path f_path = this->param.path;
                                        f_path /= (std::string)r_path;
                                        if (true == std::filesystem::is_regular_file(f_path)) {
                                            info.files.push_back(std::filesystem::canonical(f_path));
                                        }
                                    }
                                    if (0 < info.files.size()) {
                                        this->param.left_eye.image.eyelid.push_back(info);
                                    }
                                }
                            }
                        }
                    }
                    if (true == settings["image"]["left"].contains("eyeball")) {
                        step = "Get value / image.left.eyeball";
                        this->param.left_eye.image.eyeball.clear();
                        for (nlohmann::json::iterator it = settings["image"]["left"]["eyeball"].begin(); it != settings["image"]["left"]["eyeball"].end(); ++it) {
                            const nlohmann::json &item = *it;
                            if (true == item.contains("id")) {
                                if (true == item.contains("files")) {
                                    std::string it_name = "";
                                    bool it_mirror      = false;
                                    if (true == item.contains("name")) {
                                        it_name = item["name"];
                                    }
                                    if (true == item.contains("mirror")) {
                                        it_mirror = (item["mirror"] ? true : false);
                                    }
                                    StParameter::StImageInfo info((int)item["id"], it_name, it_mirror);
                                    info.files.clear();
                                    step = "Get value / image.left.eyeball.file";
                                    for (auto &element : item["files"]) {
                                        const nlohmann::json &r_path = element;
                                        std::filesystem::path f_path = this->param.path;
                                        f_path /= (std::string)r_path;
                                        if (true == std::filesystem::is_regular_file(f_path)) {
                                            info.files.push_back(std::filesystem::canonical(f_path));
                                        }
                                    }
                                    if (0 < info.files.size()) {
                                        this->param.left_eye.image.eyeball.push_back(info);
                                    }
                                }
                            }
                        }
                    }
                    if (true == settings["image"]["left"].contains("cornea")) {
                        if (true == settings["image"]["left"]["cornea"].contains("outside")) {
                            step = "Get value / image.left.cornea.outside";
                            this->param.left_eye.image.cornea_outside.clear();
                            for (nlohmann::json::iterator it = settings["image"]["left"]["cornea"]["outside"].begin(); it != settings["image"]["left"]["cornea"]["outside"].end();
                                 ++it) {
                                const nlohmann::json &item = *it;
                                if (true == item.contains("id")) {
                                    if (true == item.contains("files")) {
                                        std::string it_name = "";
                                        bool it_mirror      = false;
                                        if (true == item.contains("name")) {
                                            it_name = item["name"];
                                        }
                                        if (true == item.contains("mirror")) {
                                            it_mirror = (item["mirror"] ? true : false);
                                        }
                                        StParameter::StImageInfo info((int)item["id"], it_name, it_mirror);
                                        info.files.clear();
                                        step = "Get value / image.left.cornea.outside.file";
                                        for (auto &element : item["files"]) {
                                            const nlohmann::json &r_path = element;
                                            std::filesystem::path f_path = this->param.path;
                                            f_path /= (std::string)r_path;
                                            if (true == std::filesystem::is_regular_file(f_path)) {
                                                info.files.push_back(std::filesystem::canonical(f_path));
                                            }
                                        }
                                        if (0 < info.files.size()) {
                                            this->param.left_eye.image.cornea_outside.push_back(info);
                                        }
                                    }
                                }
                            }
                            if (0 == this->param.left_eye.image.cornea_outside.size()) {
                                this->param.left_eye.cornea_outside.enable = false;
                            }
                        }
                        if (true == settings["image"]["left"]["cornea"].contains("inside")) {
                            step = "Get value / image.left.cornea.inside";
                            this->param.left_eye.image.cornea_inside.clear();
                            for (nlohmann::json::iterator it = settings["image"]["left"]["cornea"]["inside"].begin(); it != settings["image"]["left"]["cornea"]["inside"].end();
                                 ++it) {
                                const nlohmann::json &item = *it;
                                if (true == item.contains("id")) {
                                    if (true == item.contains("files")) {
                                        std::string it_name = "";
                                        bool it_mirror      = false;
                                        if (true == item.contains("name")) {
                                            it_name = item["name"];
                                        }
                                        if (true == item.contains("mirror")) {
                                            it_mirror = (item["mirror"] ? true : false);
                                        }
                                        StParameter::StImageInfo info((int)item["id"], it_name, it_mirror);
                                        info.files.clear();
                                        step = "Get value / image.left.cornea.inside.file";
                                        for (auto &element : item["files"]) {
                                            const nlohmann::json &r_path = element;
                                            std::filesystem::path f_path = this->param.path;
                                            f_path /= (std::string)r_path;
                                            if (true == std::filesystem::is_regular_file(f_path)) {
                                                info.files.push_back(std::filesystem::canonical(f_path));
                                            }
                                        }
                                        if (0 < info.files.size()) {
                                            this->param.left_eye.image.cornea_inside.push_back(info);
                                        }
                                    }
                                }
                            }
                            if (0 == this->param.left_eye.image.cornea_inside.size()) {
                                this->param.left_eye.cornea_inside.enable = false;
                            }
                        }
                    }
                }
                // right
                if (true == settings["image"].contains("right")) {
                    if (true == settings["image"]["right"].contains("eyelid")) {
                        step = "Get value / image.right.eyelid";
                        this->param.right_eye.image.eyelid.clear();
                        for (nlohmann::json::iterator it = settings["image"]["right"]["eyelid"].begin(); it != settings["image"]["right"]["eyelid"].end(); ++it) {
                            const nlohmann::json &item = *it;
                            if (true == item.contains("id")) {
                                if (true == item.contains("files")) {
                                    std::string it_name = "";
                                    bool it_mirror      = false;
                                    if (true == item.contains("name")) {
                                        it_name = item["name"];
                                    }
                                    if (true == item.contains("mirror")) {
                                        it_mirror = (item["mirror"] ? true : false);
                                    }
                                    StParameter::StImageInfo info((int)item["id"], it_name, it_mirror);
                                    info.files.clear();
                                    step = "Get value / image.right.eyelid.file";
                                    for (auto &element : item["files"]) {
                                        const nlohmann::json &r_path = element;
                                        std::filesystem::path f_path = this->param.path;
                                        f_path /= (std::string)r_path;
                                        if (true == std::filesystem::is_regular_file(f_path)) {
                                            info.files.push_back(std::filesystem::canonical(f_path));
                                        }
                                    }
                                    if (0 < info.files.size()) {
                                        this->param.right_eye.image.eyelid.push_back(info);
                                    }
                                }
                            }
                        }
                    }
                    if (true == settings["image"]["right"].contains("eyeball")) {
                        step = "Get value / image.right.eyeball";
                        this->param.right_eye.image.eyeball.clear();
                        for (nlohmann::json::iterator it = settings["image"]["right"]["eyeball"].begin(); it != settings["image"]["right"]["eyeball"].end(); ++it) {
                            const nlohmann::json &item = *it;
                            if (true == item.contains("id")) {
                                if (true == item.contains("files")) {
                                    std::string it_name = "";
                                    bool it_mirror      = false;
                                    if (true == item.contains("name")) {
                                        it_name = item["name"];
                                    }
                                    if (true == item.contains("mirror")) {
                                        it_mirror = (item["mirror"] ? true : false);
                                    }
                                    StParameter::StImageInfo info((int)item["id"], it_name, it_mirror);
                                    info.files.clear();
                                    step = "Get value / image.right.eyeball.file";
                                    for (auto &element : item["files"]) {
                                        const nlohmann::json &r_path = element;
                                        std::filesystem::path f_path = this->param.path;
                                        f_path /= (std::string)r_path;
                                        if (true == std::filesystem::is_regular_file(f_path)) {
                                            info.files.push_back(std::filesystem::canonical(f_path));
                                        }
                                    }
                                    if (0 < info.files.size()) {
                                        this->param.right_eye.image.eyeball.push_back(info);
                                    }
                                }
                            }
                        }
                    }
                    if (true == settings["image"]["right"].contains("cornea")) {
                        if (true == settings["image"]["right"]["cornea"].contains("outside")) {
                            step = "Get value / image.right.cornea.outside";
                            this->param.right_eye.image.cornea_outside.clear();
                            for (nlohmann::json::iterator it = settings["image"]["right"]["cornea"]["outside"].begin(); it != settings["image"]["right"]["cornea"]["outside"].end();
                                 ++it) {
                                const nlohmann::json &item = *it;
                                if (true == item.contains("id")) {
                                    if (true == item.contains("files")) {
                                        std::string it_name = "";
                                        bool it_mirror      = false;
                                        if (true == item.contains("name")) {
                                            it_name = item["name"];
                                        }
                                        if (true == item.contains("mirror")) {
                                            it_mirror = (item["mirror"] ? true : false);
                                        }
                                        StParameter::StImageInfo info((int)item["id"], it_name, it_mirror);
                                        info.files.clear();
                                        step = "Get value / image.right.cornea.outside.file";
                                        for (auto &element : item["files"]) {
                                            const nlohmann::json &r_path = element;
                                            std::filesystem::path f_path = this->param.path;
                                            f_path /= (std::string)r_path;
                                            if (true == std::filesystem::is_regular_file(f_path)) {
                                                info.files.push_back(std::filesystem::canonical(f_path));
                                            }
                                        }
                                        if (0 < info.files.size()) {
                                            this->param.right_eye.image.cornea_outside.push_back(info);
                                        }
                                    }
                                }
                            }
                            if (0 == this->param.right_eye.image.cornea_outside.size()) {
                                this->param.right_eye.cornea_outside.enable = false;
                            }
                        }
                        if (true == settings["image"]["right"]["cornea"].contains("inside")) {
                            step = "Get value / image.right.cornea.inside";
                            this->param.right_eye.image.cornea_inside.clear();
                            for (nlohmann::json::iterator it = settings["image"]["right"]["cornea"]["inside"].begin(); it != settings["image"]["right"]["cornea"]["inside"].end();
                                 ++it) {
                                const nlohmann::json &item = *it;
                                if (true == item.contains("id")) {
                                    if (true == item.contains("files")) {
                                        std::string it_name = "";
                                        bool it_mirror      = false;
                                        if (true == item.contains("name")) {
                                            it_name = item["name"];
                                        }
                                        if (true == item.contains("mirror")) {
                                            it_mirror = (item["mirror"] ? true : false);
                                        }
                                        StParameter::StImageInfo info((int)item["id"], it_name, it_mirror);
                                        info.files.clear();
                                        step = "Get value / image.right.cornea.inside.file";
                                        for (auto &element : item["files"]) {
                                            const nlohmann::json &r_path = element;
                                            std::filesystem::path f_path = this->param.path;
                                            f_path /= (std::string)r_path;
                                            if (true == std::filesystem::is_regular_file(f_path)) {
                                                info.files.push_back(std::filesystem::canonical(f_path));
                                            }
                                        }
                                        if (0 < info.files.size()) {
                                            this->param.right_eye.image.cornea_inside.push_back(info);
                                        }
                                    }
                                }
                            }
                            if (0 == this->param.right_eye.image.cornea_inside.size()) {
                                this->param.right_eye.cornea_inside.enable = false;
                            }
                        }
                    }
                }
            }
            step                     = "The process has been completed";
            this->param.setting_file = json_file;
            result                   = true;
        } else {
            step = "The file does not exist";
        }
    } catch (const std::logic_error &err) {
        result = false;
        step   = err.what();
    } catch (...) {
        result = false;
    }

    if (true == result) {
        this->reload_param();
        this->_flag_reload = true;
    } else {
        std::string message = "Parsing of setting file failed.";
        message.append(" step[");
        message.append(step);
        message.append("]");

        throw new std::logic_error(message);
    }

    return result;
}

void EyeWidget::_init()
{
    if (false == this->_flag_initialized) {
        this->_flag_initialized = true;

        /* --------------------------------------------------- */
        // set timer
        /* --------------------------------------------------- */
        this->_timer_start = new QTimer(this);
        this->_timer_start->setSingleShot(true);
        this->connect(this->_timer_start, SIGNAL(timeout()), this, SLOT(showFullScreen()));

        this->current_time.start();

        /* --------------------------------------------------- */
        // load
        /* --------------------------------------------------- */
        this->eyelid->set_emotion(miens_close);
        this->eyelid->calc_animation(0);
        this->last_ros_msg_time = this->current_time.elapsed();
        this->eyelid->calc_animation(this->current_time.elapsed());
        this->_thinking_next_time_notAccepted = current_time.elapsed() + EYE_BLINK_TIME_START_TIME_MS;
    }
}

bool EyeWidget::exec_start()
{
    this->_flag_start = true;
    if (false == this->_flag_initialized) {
        this->_init();
        this->reload_param();
    }
    this->_timer_start->start(100);
    return true;
}

void EyeWidget::closing()
{
    this->eyelid->closing();
    this->eyeball->closing();
    this->_flag_running = false;
    if (true == this->_flag_initialized) {
        this->_flag_initialized = false;
#if LOGGER_ROS_INFO_DETAIL
        printf("\n==============\n CLOSE APP.\n==============\n");
#endif
    }
}

} // namespace maid_robot_system
