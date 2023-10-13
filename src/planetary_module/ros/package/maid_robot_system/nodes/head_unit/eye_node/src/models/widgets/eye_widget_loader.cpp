/**
 * @file eye_widget_loader.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace maid_robot_system
{
#define DEBUG_OUTPUT 1

bool EyeWidget::set_setting_file(std::string json_file)
{
    bool result = false;
    try {
        StVector left_eye_center;
        StVector right_eye_center;
        StVector eye_size;
        if (true == std::filesystem::exists(json_file)) {
            nlohmann::json settings = nlohmann::json::parse(this->_read_file(json_file));
            this->param.path        = settings.value("path", this->param.path);
            this->param.name        = settings.value("name", this->param.name);
            if (true == settings.contains("display")) {
                this->param.screen_scale       = settings["display"].value("scale", this->param.screen_scale);
                this->param.screen_size.width  = settings["display"].value("width", this->param.screen_size.width);
                this->param.screen_size.height = settings["display"].value("height", this->param.screen_size.height);
            }
            right_eye_center.x = this->param.screen_size.width * 0.25;
            right_eye_center.y = this->param.screen_size.height * 0.5;
            left_eye_center.x  = this->param.screen_size.width * 0.75;
            left_eye_center.y  = this->param.screen_size.height * 0.5;
            eye_size.x         = this->param.screen_size.width * 0.5;
            eye_size.y         = this->param.screen_size.height;

            if (true == settings.contains("rectangle")) {
                ////////// left
                if (true == settings["rectangle"].contains("left")) {
                    if (true == settings["rectangle"]["left"].contains("eyelid")) {
                        if (true == settings["rectangle"]["left"]["eyelid"].contains("size_rate")) {
                            this->param.left_eye.eyelid.width = //
                                    (int)(eye_size.x * settings["rectangle"]["left"]["eyelid"]["size_rate"].value("width", (double)this->param.left_eye.eyelid.width));
                            this->param.left_eye.eyelid.height = //
                                    (int)(eye_size.y * settings["rectangle"]["left"]["eyelid"]["size_rate"].value("height", (double)this->param.left_eye.eyelid.height));
                        }
                        if (true == settings["rectangle"]["left"]["eyelid"].contains("offset")) {
                            this->param.left_eye.eyelid.x     = (int)(left_eye_center.x - (this->param.left_eye.eyelid.width / 2.0)
                                                                  + settings["rectangle"]["left"]["eyelid"]["offset"].value("x", this->param.left_eye.eyelid.x));
                            this->param.left_eye.eyelid.y     = (int)(left_eye_center.y - (this->param.left_eye.eyelid.height / 2.0)
                                                                  + settings["rectangle"]["left"]["eyelid"]["offset"].value("y", this->param.left_eye.eyelid.y));
                            this->param.left_eye.eyelid.angle = (int)(settings["rectangle"]["left"]["eyelid"]["offset"].value("angle", this->param.left_eye.eyelid.angle));
                        }
                    }
                    if (true == settings["rectangle"]["left"].contains("eyeball")) {
                        if (true == settings["rectangle"]["left"]["eyeball"].contains("size_rate")) {
                            this->param.left_eye.eyeball.width = //
                                    (int)(eye_size.x * settings["rectangle"]["left"]["eyeball"]["size_rate"].value("width", (double)this->param.left_eye.eyeball.width));
                            this->param.left_eye.eyeball.height = //
                                    (int)(eye_size.y * settings["rectangle"]["left"]["eyeball"]["size_rate"].value("height", (double)this->param.left_eye.eyeball.height));
                        }
                        if (true == settings["rectangle"]["left"]["eyeball"].contains("offset")) {
                            this->param.left_eye.eyeball.x     = (int)(left_eye_center.x - (this->param.left_eye.eyeball.width / 2.0)
                                                                   + settings["rectangle"]["left"]["eyeball"]["offset"].value("x", this->param.left_eye.eyeball.x));
                            this->param.left_eye.eyeball.y     = (int)(left_eye_center.y - (this->param.left_eye.eyeball.height / 2.0)
                                                                   + settings["rectangle"]["left"]["eyeball"]["offset"].value("y", this->param.left_eye.eyeball.y));
                            this->param.left_eye.eyeball.x     = (int)(settings["rectangle"]["left"]["eyeball"]["offset"].value("x", this->param.left_eye.eyeball.x));
                            this->param.left_eye.eyeball.y     = (int)(settings["rectangle"]["left"]["eyeball"]["offset"].value("y", this->param.left_eye.eyeball.y));
                            this->param.left_eye.eyeball.angle = (int)(settings["rectangle"]["left"]["eyeball"]["offset"].value("angle", this->param.left_eye.eyeball.angle));
                        }
                    }
                    if (true == settings["rectangle"]["left"].contains("cornea")) {
                        if (true == settings["rectangle"]["left"]["cornea"].contains("outside")) {
                            this->param.left_eye.cornea_outside.enable = //
                                    settings["rectangle"]["left"]["cornea"]["outside"].value("enable", this->param.left_eye.cornea_outside.enable);
                            this->param.left_eye.cornea_outside.scale = //
                                    settings["rectangle"]["left"]["cornea"]["outside"].value("scale", this->param.left_eye.cornea_outside.scale);
                            this->param.left_eye.cornea_outside.speed = //
                                    settings["rectangle"]["left"]["cornea"]["outside"].value("speed", this->param.left_eye.cornea_outside.speed);
                        }
                    }
                    if (true == settings["rectangle"]["left"].contains("cornea")) {
                        if (true == settings["rectangle"]["left"]["cornea"].contains("inside")) {
                            this->param.left_eye.cornea_inside.enable = //
                                    settings["rectangle"]["left"]["cornea"]["inside"].value("enable", this->param.left_eye.cornea_inside.enable);
                            this->param.left_eye.cornea_inside.scale = //
                                    settings["rectangle"]["left"]["cornea"]["inside"].value("scale", this->param.left_eye.cornea_inside.scale);
                            this->param.left_eye.cornea_inside.speed = //
                                    settings["rectangle"]["left"]["cornea"]["inside"].value("speed", this->param.left_eye.cornea_inside.speed);
                        }
                    }
                }
                ////////// right
                if (true == settings["rectangle"].contains("right")) {
                    if (true == settings["rectangle"]["right"].contains("eyelid")) {
                        if (true == settings["rectangle"]["right"]["eyelid"].contains("size_rate")) {
                            this->param.right_eye.eyelid.width = //
                                    (int)(eye_size.x * settings["rectangle"]["right"]["eyelid"]["size_rate"].value("width", (double)this->param.right_eye.eyelid.width));
                            this->param.right_eye.eyelid.height = //
                                    (int)(eye_size.y * settings["rectangle"]["right"]["eyelid"]["size_rate"].value("height", (double)this->param.right_eye.eyelid.height));
                        }
                        if (true == settings["rectangle"]["right"]["eyelid"].contains("offset")) {
                            this->param.right_eye.eyelid.x     = (int)(right_eye_center.x - (this->param.right_eye.eyelid.width / 2.0)
                                                                   + settings["rectangle"]["right"]["eyelid"]["offset"].value("x", this->param.right_eye.eyelid.x));
                            this->param.right_eye.eyelid.y     = (int)(right_eye_center.y - (this->param.right_eye.eyelid.height / 2.0)
                                                                   + settings["rectangle"]["right"]["eyelid"]["offset"].value("y", this->param.right_eye.eyelid.y));
                            this->param.right_eye.eyelid.angle = (int)(settings["rectangle"]["right"]["eyelid"]["offset"].value("angle", this->param.right_eye.eyelid.angle));
                        }
                    }
                    if (true == settings["rectangle"]["right"].contains("eyeball")) {
                        if (true == settings["rectangle"]["right"]["eyeball"].contains("size_rate")) {
                            this->param.right_eye.eyeball.width = //
                                    (int)(eye_size.x * settings["rectangle"]["right"]["eyeball"]["size_rate"].value("width", (double)this->param.right_eye.eyeball.width));
                            this->param.right_eye.eyeball.height = //
                                    (int)(eye_size.y * settings["rectangle"]["right"]["eyeball"]["size_rate"].value("height", (double)this->param.right_eye.eyeball.height));
                        }
                        if (true == settings["rectangle"]["right"]["eyeball"].contains("offset")) {
                            this->param.right_eye.eyeball.x     = (int)(right_eye_center.x - (this->param.right_eye.eyeball.width / 2.0)
                                                                    + settings["rectangle"]["right"]["eyeball"]["offset"].value("x", this->param.right_eye.eyeball.x));
                            this->param.right_eye.eyeball.y     = (int)(right_eye_center.y - (this->param.right_eye.eyeball.height / 2.0)
                                                                    + settings["rectangle"]["right"]["eyeball"]["offset"].value("y", this->param.right_eye.eyeball.y));
                            this->param.right_eye.eyeball.angle = (int)(settings["rectangle"]["right"]["eyeball"]["offset"].value("angle", this->param.right_eye.eyeball.angle));
                        }
                    }
                    if (true == settings["rectangle"]["right"].contains("cornea")) {
                        if (true == settings["rectangle"]["right"]["cornea"].contains("outside")) {
                            this->param.right_eye.cornea_outside.enable = //
                                    settings["rectangle"]["right"]["cornea"]["outside"].value("enable", this->param.right_eye.cornea_outside.enable);
                            this->param.right_eye.cornea_outside.scale = //
                                    settings["rectangle"]["right"]["cornea"]["outside"].value("scale", this->param.right_eye.cornea_outside.scale);
                            this->param.right_eye.cornea_outside.speed = //
                                    settings["rectangle"]["right"]["cornea"]["outside"].value("speed", this->param.right_eye.cornea_outside.speed);
                        }
                    }
                    if (true == settings["rectangle"]["right"].contains("cornea")) {
                        if (true == settings["rectangle"]["right"]["cornea"].contains("inside")) {
                            this->param.right_eye.cornea_inside.enable = //
                                    settings["rectangle"]["right"]["cornea"]["inside"].value("enable", this->param.right_eye.cornea_inside.enable);
                            this->param.right_eye.cornea_inside.scale = //
                                    settings["rectangle"]["right"]["cornea"]["inside"].value("scale", this->param.right_eye.cornea_inside.scale);
                            this->param.right_eye.cornea_inside.speed = //
                                    settings["rectangle"]["right"]["cornea"]["inside"].value("speed", this->param.right_eye.cornea_inside.speed);
                        }
                    }
                }
            }
            if (true == settings.contains("blink_time")) {
                this->param.blink_time_quickly = settings["blink_time"].value("quickly_ms", this->param.blink_time_quickly);
                this->param.blink_time_min     = settings["blink_time"].value("min_ms", this->param.blink_time_min);
                this->param.blink_time_max     = settings["blink_time"].value("max_ms", this->param.blink_time_max);
                this->param.blink_time_limit   = settings["blink_time"].value("limit_ms", this->param.blink_time_limit);
                this->param.blink_time_offset  = settings["blink_time"].value("offset_ms", this->param.blink_time_offset);
            }
            this->param.setting_file = json_file;
            result                   = true;
        }
    } catch (...) {
        result = false;
    }

    if (true == result) {
        this->load();
        this->reload_param();
#if DEBUG_OUTPUT
        printf("=========================================================\n");
        printf("file: %s\n", this->param.setting_file.c_str());
        printf("path: %s\n", this->param.path.c_str());
        printf("name: %s\n", this->param.name.c_str());

        printf("eyelid:\n");
        printf("  color:\n");
        printf("    r: %d\n", this->param.eyelid_color.r);
        printf("    g: %d\n", this->param.eyelid_color.g);
        printf("    b: %d\n", this->param.eyelid_color.b);
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

        printf("cornea:\n");
        printf("  left_eye:\n");
        printf("    outside:\n");
        printf("      enable: %s\n", this->param.left_eye.cornea_outside.enable ? "True" : "False");
        printf("      speed: %3.1f\n", this->param.left_eye.cornea_outside.speed);
        printf("      scale: %3.1f\n", this->param.left_eye.cornea_outside.scale);
        printf("    inside:\n");
        printf("      enable: %s\n", this->param.left_eye.cornea_inside.enable ? "True" : "False");
        printf("      speed: %3.1f\n", this->param.left_eye.cornea_inside.speed);
        printf("      scale: %3.1f\n", this->param.left_eye.cornea_inside.scale);
        printf("  right_eye:\n");
        printf("    outside:\n");
        printf("      enable: %s\n", this->param.right_eye.cornea_outside.enable ? "True" : "False");
        printf("      speed: %3.1f\n", this->param.right_eye.cornea_outside.speed);
        printf("      scale: %3.1f\n", this->param.right_eye.cornea_outside.scale);
        printf("    inside:\n");
        printf("      enable: %s\n", this->param.right_eye.cornea_inside.enable ? "True" : "False");
        printf("      speed: %3.1f\n", this->param.right_eye.cornea_inside.speed);
        printf("      scale: %3.1f\n", this->param.right_eye.cornea_inside.scale);

        printf("ciliary:\n");
        printf("  color:\n");
        printf("    r: %d\n", this->param.ciliary_color.r);
        printf("    g: %d\n", this->param.ciliary_color.g);
        printf("    b: %d\n", this->param.ciliary_color.b);

        printf("eye_blink:\n");
        printf("  quickly: %3.1f\n", this->param.blink_time_quickly);
        printf("  min: %3.1f\n", this->param.blink_time_min);
        printf("  max: %3.1f\n", this->param.blink_time_max);
        printf("  limit: %3.1f\n", this->param.blink_time_limit);
        printf("  offset: %3.1f\n", this->param.blink_time_offset);

        printf("display:\n");
        printf("  brightness: %d\n", this->param.brightness);
        printf("  width: %d\n", this->param.screen_size.width);
        printf("  height: %d\n", this->param.screen_size.height);
        printf("  scale: %3.1f\n", this->param.screen_scale);
        printf("=========================================================\n");
#endif
    }

    return result;
}

void EyeWidget::_init()
{
    if (false == this->_flag_initialized) {
        this->_flag_initialized = true;
        /* --------------------------------------------------- */
        // register[START]
        /* --------------------------------------------------- */
        this->eyeball.initialize(this->param.left_eye.eyeball.angle, this->param.right_eye.eyeball.angle);

        /* --------------------------------------------------- */
        // set timer
        /* --------------------------------------------------- */
        // QTimer *timer = new QTimer(this);
        // connect(timer, SIGNAL(timeout()), this, SLOT(update()));
        // timer->start(1);
        // qApp->installEventFilter(this);
        this->current_time.start();

        /* --------------------------------------------------- */
        // load
        /* --------------------------------------------------- */
        this->eyelid.set_emotion(miens_close);
        this->eyelid.calc_animation(0);
        this->last_ros_msg_time = this->current_time.elapsed();
        this->eyelid.calc_animation(this->current_time.elapsed());
        this->_thinking_next_time_notAccepted = current_time.elapsed() + EYE_BLINK_TIME_START_TIME_MS;
    }
}

bool EyeWidget::start_exec()
{
    if (false == this->_flag_initialized) {
        this->_init();
        this->reload_param();
    }
    this->showFullScreen();
    return true;
}

void EyeWidget::closing()
{
    if (true == this->_flag_initialized) {
        this->_flag_initialized = false;
        printf("\n==============\n CLOSE APP.\n==============\n");
    }
}

} // namespace maid_robot_system
