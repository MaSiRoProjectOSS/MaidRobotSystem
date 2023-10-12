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
        if (true == std::filesystem::exists(json_file)) {
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
        printf("%s%d\n", "    r: ", this->param.color.r);
        printf("%s%d\n", "    g: ", this->param.color.g);
        printf("%s%d\n", "    b: ", this->param.color.b);
        printf("%s\n", "=========================================================");
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
        this->eyeball.Initialize(this->param.l_angle - this->param.eyeball_angle, this->param.r_angle + this->param.eyeball_angle);

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
        this->eyelid.calc_animation(EYE_BLINK_TIME_MILLISECOND_DEFAULT * -2);
        this->eyelid.calc_animation(0);
        this->eyelid.calc_animation(EYE_BLINK_TIME_MILLISECOND_DEFAULT);
        // this->resize(this->param.screen_size.width, this->param.screen_size.height);
        this->last_ros_msg_time = this->current_time.elapsed();
        this->eyelid.calc_animation(this->current_time.elapsed());
    }
}

bool EyeWidget::exec()
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
