/**
 * @file model_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_MODEL_IMPLEMENT_HPP
#define MRS_EYE_NODE_MODEL_IMPLEMENT_HPP

#include "logger/log_store.hpp"
#include "maid_robot_system/common_structure.hpp"
#include "math.h"
#include "models/calibration.hpp"
#include "models/data_structure.hpp"

#include <QApplication>
#include <QGLWidget>
#include <QImage>
#include <QPaintEvent>
#include <QPainter>
#include <QPixmap>
#include <QTime>
#include <QTimer>
#include <QWidget>
#include <functional>
#include <maid_robot_system_interfaces/msg/mrs_eye.hpp>
#include <mutex>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace maid_robot_system;

namespace maid_robot_system
{
class ModelImplement {
public:
    // =============================
    // Constructor
    // =============================
    ModelImplement();
    ~ModelImplement();

protected:
    // =============================
    // PUBLIC : OpenGL fuction
    // =============================
    bool event(QEvent *e);
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);

public:
    // =============================
    // PUBLIC : Function
    // =============================
    void open(int argc, char **argv);
    void closing();

    bool calculate();
    void set_msg_eye(int emotions,
                     int pupil_effect,
                     float size,
                     float distance,

                     float left_x,
                     float left_y,

                     float right_x,
                     float right_y);

    // =============================
    // PUBLIC : Getter
    // =============================
    std::string get_setting_file();
    int get_brightness();
    int get_color_r();
    int get_color_g();
    int get_color_b();
    std::string get_lap_time();

    // =============================
    // PUBLIC : Setter
    // =============================
    bool set_setting_file(std::string value);
    bool set_brightness(int value);
    bool set_color_r(int value);
    bool set_color_g(int value);
    bool set_color_b(int value);

private:
    // =============================
    // PRIVATE : Function
    // =============================
    bool _set_param(std::string json_file);
    std::string _read_file(const std::string &path);
    // =============================
    // PRIVATE : Function
    // =============================

private:
    // =============================
    // PRIVATE : parameter
    // =============================
    StParameter _param;
    QApplication *app;

    // =============================
    // PRIVATE : Variable
    // =============================
};

} // namespace maid_robot_system

#endif
