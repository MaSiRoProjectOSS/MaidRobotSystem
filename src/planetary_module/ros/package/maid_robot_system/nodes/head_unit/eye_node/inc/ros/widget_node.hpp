/**
 * @file widget_node.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_WIDGET_NODE_HPP
#define MRS_EYE_NODE_WIDGET_NODE_HPP

#include "eye_node_settings.hpp"
#include "maid_robot_system/common_structure.hpp"
#include "models/data_structure.hpp"
#include "widgets/eye_widget.hpp"

#include <QApplication>
#include <string>

using namespace std;

namespace maid_robot_system
{
class WidgetNode {
public:
    // =============================
    // Constructor
    // widget_node.cpp
    // =============================
    WidgetNode(std::string node_name, int argc, char **argv);
    ~WidgetNode();

public:
    // =============================
    // PUBLIC : Function
    // file : widget_node.cpp
    // =============================
    bool exec_start();
    void respawn();
    bool closing();

public:
    // =============================
    // PUBLIC : Control
    // file : widget_node_control.cpp
    // =============================
    void effect_cornea_order();
    void emotion(MIENS value);
    void dimensions(float dimensions);
    void stare(float distance, float left_y, float left_z, float right_y, float right_z);

public:
    // =============================
    // PUBLIC : Getter
    // file : widget_node_getter.cpp
    // =============================
    bool is_start();
    bool request_update();
    int running_exec();
    std::string output_message(bool verbose = false);
    std::string get_setting_file();
    int get_brightness();
    int get_eyelid_color_r();
    int get_eyelid_color_g();
    int get_eyelid_color_b();
    int get_ciliary_color_r();
    int get_ciliary_color_g();
    int get_ciliary_color_b();

public:
    // =============================
    // PUBLIC : Setter
    // file : widget_node_setter.cpp
    // =============================
    bool set_setting_file(std::string value);
    bool set_brightness(int value);
    bool set_eyelid_color_r(int value);
    bool set_eyelid_color_g(int value);
    bool set_eyelid_color_b(int value);
    bool set_ciliary_color_r(int value);
    bool set_ciliary_color_g(int value);
    bool set_ciliary_color_b(int value);

    bool is_response();

private:
    // =============================
    // PRIVATE : Function
    // =============================
    // file : widget_node_setter.cpp
    bool _within_range(int value, int min = 0, int max = 255);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    QApplication *_app;
    EyeWidget *_widget = nullptr;

    volatile bool _is_running = false;
};

} // namespace maid_robot_system

#endif
