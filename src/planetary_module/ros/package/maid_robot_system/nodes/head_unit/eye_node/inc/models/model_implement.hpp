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

#include "eye_node_settings.hpp"
#include "maid_robot_system/common_structure.hpp"
#include "models/calibration.hpp"
#include "models/data_structure.hpp"
#include "models/widgets/eye_widget.hpp"

#include <QApplication>
#include <string>

using namespace std;

namespace maid_robot_system
{
class ModelImplement {
public:
    // =============================
    // Constructor
    // =============================
    ModelImplement();
    ~ModelImplement();

public:
    // =============================
    // PUBLIC : Function
    // =============================
    bool open(int argc, char **argv);
    bool exec();
    bool closing();

    bool calculate();
    void set_msg_eye(int emotions,
                     int pupil_effect,
                     float size,
                     float distance,

                     float left_x,
                     float left_y,

                     float right_x,
                     float right_y);
    void effect_pupil_order();

    // =============================
    // PUBLIC : Setter
    // =============================
    bool set_setting_file(std::string value);
    bool set_brightness(int value);
    bool set_color_r(int value);
    bool set_color_g(int value);
    bool set_color_b(int value);

    // =============================
    // PUBLIC : Getter
    // =============================
    std::string output_message();
    std::string get_setting_file();
    int get_brightness();
    int get_color_r();
    int get_color_g();
    int get_color_b();

private:
    // =============================
    // PRIVATE : Function
    // =============================
    bool _set_param();
    bool _parse_param(std::string json_file);
    std::string _read_file(const std::string &path);

private:
    // =============================
    // PRIVATE : parameter
    // =============================
    QApplication *app;
    EyeWidget *_widget = nullptr;

    // =============================
    // PRIVATE : Variable
    // =============================
};

} // namespace maid_robot_system

#endif
