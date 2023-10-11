/**
 * @file eye_widget.hpp
 * @brief
 * @date 2020-02-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
*/
#ifndef MRS_EYE_NODE_MODELS_EYE_WIDGET_HPP
#define MRS_EYE_NODE_MODELS_EYE_WIDGET_HPP

#include "eye_node_settings.hpp"
#include "maid_robot_system/common_structure.hpp"
#include "math.h"
#include "models/calibration.hpp"
#include "models/data_structure.hpp"
#include "models/log_store.hpp"
#include "parts/parts_eyeball.hpp"
#include "parts/parts_eyelid.hpp"

#include <QApplication>
#include <QGLWidget>
#include <QImage>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QPaintEvent>
#include <QPainter>
#include <QPixmap>
#include <QTime>
#include <QTimer>
#include <QVector>
#include <QWidget>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>

using namespace std;

namespace maid_robot_system
{
class EyeWidget : public QOpenGLWidget, protected QOpenGLFunctions {
private:
    typedef enum ENUM_CONTROL_STATE
    {
        STATE_FREE,         //!< 瞬き制御の未制御区間
        STATE_ACCEPTED,     //!< 瞬き制御の許可区間
        STATE_NOT_ACCEPTED, //!< 瞬き制御の不許可区間

    } control_state;

public:
    // =============================
    // Constructor
    // =============================
    EyeWidget(QWidget *parent = nullptr);
    ~EyeWidget();

    // =============================
    // PUBLIC : Variable
    // =============================
    StParameter param;

public:
    // =============================
    // QT fuction
    // =============================
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

public:
    // =============================
    // PUBLIC : Function
    // =============================
    void initialize(StParameter param);
    void closing();
    // =============================
    // PUBLIC :
    // =============================
    void update_screen();
    void pupil_order();
    void cmd_eye_input(int emotions, int pupil_effect, float size, float distance, float left_x, float left_y, float right_x, float right_y);

    std::string output_message();

    // =============================
    bool reload_param();
    bool set_setting_file(std::string value);

private:
    // =============================
    // PRIVATE : Function
    // =============================
    void make_image();

    std::string _read_file(const std::string &path);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    PartsEyeball eyeball;
    PartsEyelid eyelid;
    LogStore logger;

    QTime current_time;
    int last_ros_msg_time;
    int last_voiceId_time;

    bool flag_first_request                 = false;
    bool flag_voice_id                      = false;
    control_state thinking_flag_notAccepted = control_state::STATE_FREE;

private:
    // =============================
    // CONST
    // =============================

#if 1
private:
    /* ============================================= */
    GLuint m_program;

    GLuint m_vao;
    GLuint m_vbo;

    QVector<GLfloat> m_vertices;

    const char *vshader_src = "#version 400 core\n"
                              "layout(location = 0) in vec4 position;\n"
                              "void main(){\n"
                              "gl_Position = position;\n"
                              "}\n";
    const char *fshader_src = "#version 400 core\n"
                              "out vec4 color;\n"
                              "void main(){\n"
                              "color = vec4(1.0,1.0,1.0,1.0);\n"
                              "}\n";
    /* ============================================= */
#endif
};

} // namespace maid_robot_system

#endif
