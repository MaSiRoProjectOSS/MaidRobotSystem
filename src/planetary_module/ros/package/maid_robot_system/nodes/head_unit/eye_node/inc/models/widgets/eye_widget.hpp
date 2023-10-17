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
    bool exec_start();
    void closing();
    bool is_start();
    bool request_update();

public:
    // =============================
    bool load();
    bool reload_param();

public:
    // =============================
    // PUBLIC :
    // =============================
    void cornea_order();
    void emotion(MIENS value);
    void dimensions(float dimensions);
    void stare(float distance, float left_y, float left_z, float right_y, float right_z);

public:
    // =============================
    // PUBLIC : Setter
    // =============================
    bool set_setting_file(std::string value);

public:
    // =============================
    // PUBLIC : Getter
    // =============================
    std::string output_message(bool verbose);

private:
    // =============================
    // PRIVATE : Function
    // =============================
    void _screen_calculate();
    void _screen_update();
    std::string _read_file(const std::string &path);
    std::string _miens_text(MIENS value);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    bool _flag_initialized = false;
    PartsEyeball *eyeball;
    PartsEyelid *eyelid;
    LogStore *logger;

    int _thinking_next_time_notAccepted = 0;
    QTimer *_timer_start;
    volatile bool _flag_reload  = false;
    volatile bool _flag_start   = false;
    volatile bool _flag_running = false;

    // =============================
    QTime current_time;
    int last_ros_msg_time;
    int last_voiceId_time;

    bool flag_voice_id                      = false;
    control_state thinking_flag_notAccepted = control_state::STATE_FREE;

    StVector _request_left;
    StVector _request_right;
    float _request_distance;

private:
    // =============================
    // CONST
    // =============================

#if 1
private:
    // =============================
    // PRIVATE : QT
    // =============================
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
