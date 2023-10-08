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

#include "logger/log_store.hpp"
#include "maid_robot_system/common_structure.hpp"
#include "math.h"
#include "models/calibration.hpp"
#include "models/data_structure.hpp"
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
#include <maid_robot_system_interfaces/msg/mrs_eye.hpp>
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
public:
    EyeWidget(QWidget *parent = nullptr);
    ~EyeWidget();
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

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void initialize(StParameter param);
    void set_param(StParameter param);
    void Setup(float param_calibration_eye_blink_time_offset);
    void Closing();
    void UpdateScreen();

    /* ============================================= */
    void CMD_voiceId();
    void CMD_eye_input(const maid_robot_system_interfaces::msg::MrsEye msg);

    /* ============================================= */
protected:
    void paintEvent(QPaintEvent *event) override;
    bool eventFilter(QObject *obj, QEvent *event);
    bool event(QEvent *e) override;
    void resizeEvent(QResizeEvent *event) override;
    /* ============================================= */
private:
    /* ============================================= */
    typedef enum ENUM_CONTROL_STATE
    {
        STATE_FREE,         //!< 瞬き制御の未制御区間
        STATE_ACCEPTED,     //!< 瞬き制御の許可区間
        STATE_NOT_ACCEPTED, //!< 瞬き制御の不許可区間

    } control_state;
    /* ============================================= */
    // for convenience
    using json = nlohmann::json;
    std::mutex mtx_;
    /* ============================================= */
    int calibration_eyelid_size_x = CALIBRATION_EYELID_SIZE_X;
    int calibration_eyelid_size_y = CALIBRATION_EYELID_SIZE_Y;

    double window_size_x = WINDOW_SIZE_X;
    double window_size_y = WINDOW_SIZE_Y;

    double eyeball_size_x              = EYEBALL_SIZE_X;
    double eyeball_size_y              = EYEBALL_SIZE_Y;
    int thinking_next_time_notAccepted = 0;
    /* ============================================= */
    // Qt::ImageConversionFlag imageFlag = Qt::NoOpaqueDetection;
    Qt::ImageConversionFlag imageFlag = Qt::OrderedAlphaDither;
    /* ============================================= */
    void make_image();
    void log();
    void loadSkin(StParameter param);
    void loadEyelid(StParameter param);
    void loadEyeball(StParameter param);

    /* ============================================= */
    PartsEyeball eyeball;
    PartsEyelid eyelid;
    LogStore logger;

    QTime current_time;
    int log_timer;
    int last_ros_msg_time;
    int last_voiceId_time;

    /* ============================================= */
    bool flag_first_request                 = false;
    bool flag_voiceId                       = false;
    control_state thinking_flag_notAccepted = control_state::STATE_FREE;
    /* ============================================= */
};

} // namespace maid_robot_system

#endif
