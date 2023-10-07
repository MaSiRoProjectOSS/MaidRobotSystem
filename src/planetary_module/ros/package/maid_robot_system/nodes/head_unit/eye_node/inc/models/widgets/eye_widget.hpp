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

#define SET_GLWIDGET 1

#include "logger/log_store.hpp"
#include "maid_robot_system/common_structure.hpp"
#include "math.h"
#include "models/calibration.hpp"
#include "parts/parts_eyeball.hpp"
#include "parts/parts_eyelid.hpp"

#include <QApplication>
#include <QImage>
#include <QPaintEvent>
#include <QPainter>
#include <QPixmap>
#include <QTime>
#include <QTimer>
#include <functional>
#include <maid_robot_system_interfaces/msg/mrs_eye.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>
#if SET_GLWIDGET
#include <QGLWidget>
#else
#include <QWidget>
#endif

using namespace std;

namespace maid_robot_system
{
class EyeWidget :
#if SET_GLWIDGET
        public QGLWidget
#else
        public QWidget
#endif
{
    // Q_OBJECT
public:
    EyeWidget(QWidget *parent = 0);
    ~EyeWidget();
    /* ============================================= */
#if SET_GLWIDGET
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
#endif
    void Initialize(QString skin_name,
                    double param_calibration_l_x,
                    double param_calibration_l_y,
                    double param_calibration_l_angle,
                    double param_calibration_r_x,
                    double param_calibration_r_y,
                    double param_calibration_r_angle,
                    int param_calibration_eyelid_size_x,
                    int param_calibration_eyelid_size_y,
                    double param_calibration_eyeball_position_l_x,
                    double param_calibration_eyeball_position_l_y,
                    double param_calibration_eyeball_position_r_x,
                    double param_calibration_eyeball_position_r_y,
                    double param_calibration_eyeball_angle,
                    float param_calibration_eye_blink_time_quickly,
                    float param_calibration_eye_blink_time_min,
                    float param_calibration_eye_blink_time_max,
                    float param_calibration_eye_blink_time_limit,
                    float param_calibration_eye_blink_time_offset);
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
    void loadSkin(QString skin_name,
                  double param_calibration_l_x,
                  double param_calibration_l_y,
                  double param_calibration_l_angle,
                  double param_calibration_r_x,
                  double param_calibration_r_y,
                  double param_calibration_r_angle,
                  int param_calibration_eyelid_size_x,
                  int param_calibration_eyelid_size_y,
                  double param_calibration_eyeball_position_l_x,
                  double param_calibration_eyeball_position_l_y,
                  double param_calibration_eyeball_position_r_x,
                  double param_calibration_eyeball_position_r_y,
                  double param_calibration_eyeball_angle,
                  float param_calibration_eye_blink_time_quickly,
                  float param_calibration_eye_blink_time_min,
                  float param_calibration_eye_blink_time_max,
                  float param_calibration_eye_blink_time_limit);

    void loadEyelid(QString skin_name,
                    double param_calibration_l_x,
                    double param_calibration_l_y,
                    double param_calibration_l_angle,
                    double param_calibration_r_x,
                    double param_calibration_r_y,
                    double param_calibration_r_angle,
                    int param_calibration_eyelid_size_x,
                    int param_calibration_eyelid_size_y,
                    float param_calibration_eye_blink_time_quickly,
                    float param_calibration_eye_blink_time_min,
                    float param_calibration_eye_blink_time_max,
                    float param_calibration_eye_blink_time_limit);
    void loadEyeball(QString skin_name,
                     double param_calibration_l_angle,
                     double param_calibration_r_angle,
                     double param_calibration_eyeball_position_l_x,
                     double param_calibration_eyeball_position_l_y,
                     double param_calibration_eyeball_position_r_x,
                     double param_calibration_eyeball_position_r_y,
                     double param_calibration_eyeball_angle);

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
