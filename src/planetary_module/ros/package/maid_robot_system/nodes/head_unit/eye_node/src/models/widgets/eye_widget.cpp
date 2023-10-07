/**
 * @file eye_widget.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

namespace maid_robot_system
{
/**
 @brief Construct a new Main Window:: Main Window object

 @param parent
*/

#if SET_GLWIDGET
EyeWidget::EyeWidget(QWidget *parent) : QGLWidget(parent), eyeball(), eyelid(), logger()
{
}
#else
EyeWidget::EyeWidget(QWidget *parent) : QWidget(parent), eyeball(), eyelid(), logger()
{
}
#endif
EyeWidget::~EyeWidget()
{
}

/**
 * @brief
 *
 * @param value
 */
void EyeWidget::loadSkin(QString skin_name,
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
                         float param_calibration_eye_blink_time_limit)
{
    printf("  ---- LOAD [Skin] ----\n");
    printf("    register - ROS\n");
    printf("        SKIN NAME : %s\n", skin_name.toStdString().c_str());
    this->eyeball.loadPupil(skin_name + "/", this->imageFlag);
    this->loadEyelid(skin_name + "/",
                     param_calibration_l_x,
                     param_calibration_l_y,
                     param_calibration_l_angle,
                     param_calibration_r_x,
                     param_calibration_r_y,
                     param_calibration_r_angle,
                     param_calibration_eyelid_size_x,
                     param_calibration_eyelid_size_y,
                     param_calibration_eye_blink_time_quickly,
                     param_calibration_eye_blink_time_min,
                     param_calibration_eye_blink_time_max,
                     param_calibration_eye_blink_time_limit);
    this->loadEyeball(skin_name + "/",
                      param_calibration_l_angle,
                      param_calibration_r_angle,
                      param_calibration_eyeball_position_l_x,
                      param_calibration_eyeball_position_l_y,
                      param_calibration_eyeball_position_r_x,
                      param_calibration_eyeball_position_r_y,
                      param_calibration_eyeball_angle);
}

/**
 * @brief
 *
 * @param skin_name
 */
void EyeWidget::loadEyelid(QString skin_name,
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
                           float param_calibration_eye_blink_time_limit)
{
    printf("  ---- LOAD [Eyelid] ----\n");
    this->eyelid.loadSkin(skin_name,
                          param_calibration_eyelid_size_x,
                          param_calibration_eyelid_size_y,
                          param_calibration_r_angle,
                          param_calibration_l_angle,
                          param_calibration_eye_blink_time_quickly,
                          param_calibration_eye_blink_time_min,
                          param_calibration_eye_blink_time_max,
                          param_calibration_eye_blink_time_limit);
    StVector calibration_right = StVector(param_calibration_r_x, param_calibration_r_y);
    StVector calibration_left  = StVector(param_calibration_l_x, param_calibration_l_y);
    this->eyelid.setting(this->window_size_x, this->window_size_y, param_calibration_eyelid_size_x, param_calibration_eyelid_size_y, calibration_right, calibration_left);
#if DEBUG_PRINT
    printf("=============== Eyelid Postion ==============\n");
    printf(" Right (x,y) = (%f,%f)\n", this->eyelid.right.pos.x, this->eyelid.right.pos.y);
    printf(" Left (x,y) = (%f,%f)\n", this->eyelid.left.pos.x, this->eyelid.left.pos.y);
    printf("=============================================\n");
#endif
    ////////////////
}

void EyeWidget::Setup(float param_calibration_eye_blink_time_offset)
{
    this->eyelid.setup(param_calibration_eye_blink_time_offset);
}

/**
 * @brief
 *
 * @param skin_name
 */
void EyeWidget::loadEyeball(QString skin_name,
                            double param_calibration_l_angle,
                            double param_calibration_r_angle,
                            double param_calibration_eyeball_position_l_x,
                            double param_calibration_eyeball_position_l_y,
                            double param_calibration_eyeball_position_r_x,
                            double param_calibration_eyeball_position_r_y,
                            double param_calibration_eyeball_angle)
{
    printf("  ---- LOAD [Eyeball] ----\n");
    const QString str_eyeball_origin = "eyeball/eye_all.png";
    ////////////////
    // eyeball_origin_l
    QMatrix matrix_eyeball_l;
    matrix_eyeball_l.rotate(-param_calibration_l_angle - param_calibration_eyeball_angle);
    eyeball.eyeball_origin_l = QPixmap(str_eyeball_origin, nullptr, imageFlag);
    eyeball.eyeball_origin_l = eyeball.eyeball_origin_l.scaled(this->eyeball_size_x, this->eyeball_size_y, Qt::IgnoreAspectRatio);
    eyeball.eyeball_origin_l = eyeball.eyeball_origin_l.transformed(matrix_eyeball_l);
    // eyeball_origin
    QMatrix matrix_eyeball_r;
    matrix_eyeball_r.rotate(-param_calibration_r_angle + param_calibration_eyeball_angle);
    eyeball.eyeball_origin_r = QPixmap(str_eyeball_origin, nullptr, imageFlag);
    eyeball.eyeball_origin_r = eyeball.eyeball_origin_r.scaled(this->eyeball_size_x, this->eyeball_size_y, Qt::IgnoreAspectRatio);
    eyeball.eyeball_origin_r = eyeball.eyeball_origin_r.transformed(matrix_eyeball_r);
    ////////////////
    StVector eyeball_center_left
            = StVector(this->eyelid.left.pos_center.x + param_calibration_eyeball_position_l_x, this->eyelid.left.pos_center.y + param_calibration_eyeball_position_l_y);
    StVector eyeball_center_right
            = StVector(this->eyelid.right.pos_center.x + param_calibration_eyeball_position_r_x, this->eyelid.right.pos_center.y + param_calibration_eyeball_position_r_y);
    //////////////////////////////
    //////////////////////////////
    eyeball.right.setting(this->eyeball_size_x, this->eyeball_size_y, eyeball_center_right);
    eyeball.left.setting(this->eyeball_size_x, this->eyeball_size_y, eyeball_center_left);
#if DEBUG_PRINT
    printf("============== Eyeball center ===============\n");
    printf(" Right (x,y) = (%f,%f)\n", eyeball_center_right.x, eyeball_center_right.y);
    printf(" Left (x,y) = (%f,%f)\n", eyeball_center_left.x, eyeball_center_left.y);
    printf("=============================================\n");
    printf("------------ Calibration Postion ------------\n");
    printf(" Size (x,y) = (%d,%d)\n", this->calibration_eyelid_size_x, this->calibration_eyelid_size_y);
    printf(" Pos r(x,y) = (%f,%f)\n", param_calibration_eyeball_position_l_x, param_calibration_eyeball_position_l_y);
    printf(" Pos l(x,y) = (%f,%f)\n", param_calibration_eyeball_position_r_x, param_calibration_eyeball_position_r_y);
    printf("---------------------------------------------\n");
#endif
    /* ============================================= */
}

/**
 * @brief 初期化関数。起動時に使用する画像を読み込む
 *
 * @param param_calibration_l_x
 * @param param_calibration_l_y
 * @param param_calibration_r_x
 * @param param_calibration_r_y
 * @param param_eyeball_position_x
 * @param param_eyeball_position_y
 */
void EyeWidget::Initialize(QString skin_name,
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
                           float param_calibration_eye_blink_time_offset)
{
    /* --------------------------------------------------- */
    // register[START]
    printf("==== register[START] ====\n");
    /* --------------------------------------------------- */
    this->calibration_eyelid_size_x = param_calibration_eyelid_size_x;
    this->calibration_eyelid_size_y = param_calibration_eyelid_size_y;
    this->eyeball.Initialize(param_calibration_l_angle - param_calibration_eyeball_angle, param_calibration_r_angle + param_calibration_eyeball_angle);
    /* ============================================= */
    // set timer
    printf("    register - Timer\n");
    /* ============================================= */
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(1);
    qApp->installEventFilter(this);
    /* ============================================= */
    // Initial position
    printf("    register - Initial position\n");
    /* ============================================= */
    this->Setup(param_calibration_eye_blink_time_offset);
    this->loadSkin(skin_name,
                   param_calibration_l_x,
                   param_calibration_l_y,
                   param_calibration_l_angle,
                   param_calibration_r_x,
                   param_calibration_r_y,
                   param_calibration_r_angle,
                   param_calibration_eyelid_size_x,
                   param_calibration_eyelid_size_y,
                   param_calibration_eyeball_position_l_x,
                   param_calibration_eyeball_position_l_y,
                   param_calibration_eyeball_position_r_x,
                   param_calibration_eyeball_position_r_y,
                   param_calibration_eyeball_angle,
                   param_calibration_eye_blink_time_quickly,
                   param_calibration_eye_blink_time_min,
                   param_calibration_eye_blink_time_max,
                   param_calibration_eye_blink_time_limit);
    //////////////
    // smile anime pre load///////////////////////////////////////////////
    printf("    register - Buffering image for other\n");
    /* ============================================= */
    /* ============================================= */
    // smile anime pre load///////////////////////////////////////////////
    printf("    register - Start window\n");
    /* ============================================= */
    current_time.start();
    log_timer = current_time.elapsed();
    /* ============================================= */
    printf("    register - Initialize\n");
    /* ============================================= */
    this->eyelid.set_emotion(miens_close);
    this->eyelid.calc_animation(EYE_BLINK_TIME_MILLISECOND_DEFAULT * -2);
    this->eyelid.calc_animation(0);
    this->eyelid.calc_animation(EYE_BLINK_TIME_MILLISECOND_DEFAULT);
    this->resize(this->window_size_x, this->window_size_y);
#ifdef _WIN32
    Sleep(EYE_BLINK_TIME_MILLISECOND_DEFAULT);
#else
    //usleep(EYE_BLINK_TIME_MILLISECOND_DEFAULT * 1000);
#endif
    last_ros_msg_time = current_time.elapsed();
    this->eyelid.calc_animation(current_time.elapsed());
    /* --------------------------------------------------- */
    // register[END]
    printf("==== register[END] ====\n");
    /* --------------------------------------------------- */
}

/**
 * @brief 終了処理
 *
 */
void EyeWidget::Closing()
{
    printf("\n==============\n CLOSE APP.\n==============\n");

    // close();
    exit(0);
}

} // namespace maid_robot_system
