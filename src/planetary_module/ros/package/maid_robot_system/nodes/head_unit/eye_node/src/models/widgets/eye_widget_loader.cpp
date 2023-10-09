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

EyeWidget::EyeWidget(QWidget *parent) : QOpenGLWidget(parent), eyeball(), eyelid(), logger()
{
}
EyeWidget::~EyeWidget()
{
}

/**
 * @brief
 *
 * @param value
 */
void EyeWidget::loadSkin(StParameter param)
{
    printf("  ---- LOAD [Skin] ----\n");
    printf("    register - ROS\n");
    printf("        SKIN NAME : %s\n", param.name.c_str());
    this->eyeball.loadPupil(param.name, this->imageFlag);
    this->loadEyelid(param);
    this->loadEyeball(param);
}

/**
 * @brief
 *
 * @param skin_name
 */
void EyeWidget::loadEyelid(StParameter param)
{
    printf("  ---- LOAD [Eyelid] ----\n");
    this->eyelid.loadSkin(param);
    StVector calibration_right = StVector(param.r_x, param.r_y);
    StVector calibration_left  = StVector(param.l_x, param.l_y);
    this->eyelid.setting(this->window_size_x,
                         this->window_size_y, //
                         param.eyelid_size_x,
                         param.eyelid_size_y,
                         calibration_right,
                         calibration_left);
#if DEBUG_OUTPUT_WIDGET
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
void EyeWidget::loadEyeball(StParameter param)
{
    printf("  ---- LOAD [Eyeball] ----\n");
    const QString str_eyeball_origin = "eyeball/eye_all.png";
    ////////////////
    // eyeball_origin_l
    QMatrix matrix_eyeball_l;
    matrix_eyeball_l.rotate(-param.l_angle - param.eyeball_angle);
    eyeball.eyeball_origin_l = QPixmap(str_eyeball_origin, nullptr, imageFlag);
    eyeball.eyeball_origin_l = eyeball.eyeball_origin_l.scaled(this->eyeball_size_x, this->eyeball_size_y, Qt::IgnoreAspectRatio);
    eyeball.eyeball_origin_l = eyeball.eyeball_origin_l.transformed(matrix_eyeball_l);
    // eyeball_origin
    QMatrix matrix_eyeball_r;
    matrix_eyeball_r.rotate(-param.r_angle + param.eyeball_angle);
    eyeball.eyeball_origin_r = QPixmap(str_eyeball_origin, nullptr, imageFlag);
    eyeball.eyeball_origin_r = eyeball.eyeball_origin_r.scaled(this->eyeball_size_x, this->eyeball_size_y, Qt::IgnoreAspectRatio);
    eyeball.eyeball_origin_r = eyeball.eyeball_origin_r.transformed(matrix_eyeball_r);
    ////////////////
    StVector eyeball_center_left  = StVector(this->eyelid.left.pos_center.x + param.eyeball_position_l_x, //
                                            this->eyelid.left.pos_center.y + param.eyeball_position_l_y);
    StVector eyeball_center_right = StVector(this->eyelid.right.pos_center.x + param.eyeball_position_r_x, //
                                             this->eyelid.right.pos_center.y + param.eyeball_position_r_y);
    //////////////////////////////
    //////////////////////////////
    eyeball.right.setting(this->eyeball_size_x, this->eyeball_size_y, eyeball_center_right);
    eyeball.left.setting(this->eyeball_size_x, this->eyeball_size_y, eyeball_center_left);
#if DEBUG_OUTPUT_WIDGET
    printf("============== Eyeball center ===============\n");
    printf(" Right (x,y) = (%f,%f)\n", eyeball_center_right.x, eyeball_center_right.y);
    printf(" Left (x,y) = (%f,%f)\n", eyeball_center_left.x, eyeball_center_left.y);
    printf("=============================================\n");
    printf("------------ Calibration Postion ------------\n");
    printf(" Size (x,y) = (%d,%d)\n", this->calibration_eyelid_size_x, this->calibration_eyelid_size_y);
    printf(" Pos r(x,y) = (%f,%f)\n", param.eyeball_position_l_x, param.eyeball_position_l_y);
    printf(" Pos l(x,y) = (%f,%f)\n", param.eyeball_position_r_x, param.eyeball_position_r_y);
    printf("---------------------------------------------\n");
#endif
    /* ============================================= */
}

void EyeWidget::set_param(StParameter param)
{
}

void EyeWidget::initialize(StParameter param)
{
    /* --------------------------------------------------- */
    // register[START]
    printf("==== register[START] ====\n");
    /* --------------------------------------------------- */
    this->calibration_eyelid_size_x = param.eyelid_size_x;
    this->calibration_eyelid_size_y = param.eyelid_size_y;
    this->eyeball.Initialize(param.l_angle - param.eyeball_angle, param.r_angle + param.eyeball_angle);
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
    this->Setup(param.eye_blink_time_offset);
    this->loadSkin(param);
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
