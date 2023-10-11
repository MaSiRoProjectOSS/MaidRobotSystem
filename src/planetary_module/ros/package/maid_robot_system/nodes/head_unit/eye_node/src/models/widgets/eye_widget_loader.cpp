/**
 * @file eye_widget_loader.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

namespace maid_robot_system
{


void EyeWidget::initialize(StParameter param)
{
    // TODO
    /* --------------------------------------------------- */
    // register[START]
    printf("==== register[START] ====\n");
    /* --------------------------------------------------- */
    param.calibration_eyelid_size_x = param.eyelid_size_x;
    param.calibration_eyelid_size_y = param.eyelid_size_y;
    this->eyeball.Initialize(param.l_angle - param.eyeball_angle, param.r_angle + param.eyeball_angle);
    /* ============================================= */
    // set timer
    printf("    register - Timer\n");
    /* ============================================= */
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(1);
    qApp->installEventFilter(this);
    this->reload_param();
    current_time.start();
    this->eyelid.set_emotion(miens_close);
    this->eyelid.calc_animation(EYE_BLINK_TIME_MILLISECOND_DEFAULT * -2);
    this->eyelid.calc_animation(0);
    this->eyelid.calc_animation(EYE_BLINK_TIME_MILLISECOND_DEFAULT);
    this->resize(param.screen_size.width, param.screen_size.height);
    last_ros_msg_time = current_time.elapsed();
    this->eyelid.calc_animation(current_time.elapsed());
}

void EyeWidget::closing()
{
    // TODO
    printf("\n==============\n CLOSE APP.\n==============\n");
}

} // namespace maid_robot_system
