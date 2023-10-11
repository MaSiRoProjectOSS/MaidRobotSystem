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
void EyeWidget::reload_parameter(StParameter param, StRectangle screen_size)
{
    this->eyelid.load_eyelid(param, screen_size);

    param.eyeball_center_left.x  = this->eyelid.left.pos_center.x + param.eyeball_position_l_x;
    param.eyeball_center_left.y  = this->eyelid.left.pos_center.y + param.eyeball_position_l_y;
    param.eyeball_center_right.x = this->eyelid.right.pos_center.x + param.eyeball_position_r_x;
    param.eyeball_center_right.y = this->eyelid.right.pos_center.y + param.eyeball_position_r_y;

    this->eyeball.load_eyeball(param, screen_size);
}


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
    this->set_param(param);
    this->reload_parameter(param, screen_size);
    current_time.start();
    this->eyelid.set_emotion(miens_close);
    this->eyelid.calc_animation(EYE_BLINK_TIME_MILLISECOND_DEFAULT * -2);
    this->eyelid.calc_animation(0);
    this->eyelid.calc_animation(EYE_BLINK_TIME_MILLISECOND_DEFAULT);
    this->resize(screen_size.width, screen_size.height);
    last_ros_msg_time = current_time.elapsed();
    this->eyelid.calc_animation(current_time.elapsed());
}

void EyeWidget::closing()
{
    // TODO
    printf("\n==============\n CLOSE APP.\n==============\n");
}

} // namespace maid_robot_system
