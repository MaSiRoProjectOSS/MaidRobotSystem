/**
 * @file st_eyeball.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-10-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "widgets/parts/st_eyeball.hpp"

namespace maid_robot_system
{
// =============================
// Constructor
// =============================
StEyeball::StEyeball()
{
    this->rect_ciliary.setRect(0, 0, 100, 100);
#if DRAW_CORNEA_OUTSIDE
    this->store.matrix_cornea_outside.reset();
#endif
#if DRAW_CORNEA_INSIDE
    this->store.matrix_cornea_inside.reset();
#endif
}
StEyeball::~StEyeball()
{
}

// =============================
// PUBLIC : Function
// =============================
void StEyeball::eye_p_drive()
{
    if (target.x != now.x) {
        now.x += (target.x - now.x) * 0.4;
    }

    if (target.y != now.y) {
        now.y += (target.y - now.y) * 0.4;
    }

    // cornea_outside_angle
    cornea_outside_angle++;

    if (360 <= cornea_outside_angle) {
        cornea_outside_angle = cornea_outside_angle - 360;
    }

    if (0 > cornea_outside_angle) {
        cornea_outside_angle = cornea_outside_angle + 360;
    }

    // cornea_inside_angle
    cornea_inside_angle -= func_rand(1, 3);

    if (360 <= cornea_inside_angle) {
        cornea_inside_angle = cornea_inside_angle - 360;
    }

    if (0 > cornea_inside_angle) {
        cornea_inside_angle = cornea_inside_angle + 360;
    }
}

// =============================
// PUBLIC : Setter
// =============================
void StEyeball::setting(StParameter::StEyeSettings param, int center_x, int center_y)
{
    this->rect_ciliary.setWidth(param.eyelid.width);
    this->rect_ciliary.setHeight(param.eyelid.height);

    this->calibration_angle     = param.eyeball.angle * (M_PI / 180);
    this->calibration_angle_cos = std::abs(std::cos(this->calibration_angle));
    this->calibration_angle_sin = std::abs(std::sin(this->calibration_angle));

    now.set(0, 0, 0);
    target.set(0, 0, 0);
    center.set(center_x, center_y, 0);
    size.set_size(param.eyeball.width, param.eyeball.height);
    draw_postion.set_size(param.eyeball.width, param.eyeball.height);
    size_cornea_anime.set(550, 550, 0);
}

void StEyeball::set_draw_pixel(int progress, double dimensions, double calibration_angle_cos, double calibration_angle_sin)
{
    // the eyeballs also descend when close the eyelids
    wink_eye_up = (((std::abs(progress) - 100.0) / 100.0) * this->size.height) * this->descend;

    double size_width  = dimensions * (this->size.width * (1.0 - (abs(now.x) / (this->size.width * 1.3))));
    double size_height = dimensions * (this->size.height * (1.0 - (abs(now.y) / (this->size.height * 1.3))));
    draw_postion.set_size(size_width * calibration_angle_cos + size_height * calibration_angle_sin, size_width * calibration_angle_sin + size_height * calibration_angle_cos);
    cornea.set(center.x + now.x, center.y + now.y, 0);
    draw_postion.set_axis(center.x + now.x - (draw_postion.width / 2.0), center.y + now.y - (draw_postion.height / 2.0) - wink_eye_up);
    eye_p_drive();
}

} // namespace maid_robot_system
