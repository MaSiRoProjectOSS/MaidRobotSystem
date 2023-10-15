/**
 * @file st_eyeball.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-10-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "models/widgets/parts/st_eyeball.hpp"

namespace maid_robot_system
{
// =============================
// Constructor
// =============================
StEyeball::StEyeball()
{
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
void StEyeball::setting(int size_x, int size_y, StVector axis)
{
    now.set(0, 0, 0);
    target.set(0, 0, 0);
    center.set(axis.x, axis.y, 0);
    size.set(size_x, size_y, 0);
    draw_postion.set_size(size_x, size_y);
    size_cornea_anime.set(550, 550, 0);
}

void StEyeball::set_draw_pixel(int send_animation, double dimensions, double calibration_angle_cos, double calibration_angle_sin)
{
    wink_eye_up        = ((send_animation / 30.0) * size.y) / 27.0;
    double size_width  = dimensions * (size.x * (1.0 - (abs(now.x) / (size.x * 1.3))));
    double size_height = dimensions * (size.y * (1.0 - (abs(now.y) / (size.y * 1.3))));
    draw_postion.set_size(size_width * calibration_angle_cos + size_height * calibration_angle_sin, size_width * calibration_angle_sin + size_height * calibration_angle_cos);
    cornea.set(center.x + now.x, center.y + now.y, 0);
    draw_postion.set_axis(center.x + now.x - (draw_postion.width / 2.0), center.y + now.y - (draw_postion.height / 2.0) - wink_eye_up);
    eye_p_drive();
}

} // namespace maid_robot_system
