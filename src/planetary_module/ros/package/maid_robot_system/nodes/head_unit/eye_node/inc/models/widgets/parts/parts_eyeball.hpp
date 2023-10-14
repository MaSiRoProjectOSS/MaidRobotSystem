/**
 * @file parts_eyeball.hpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#ifndef MRS_EYE_NODE_MODELS_PARTS_EYEBALL_HPP
#define MRS_EYE_NODE_MODELS_PARTS_EYEBALL_HPP

#include "eye_node_settings.hpp"
#include "maid_robot_system/common_structure.hpp"
#include "math.h"
#include "models/calibration.hpp"
#include "models/data_structure.hpp"

#include <QPixmap>
#include <cmath>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <vector>

namespace maid_robot_system
{
class StEyeball {
public:
    QPixmap eyeball{ 1, 1 };
    QPixmap cornea_outside{ 1, 1 };
    QPixmap cornea_inside{ 1, 1 };
    class StImageStorage {
    public:
        std::vector<StImageMap> vitreous;
#if DRAW_CORNEA_OUTSIDE
        std::vector<StImageMap> cornea_outside;
        QMatrix matrix_cornea_outside;
#endif
#if DRAW_CORNEA_INSIDE
        std::vector<StImageMap> cornea_inside;
        QMatrix matrix_cornea_inside;
#endif
    };
    StImageStorage store;

    uint elapsed = 0;
    StVector pos;
    QPointF eyeball_center;
    double calibration_angle     = 0.0;
    double calibration_angle_cos = 0.0;
    double calibration_angle_sin = 0.0;

    StVector now;
    StVector target;
    StVector center;
    StVector size;
    StVector cornea;

    int ini_rotation;

    StVector draw_cornea_anime;
    StVector draw_cornea_anime2;
    StVector size_cornea_anime;
    StVector size_cornea_anime2;
    int cornea_outside_angle = 0;
    int cornea_inside_angle  = 0;

    int wink_eye_up = 0;

    StRectangle draw_postion;
    void setting(int size_x, int size_y, StVector axis)
    {
        now.set(0, 0, 0);
        target.set(0, 0, 0);
        center.set(axis.x, axis.y, 0);
        size.set(size_x, size_y, 0);
        draw_postion.set_size(size_x, size_y);
        size_cornea_anime.set(550, 550, 0);
    }

    void set_draw_pixel(int send_animation, double dimensions, double calibration_angle_cos, double calibration_angle_sin)
    {
        wink_eye_up        = ((send_animation / 30.0) * size.y) / 27.0;
        double size_width  = dimensions * (size.x * (1.0 - (abs(now.x) / (size.x * 1.3))));
        double size_height = dimensions * (size.y * (1.0 - (abs(now.y) / (size.y * 1.3))));
        draw_postion.set_size(size_width * calibration_angle_cos + size_height * calibration_angle_sin, size_width * calibration_angle_sin + size_height * calibration_angle_cos);
        cornea.set(center.x + now.x, center.y + now.y, 0);
        draw_postion.set_axis(center.x + now.x - (draw_postion.width / 2.0), center.y + now.y - (draw_postion.height / 2.0) - wink_eye_up);
        eye_p_drive();
    }

    void eye_p_drive()
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
};

class PartsEyeball {
public:
    typedef enum ENUM_STATE
    {
        Normal,
        Receiving,

    } CorneaState;

    /* ============================================= */
    PartsEyeball();
    void set_dimensions(float value);
    /* ============================================= */
    void set_state_cornea(CorneaState state);
    void load(StParameter param);
    void set_param(StParameter param);

    StEyeball left_eye;
    StEyeball right_eye;
    void calc_draw_pixel(int elapsed, int send_animation);
    void draw_outside();
    void draw_inside();

    void set_default();

private:
    void _set_image(StParameter param);

    QPixmap _blank{ 1, 1 };

private:
    /* ============================================= */
    /* ============================================= */
    int get_index();
    ENUM_STATE current_cornea_state;
    ENUM_STATE request_cornea_state;
    float request_dimensions = EYEBALL_DIMENSIONS_DEFAULT;
    float dimensions         = EYEBALL_DIMENSIONS_DEFAULT;
};

} // namespace maid_robot_system

#endif
