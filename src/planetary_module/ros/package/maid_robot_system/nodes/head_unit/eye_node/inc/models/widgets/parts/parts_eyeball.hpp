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

namespace maid_robot_system
{
struct StEyeball {
    typedef struct {
        uint elapsed = 0;
        StVector pos;
    } eyeball;

    StVector now;
    StVector target;
    StVector center;
    StVector size;
    StVector pupil;

    int ini_rotation;

    StVector draw_pupil_anime;
    StVector draw_pupil_anime2;
    StVector size_pupil_anime;
    StVector size_pupil_anime2;
    int pupil_outside_angle = 0;
    int pupil_inside_angle  = 0;

    int wink_eye_up = 0;

    StRectangle draw_postion;
    void setting(int size_x, int size_y, StVector axis)
    {
        now.set(0, 0);
        target.set(0, 0);
        center.set(axis.x, axis.y);
        size.set(size_x, size_y);
        draw_postion.set_size(size_x, size_y);
        size_pupil_anime.set(550, 550);
    }

    void set_draw_pixel(int send_animation, double dimensions, double calibration_angle_cos, double calibration_angle_sin)
    {
        wink_eye_up        = ((send_animation / 30.0) * size.y) / 27.0;
        double size_width  = dimensions * (size.x * (1.0 - (abs(now.x) / (size.x * 1.3))));
        double size_height = dimensions * (size.y * (1.0 - (abs(now.y) / (size.y * 1.3))));
        draw_postion.set_size(size_width * calibration_angle_cos + size_height * calibration_angle_sin, size_width * calibration_angle_sin + size_height * calibration_angle_cos);
        pupil.set(center.x + now.x, center.y + now.y);
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

        // pupil_outside_angle
        pupil_outside_angle++;

        if (360 <= pupil_outside_angle) {
            pupil_outside_angle = pupil_outside_angle - 360;
        }

        if (0 > pupil_outside_angle) {
            pupil_outside_angle = pupil_outside_angle + 360;
        }

        // pupil_inside_angle
        pupil_inside_angle -= func_rand(1, 3);

        if (360 <= pupil_inside_angle) {
            pupil_inside_angle = pupil_inside_angle - 360;
        }

        if (0 > pupil_inside_angle) {
            pupil_inside_angle = pupil_inside_angle + 360;
        }
    }
};

class PartsEyeball {
public:
    typedef enum ENUM_STATE
    {
        Normal,
        Receiving,

    } PupilState;

    /* ============================================= */
    PartsEyeball();
    void set_dimensions(float value);
    /* ============================================= */
    void set_state_pupil(PupilState state);
    void loadPupil(QString skin_name, Qt::ImageConversionFlag imageFlag);

    StEyeball left;
    StEyeball right;
    void calc_draw_pixel(int elapsed, int send_animation);
    void draw_outside();
    void draw_inside();

    void set_default();
    void Initialize(double param_calibration_l_angle, double param_calibration_r_angle);

    /* ============================================= */
    // QMatrix matrix_eyeball;
    QPixmap eyeball_origin_l;
    QPixmap eyeball_origin_r;
#if DRAW_PUPIL_OUTSIDE
    QPixmap pupil_outside;
#endif
#if DRAW_PUPIL_INSIDE
    QPixmap pupil_inside;
#endif
    /* ============================================= */
private:
    /* ============================================= */
#if DRAW_PUPIL_OUTSIDE
    QMatrix matrix_pupil_outside;
    QPixmap pupil_outside_origin[2];
    // int pupil_outside_select = 0;
    double pupil_ling_size_outside = PUPIL_LING_SIZE_OUTSIDE;
#endif
#if DRAW_PUPIL_INSIDE
    QMatrix matrix_pupil_inside;
    QPixmap pupil_inside_origin[2];
    // int pupil_inside_select = 0;
    double pupil_ling_size_inside = PUPIL_LING_SIZE_INSIDE;
#endif
    /* ============================================= */
    int get_index();
    ENUM_STATE current_pupil_state;
    ENUM_STATE request_pupil_state;
    double calibration_l_angle     = 0.0;
    double calibration_r_angle     = 0.0;
    double calibration_l_angle_cos = 0.0;
    double calibration_l_angle_sin = 0.0;
    double calibration_r_angle_cos = 0.0;
    double calibration_r_angle_sin = 0.0;
    QPointF eyeball_center_right;
    QPointF eyeball_center_left;
    float request_dimensions = EYEBALL_DIMENSIONS_DEFAULT;
    float dimensions         = EYEBALL_DIMENSIONS_DEFAULT;
};

} // namespace maid_robot_system

#endif
