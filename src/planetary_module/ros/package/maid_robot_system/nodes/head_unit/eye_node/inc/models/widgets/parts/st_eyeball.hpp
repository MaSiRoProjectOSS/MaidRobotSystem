/**
 * @file st_eyeball.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-10-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/common_structure.hpp"
#include "models/data_structure.hpp"

#include <QPainter>
#include <QPixmap>
#include <QRect>
#include <stdio.h>

namespace maid_robot_system
{
class StEyeball {
public:
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
        bool exit_vitreous       = false;
        bool exit_cornea_outside = false;
        bool exit_cornea_inside  = false;
    };

public:
    // =============================
    // Constructor
    // =============================
    StEyeball();
    ~StEyeball();

public:
    QRect rect_ciliary;
    QColor color;

    // =============================
    // PUBLIC : Variable
    // =============================
    QPixmap eyeball{ 1, 1 };
    QPixmap cornea_outside{ 1, 1 };
    QPixmap cornea_inside{ 1, 1 };
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

public:
    // =============================
    // PUBLIC : Function
    // =============================
    void eye_p_drive();

public:
    // =============================
    // PUBLIC : Setter
    // =============================
    void setting(int size_x, int size_y, StVector axis);
    void set_draw_pixel(int send_animation, double dimensions, double calibration_angle_cos, double calibration_angle_sin);
};
} // namespace maid_robot_system
