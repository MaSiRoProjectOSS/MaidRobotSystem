/**
 * @file st_eyeball.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-10-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "eye_node_settings.hpp"
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
        QTransform matrix_cornea_outside;
#endif
#if DRAW_CORNEA_INSIDE
        std::vector<StImageMap> cornea_inside;
        QTransform matrix_cornea_inside;
#endif
        bool exit_vitreous       = false;
        bool exit_cornea_outside = false;
        bool exit_cornea_inside  = false;

        QPixmap buf_cornea_outside{ 1, 1 };
        QPixmap buf_cornea_inside{ 1, 1 };
        StRectangle rect_cornea_outside;
        StRectangle rect_cornea_inside;
    };

public:
    // =============================
    // Constructor
    // =============================
    StEyeball();
    ~StEyeball();

public:
    QRect rect_ciliary;

    // =============================
    // PUBLIC : Variable
    // =============================
    QPixmap eyeball{ 1, 1 };
    QPixmap cornea_outside{ 1, 1 };
    QPixmap cornea_inside{ 1, 1 };
    QRectF target_cornea_outside;
    QRectF target_cornea_inside;
    QRectF source_cornea_outside;
    QRectF source_cornea_inside;
    StRectangle size_cornea_outside;
    StRectangle size_cornea_inside;

    double speed_min_cornea_outside   = 0;
    double speed_min_cornea_inside    = 0;
    double speed_max_cornea_outside   = 0;
    double speed_max_cornea_inside    = 0;
    StVector scale_cornea_outside = 0;
    StVector scale_cornea_inside  = 0;

    StImageStorage store;
    StVector eyeball_center;
    double calibration_angle     = 0.0;
    double calibration_angle_cos = 0.0;
    double calibration_angle_sin = 0.0;

    StVector now;
    StVector target;
    StVector center;
    StRectangle size;

    int wink_eye_up = 0;

    StRectangle draw_postion;

    double descend = 0.1;

public:
    // =============================
    // PUBLIC : Function
    // =============================
    void eye_p_drive();

public:
    // =============================
    // PUBLIC : Setter
    // =============================
    void setting(StParameter::StEyeSettings param, int center_x, int center_y);
    void set_draw_pixel(int progress, double dimensions, double calibration_angle_cos, double calibration_angle_sin);
};
} // namespace maid_robot_system
