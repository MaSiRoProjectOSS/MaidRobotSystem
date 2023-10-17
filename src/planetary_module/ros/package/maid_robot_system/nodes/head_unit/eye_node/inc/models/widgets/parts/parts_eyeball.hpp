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
#include "st_eyeball.hpp"

#include <QPainter>
#include <QPixmap>
#include <cmath>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <vector>

namespace maid_robot_system
{
class PartsEyeball {
public:
    typedef enum ENUM_STATE
    {
        Normal,
        Receiving,

    } CorneaState;

public:
    // =============================
    // Constructor
    // =============================
    PartsEyeball();
    ~PartsEyeball();

public:
    // =============================
    // PUBLIC : Variable
    // =============================
    StEyeball left_eye;
    StEyeball right_eye;

public:
    // =============================
    // PUBLIC : Function
    // =============================
    void init();
    void closing();
    void calculate(int progress, int elapsed);
    void load(StParameter param);
    void update_background(QPainter &painter);
    void update_eyeball(QPainter &painter);
    void update_cornea_outside(QPainter &painter);
    void update_cornea_inside(QPainter &painter);

public:
    // =============================
    // PUBLIC : Setter
    // =============================
    void set_param(StParameter param, StVector left_eye_center, StVector right_eye_center);
    void set_dimensions(float value);
    void set_state_cornea(CorneaState state);
    void set_default();

private:
    // =============================
    // PRIVATE : Function
    // =============================
    void _drawing(ENUM_STATE state);
    int _get_index(ENUM_STATE state);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    QPixmap _blank{ 1, 1 };
    ENUM_STATE _request_cornea_state;
    float _request_dimensions = EYEBALL_DIMENSIONS_DEFAULT;
    float _dimensions         = EYEBALL_DIMENSIONS_DEFAULT;
};

} // namespace maid_robot_system

#endif
