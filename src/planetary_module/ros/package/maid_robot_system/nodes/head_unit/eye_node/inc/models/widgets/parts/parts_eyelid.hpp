/**
 * @file parts_eyelid.hpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#ifndef MRS_EYE_NODE_MODELS_PARTS_EYELID_HPP
#define MRS_EYE_NODE_MODELS_PARTS_EYELID_HPP

#include "eye_node_settings.hpp"
#include "maid_robot_system/common_structure.hpp"
#include "maid_robot_system/emotion.hpp"
#include "math.h"
#include "models/calibration.hpp"
#include "models/data_structure.hpp"
#include "st_eyelid.hpp"

#include <QPixmap>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <vector>

namespace maid_robot_system
{
/**
 * @brief
 *
 */
class PartsEyelid {
private:
    typedef enum
    {
        INDEX_LIP_NORMAL = 0,
        INDEX_LIP_SMILE,
    } INDEX_LIP_IMAGE;

public:
    typedef enum ENUM_BLINK_TYPE
    {
        BLINK_TYPE_MIN_MAX,
        BLINK_TYPE_LONG,
        BLINK_TYPE_QUICKLY,

    } blink_type;

public:
    // =============================
    // Constructor
    // =============================
    PartsEyelid();
    ~PartsEyelid();

public:
    // =============================
    // PUBLIC : Variable
    // =============================
    QColor color;
    StEyelid left_eye;
    StEyelid right_eye;

    MIENS eye_emotion = miens_close;

public:
    // =============================
    // PUBLIC : Function
    // =============================
    int calc_animation(int elapsed);
    void load(StParameter param);

public:
    // =============================
    // PUBLIC : Setter
    // =============================
    void set_param(StParameter param);
    void set_eye_blink(blink_type eye_emotion, bool start_flag);
    double set_eye_blink_time(blink_type type);
    void set_emotion(MIENS eye_emotion);
    void set_on_the_way();

public:
    // =============================
    // PUBLIC : Getter
    // =============================
    bool enable_motion();
    QPixmap get_eye_id_right();
    QPixmap get_eye_id_left();
    uint get_ms_time(int time_current, int time_check, int add_Value);

private:
    // =============================
    // PRIVATE : Function
    // =============================
    QPixmap _get_eye_id(bool is_left);
    void _reset_position(StParameter param);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    QPixmap _blank{ 1, 1 };

    int _send_animation = 0;
    int _elapsed_next   = 0;
    float _wink_value   = 3.0;

    float _eye_blink_time_quickly = 150.0f;            // ms
    float _eye_blink_time_min     = (500.0f - 100.0f); // ms
    float _eye_blink_time_max     = (500.0f + 100.0f); // ms
    float _eye_blink_time_limit   = 15000.0f;          // ms
    float _eye_blink_time_offset  = 0.0f;
    float _eye_blink_time         = (500.0f + 100.0f); // ms

    MIENS _next_eye_emotion = miens_close;

    int _lib_animation     = 0;
    bool _thinking         = false;
    bool _flag_EmotionKeep = false;
};

} // namespace maid_robot_system

#endif
