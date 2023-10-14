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

#include <QPixmap>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <vector>

namespace maid_robot_system
{
class StEyelid {
public:
    std::vector<StImageMap> store;

    /* ============================================= */

    StEyelid();
    uint get_elapsed();

    void open_eye(uint wink_time_millisecond);

    void wink(uint wink_time_millisecond);

    void set_elapsed(uint current_time);
    bool enable_motion();
    /* ============================================= */
    StRectangle pos;
    StVector pos_center;
    /* ============================================= */

private:
    bool _flag_next_blink       = false;
    uint _eye_blink_time        = 0;
    uint _elapsedIndex          = 0;
    uint _start_time            = 0xFFFFFFFF;
    uint _wink_time_millisecond = 0;
    bool _wink                  = false;
    uint _current_time          = 0;
    void fin_wink();
};

/**
 * @brief
 *
 */
class PartsEyelid {
public:
    typedef enum ENUM_BLINK_TYPE
    {
        BLINK_TYPE_MIN_MAX,
        BLINK_TYPE_LONG,
        BLINK_TYPE_QUICKLY,

    } blink_type;
    /* ============================================= */

public:
    PartsEyelid();

public:
    QColor color;

private:
    QPixmap _get_eye_id(bool is_left);

public:
    void load(StParameter param);
    void set_param(StParameter param);

    QPixmap get_eye_id_right();
    QPixmap get_eye_id_left();

    double set_eye_blink_time(blink_type type);
    void set_eye_blink(blink_type eye_emotion, bool start_flag);
    void set_emotion(MIENS eye_emotion);
    void set_cycle(uint elapsed);

    void set_on_the_way();
    void cycle();

    bool enable_motion();

    uint get_ms_time(int time_current, int time_check, int add_Value);

private:
    void _set_image(StParameter param);
    void _reset_position(StParameter param);

public:
    /* ============================================= */
    StEyelid left_eye;
    StEyelid right_eye;
    QPixmap _blank{ 1, 1 };

    int send_animation     = 0;
    int lib_animation      = 0;
    int lib_animation_flag = 0;

    bool thinking = false;
    int qt_wink_anime_start_time;

    bool flag_EmotionKeep  = false;
    MIENS eye_emotion      = miens_close;
    MIENS next_eye_emotion = miens_close;
    void calc_animation(int elapsed);

    float eye_blink_time = (500.0f + 100.0f); // ms
private:
    /* ============================================= */
    typedef enum
    {
        INDEX_LIP_NORMAL = 0,
        INDEX_LIP_SMILE,
    } INDEX_LIP_IMAGE;
    /* ============================================= */
    float eye_blink_time_quickly = 150.0f;            // ms
    float eye_blink_time_min     = (500.0f - 100.0f); // ms
    float eye_blink_time_max     = (500.0f + 100.0f); // ms
    float eye_blink_time_limit   = 15000.0f;          // ms
    float eye_blink_time_offset  = 0.0f;
    /* ============================================= */
    int elapsed_next = 0;
    /* ============================================= */
    /* ============================================= */
    float winkValue = 3.0;
};

} // namespace maid_robot_system

#endif
