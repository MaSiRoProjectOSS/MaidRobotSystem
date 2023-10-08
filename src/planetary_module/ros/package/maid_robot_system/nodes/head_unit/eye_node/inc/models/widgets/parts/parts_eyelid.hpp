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

namespace maid_robot_system
{
#define EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX 30

/**
 * @brief
 *
 */
class StEyelid {
    /* ============================================= */
public:
    /* ============================================= */
    void debug_old_calc(uint current_time);

    StEyelid();
    uint get_elapsed();

    void open_eye(uint wink_time_millisecond);

    void wink(uint wink_time_millisecond);

    void set_elapsed(uint current_time);
    bool enable_motion();
    /* ============================================= */
    StVector pos;
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
    PartsEyelid();
    void loadSkin(StParameter param);
    void setup(float param_calibration_eye_blink_time_offset);
    /* ============================================= */
    QPixmap get_eye_id(ENUM_TARGET_EYE target);

    double set_eye_blink_time(blink_type type);
    void set_eye_blink(blink_type eye_emotion, bool start_flag);
    void set_emotion(MIENS eye_emotion);
    void set_cycle(uint elapsed);

    void set_onTheWay();
    void cycle();

    bool enable_motion();

    void setting(double window_size_x, double window_size_y, int size_x, int size_y, StVector right_axis, StVector left_axis);

    uint get_ms_time(int time_current, int time_check, int add_Value);
    /* ============================================= */
    StEyelid left;
    StEyelid right;

    int send_animation     = 0;
    int lib_animation      = 0;
    int lib_animation_flag = 0;

    bool thinking = false;
    int qt_wink_anime_start_time;

    bool flag_EmotionKeep  = false;
    MIENS eye_emotion      = miens_close;
    MIENS next_eye_emotion = miens_close;
    void calc_animation(int elapsed);

    float eye_blink_time = EYE_BLINK_TIME_MILLISECOND_DEFAULT_MAX; // ms
private:
    /* ============================================= */
    typedef enum
    {
        INDEX_LIP_NORMAL,
        INDEX_LIP_SMILE,
    } INDEX_LIP_IMAGE;
    /* ============================================= */
    float eye_blink_time_quickly = EYE_BLINK_TIME_MILLISECOND_DEFAULT_QUICKLY; // ms
    float eye_blink_time_min     = EYE_BLINK_TIME_MILLISECOND_DEFAULT_MIN;     // ms
    float eye_blink_time_max     = EYE_BLINK_TIME_MILLISECOND_DEFAULT_MAX;     // ms
    float eye_blink_time_limit   = EYE_BLINK_TIME_MILLISECOND_DEFAULT_LIMITED; // ms
    float eye_blink_time_offset  = EYE_BLINK_TIME_MILLISECOND_DEFAULT_OFFSET;
    /* ============================================= */
    int elapsed_next = 0;
    /* ============================================= */
    QPixmap L_eyelid[EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX];
    QPixmap R_eyelid[EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX];
    QPixmap L_smile_lid[EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX];
    QPixmap R_smile_lid[EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX];
    /* ============================================= */
    float winkValue = 3.0;
};

} // namespace maid_robot_system

#endif
