﻿/**
 * @file log_store.hpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#ifndef MRS_EYE_NODE_LOG_STORE_HPP
#define MRS_EYE_NODE_LOG_STORE_HPP

#include "maid_robot_system/common_structure.hpp"
#include "math.h"
#include "models/calibration.hpp"

#include <QPixmap>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>

#define EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX 30

namespace maid_robot_system
{
class LogStore {
public:
    /* ============================================= */
    LogStore();
    /* ============================================= */
    typedef enum
    {
        ST_INDEX_INIT,
        ST_INDEX_PRE_CALCULATION,
        ST_INDEX_MAKE_PUPIL_INSIDE,
        ST_INDEX_MAKE_PUPIL_OUTSIDE,
        ST_INDEX_DRAW_BACKGROUND,
        ST_INDEX_DRAW_EYEBALL,
        ST_INDEX_DRAW_PUPIL_INSIDE,
        ST_INDEX_DRAW_PUPIL_OUTSIDE,
        ST_INDEX_DRAW_EYELID,
        ST_INDEX_FIN,
        /* ======== */
        ST_INDEX_TOTAL,
        ST_INDEX_LOG_MAX
    } ST_INDEX_LOG;

    void set_index(ST_INDEX_LOG index, int spent_time);
    std::string get_message(std::string miens_text, double current_time_msec);

private:
    /* ============================================= */
    void _clear();
    int _spent_time_list[ST_INDEX_LOG_MAX];
    int _count = 0;
};

} // namespace maid_robot_system

#endif
