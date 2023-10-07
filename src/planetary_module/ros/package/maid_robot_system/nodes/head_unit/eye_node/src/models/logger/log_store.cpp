/**
 * @file log_store.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "logger/log_store.hpp"

namespace maid_robot_system
{
LogStore::LogStore()
{
    clear();
}

void LogStore::clear()
{
    for (int i = 0; i < ST_INDEX_LOG_MAX; i++) {
        spent_time_list[i] = 0;
    }

    count       = 0;
    count_total = 0;
}

void LogStore::set_index(ST_INDEX_LOG index, int spent_time)
{
    if (ST_INDEX_LOG_MAX > index) {
        spent_time_list[(int)index] += spent_time;
    }

    if (ST_INDEX_FPS == index) {
        count++;
    }

    if (ST_INDEX_TOTAL == index) {
        count_total++;
    }
}
void LogStore::print(int eye_emotion, double total_spent_msec, double current_time_msec)
{
    if (0 < count) {
        double p_count    = (double)count;
        int p_count_total = count_total;
        float p_spent_time_list[ST_INDEX_LOG_MAX];

        for (int i = 0; i < ST_INDEX_LOG_MAX; i++) {
            p_spent_time_list[i] = spent_time_list[i] / p_count;
        }

        clear();
#if DEBUG_PRINT
        //********************************************************************************//
        printf("======================\n"
               "current : %7.3f\n"
               "spent   : %7.3f ms\n"
               "pupil - outside       [%7.3f ms]\n"
               "pupil - inside        [%7.3f ms]\n"
               "DRAW  - eyeball       [%7.3f ms]\n"
               "DRAW  - pupil outside [%7.3f ms]\n"
               "DRAW  - pupil inside  [%7.3f ms]\n"
               "DRAW  - eyelid        [%7.3f ms]\n"
               "Request : Total[%d] draw[%d] FPS[%7.3f]\n"
               " -> Emotion: %d\n",
               current_time_msec,
               total_spent_msec,
               p_spent_time_list[ST_INDEX_MAKE_PUPIL_OUTSIDE],
               p_spent_time_list[ST_INDEX_MAKE_PUPIL_INSIDE],
               p_spent_time_list[ST_INDEX_DRAW_EYEBALL],
               p_spent_time_list[ST_INDEX_DRAW_PUPIL_INSIDE],
               p_spent_time_list[ST_INDEX_DRAW_PUPIL_OUTSIDE],
               p_spent_time_list[ST_INDEX_DRAW_EYELID],
               p_count_total,
               (int)p_count,
               (p_count * 1000.0) / total_spent_msec,
               (int)eye_emotion);
#endif
        printf("FPS is %7.3f [%8.3f ms]\n", (p_count * 1000.0) / total_spent_msec, p_spent_time_list[ST_INDEX_FPS]);
    }
}

} // namespace maid_robot_system
