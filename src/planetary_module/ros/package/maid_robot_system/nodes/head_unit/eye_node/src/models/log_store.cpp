/**
 * @file log_store.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/log_store.hpp"

#include "eye_node_settings.hpp"

namespace maid_robot_system
{
LogStore::LogStore()
{
    this->_clear();
}

void LogStore::_clear()
{
    for (int i = 0; i < ST_INDEX_LOG::ST_INDEX_LOG_MAX; i++) {
        this->_spent_time_list[i] = 0;
    }
    this->_count = 0;
}

void LogStore::set_index(ST_INDEX_LOG index, int spent_time)
{
    if (ST_INDEX_LOG::ST_INDEX_LOG_MAX > index) {
        this->_spent_time_list[(int)index] += spent_time;
    }

    if (ST_INDEX_LOG::ST_INDEX_TOTAL == index) {
        this->_count++;
    }
}
std::string LogStore::get_message(std::string miens_text, double current_time_msec)
{
    std::string result    = "";
    static double elapsed = current_time_msec;
    if (0 < this->_count) {
        char buffer[1024];
        double total_spent_msec = (current_time_msec - elapsed);
        double p_count          = (double)this->_count;
        float p_spent_time_list[ST_INDEX_LOG::ST_INDEX_LOG_MAX];
        for (int i = 0; i < ST_INDEX_LOG::ST_INDEX_LOG_MAX; i++) {
            p_spent_time_list[i] = this->_spent_time_list[i] / p_count;
        }
        this->_clear();
        if (0 < total_spent_msec) {
#if DEBUG_OUTPUT_FPS_VERBOSE
            result.append("======================\n");
            sprintf(buffer,
                    "  elapsed : %12.3f s\n"
                    "  spent   : %12.3f s\n",
                    (current_time_msec / 1000.0),
                    (total_spent_msec / 1000.0));
            result.append(buffer);

            sprintf(buffer,
                    "  Init                 [%7.3f ms]\n"
                    "  Calculate            [%7.3f ms]\n"
                    "  ROUTE- pupil_outside [%7.3f ms]\n"
                    "  ROUTE- pupil_inside  [%7.3f ms]\n"
                    "  DRAW - background    [%7.3f ms]\n"
                    "  DRAW - eyeball       [%7.3f ms]\n"
                    "  DRAW - pupil outside [%7.3f ms]\n"
                    "  DRAW - pupil inside  [%7.3f ms]\n"
                    "  DRAW - eyelid        [%7.3f ms]\n"
                    "  FIN                  [%7.3f ms]\n",
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_INIT],
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_PRE_CALCULATION],
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_MAKE_PUPIL_OUTSIDE],
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_MAKE_PUPIL_INSIDE],
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_BACKGROUND],
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_EYEBALL],
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_PUPIL_INSIDE],
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_PUPIL_OUTSIDE],
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_EYELID],
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_FIN]);
            result.append(buffer);

            sprintf(buffer, "  calling[%d]", (int)p_count);
            result.append(buffer);

            sprintf(buffer, " -> Emotion: %s\n", miens_text.c_str());
            result.append(buffer);
#endif
            sprintf(buffer,
                    "FPS is %7.3f [%8.3f ms][%4d times]", //
                    (p_count * 1000.0) / total_spent_msec,
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_TOTAL],
                    (int)p_count);
            result.append(buffer);
        } else {
            result = "Measuring...";
        }
        elapsed = current_time_msec;
    } else {
        result = "not recorded.";
    }

    return result;
}

} // namespace maid_robot_system
