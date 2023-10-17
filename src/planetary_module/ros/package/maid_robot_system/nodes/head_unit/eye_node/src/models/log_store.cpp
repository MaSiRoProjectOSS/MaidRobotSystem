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
// =============================
// Constructor
// =============================
LogStore::LogStore()
{
    this->_clear();
}
LogStore::~LogStore()
{
}

// =============================
// PUBLIC : Function
// =============================
void LogStore::set_index(ST_INDEX_LOG index, int spent_time)
{
    if (ST_INDEX_LOG::ST_INDEX_LOG_MAX > index) {
        this->_spent_time_list[(int)index] += spent_time;
    }

    if (ST_INDEX_LOG::ST_INDEX_UPDATE == index) {
        this->_count_update++;
    }

    if (ST_INDEX_LOG::ST_INDEX_REQUEST_UPDATE == index) {
        this->_count_request++;
    }
}

std::string LogStore::get_message(std::string miens_text, double current_time_msec, bool verbose)
{
    std::string result    = "";
    static double elapsed = current_time_msec;
    char buffer[1024];
    double cnt_update  = (double)this->_count_update;
    double cnt_request = (double)this->_count_request;
    if (0 < this->_count_update) {
        double total_spent_msec = (current_time_msec - elapsed);
        float p_spent_time_list[ST_INDEX_LOG::ST_INDEX_LOG_MAX];
        for (int i = 0; i < ST_INDEX_LOG::ST_INDEX_LOG_MAX; i++) {
            p_spent_time_list[i] = this->_spent_time_list[i] / cnt_update;
        }
        this->_clear();
        if (0 < total_spent_msec) {
            if (true == verbose) {
                result.append("======================\n");
                sprintf(buffer,
                        "  elapsed : %12.3f s\n"
                        "  spent   : %12.3f s\n",
                        (current_time_msec / 1000.0),
                        (total_spent_msec / 1000.0));
                result.append(buffer);

                sprintf(buffer,
                        "  Init                  [%7.3f ms]\n"
                        "  Calculate[0]          [%7.3f ms]\n"
                        "  Calculate[1]          [%7.3f ms]\n"
                        "  DRAW - foundation     [%7.3f ms]\n"
                        "  DRAW - eyeball        [%7.3f ms]\n"
                        "  DRAW - cornea outside [%7.3f ms]\n"
                        "  DRAW - cornea inside  [%7.3f ms]\n"
                        "  DRAW - eyelid         [%7.3f ms]\n"
                        "  FIN                   [%7.3f ms]\n",
                        p_spent_time_list[ST_INDEX_LOG::ST_INDEX_INIT],
                        p_spent_time_list[ST_INDEX_LOG::ST_INDEX_CALCULATION_STEP1],
                        p_spent_time_list[ST_INDEX_LOG::ST_INDEX_CALCULATION_STEP2],
                        p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_FOUNDATION],
                        p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_EYEBALL],
                        p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_CORNEA_INSIDE],
                        p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_CORNEA_OUTSIDE],
                        p_spent_time_list[ST_INDEX_LOG::ST_INDEX_DRAW_EYELID],
                        p_spent_time_list[ST_INDEX_LOG::ST_INDEX_FIN]);
                result.append(buffer);

                sprintf(buffer, "  update [%d]", (int)cnt_update);
                result.append(buffer);

                sprintf(buffer, " -> Emotion: %s\n", miens_text.c_str());
                result.append(buffer);
            }
            sprintf(buffer,
                    "FPS is %7.3f [%8.3f ms][%4d/%4d times] interval: %8.3f s", //
                    (cnt_update * 1000.0) / total_spent_msec,
                    p_spent_time_list[ST_INDEX_LOG::ST_INDEX_UPDATE],
                    (int)cnt_update,
                    (int)cnt_request,
                    (total_spent_msec / 1000.0));
            result.append(buffer);
        } else {
            result = "Measuring...";
        }
        elapsed = current_time_msec;
    } else {
        sprintf(buffer, "not recorded [%4d/%4d times] ", (int)cnt_update, (int)cnt_request);
        result.append(buffer);
    }

    return result;
}

// =============================
// PRIVATE : Function
// =============================
void LogStore::_clear()
{
    for (int i = 0; i < ST_INDEX_LOG::ST_INDEX_LOG_MAX; i++) {
        this->_spent_time_list[i] = 0;
    }
    this->_count_update  = 0;
    this->_count_request = 0;
}

} // namespace maid_robot_system
