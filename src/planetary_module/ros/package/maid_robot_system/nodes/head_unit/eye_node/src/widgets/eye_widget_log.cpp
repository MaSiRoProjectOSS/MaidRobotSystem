/**
 * @file eye_widget_log.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "widgets/eye_widget.hpp"

namespace maid_robot_system
{
std::string EyeWidget::_miens_text(MIENS value)
{
    std::string result = "";
    switch (value) {
        case MIENS::miens_smile:
            result = "smile";
            break;
        case MIENS::miens_close_left:
            result = "lose_left";
            break;
        case MIENS::miens_close_right:
            result = "close_right";
            break;
        case MIENS::miens_close:
            result = "close";
            break;
        case MIENS::miens_normal:
            result = "normal";
            break;
        case MIENS::miens_wink_left:
            result = "wink_left";
            break;
        case MIENS::miens_wink_right:
            result = "wink_right";
            break;
        default:
            result = "unknow";
            break;
    }

    return result;
}

std::string EyeWidget::output_message(bool verbose)
{
    return this->logger->get_message(this->_miens_text(this->eyelid->eye_emotion), this->current_time.elapsed(), verbose);
}

bool EyeWidget::is_response()
{
    static int times = 0;
    bool result      = true;

    if (COUNTER_TIMEOUT_QT_THRESHOLD_PERCENT <= this->logger->get_response_percent()) {
        times = 0;
    } else {
        times++;
    }
    if (COUNTER_TIMEOUT_QT_TIMES <= times) {
        times  = 0;
        result = false;
    }

    return result;
}

} // namespace maid_robot_system
