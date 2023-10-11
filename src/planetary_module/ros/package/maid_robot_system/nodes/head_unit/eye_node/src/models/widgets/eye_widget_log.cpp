/**
 * @file eye_widget_log.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

namespace maid_robot_system
{
std::string EyeWidget::output_message()
{
    static int span = (int)(1000.0 * 60);

    static int log_timer = current_time.elapsed();
    int current          = current_time.elapsed();
    int elapsed          = (current - log_timer);

    if (span < elapsed) {
        logger.print((int)eyelid.eye_emotion, elapsed, current);
        log_timer = current;
    }
    if (0 > elapsed) {
        log_timer = current;
    }
    return "std::string EyeWidget::output_message";
}

} // namespace maid_robot_system
