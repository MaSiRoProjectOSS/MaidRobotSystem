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
void EyeWidget::log()
{
    int current = current_time.elapsed();
    int elapsed = (current - log_timer);

    if ((CTRL_HITOMI_LOG_FPS * 1000.0) < elapsed) {
        logger.print((int)eyelid.eye_emotion, elapsed, current);
        log_timer = current;
    }
    if (0 > elapsed) {
        log_timer = current;
    }
}

} // namespace maid_robot_system
