/**
 * @file eye_widget_common.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

namespace maid_robot_system
{
/**
 * @brief
 *
 * @param obj
 * @param event
 * @return true
 * @return false
 */
bool EyeWidget::eventFilter(QObject *obj, QEvent *event)
{
    bool flag_close = false;

    switch (event->type()) {
        case QEvent::MouseButtonDblClick:
#if DEBUG_VIEW
            flag_close = true;
#endif
            break;
        case QEvent::KeyPress:
        case QEvent::MouseButtonPress:
        default:
            break;
    }

    if (true == flag_close) {
        Closing();
    }

    return false;
}
bool EyeWidget::event(QEvent *e)
{
    return true;
}
void EyeWidget::resizeEvent(QResizeEvent *event)
{
}

} // namespace maid_robot_system
