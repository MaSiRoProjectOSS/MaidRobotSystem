/**
 * @file eye_widget_gl_events.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

namespace maid_robot_system
{
void EyeWidget::initializeGL()
{
#if DEBUG_OUTPUT_WIDGET
    printf(" * initializeGL\n");
#endif
    //  connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &EyeWidget::cleanup);
}

void EyeWidget::resizeGL(int w, int h)
{
    this->param.screen_size.width  = (double)w;
    this->param.screen_size.height = (double)h;
#if DEBUG_OUTPUT_WIDGET
    printf(" * resizeGL [%3.1f, %3.1f]\n", this->param.screen_size.width, this->param.screen_size.height);
#endif
    this->reload_param();
}

void EyeWidget::paintGL()
{
    this->update_screen();
}

} // namespace maid_robot_system
