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
#if DEBUG_OUTPUT_OPEN_GL
    printf(" * initializeGL\n");
#endif
    // initializeOpenGLFunctions();
    //  connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &EyeWidget::cleanup);

#if DEBUG_OUTPUT_OPEN_GL
    printf(" * initializeGL end\n");
#endif
}

void EyeWidget::resizeGL(int w, int h)
{
    this->param.view_size.width  = (double)(w);
    this->param.view_size.height = (double)(h);
#if DEBUG_OUTPUT_OPEN_GL
    printf(" * resizeGL [%d, %d]\n", this->param.view_size.width, this->param.view_size.height);
#endif
    this->reload_param();
    this->load();
}

void EyeWidget::paintGL()
{
    int current = current_time.elapsed();
    this->calculate();
    this->_update_screen();
    logger.set_index(logger.ST_INDEX_TOTAL, current_time.elapsed() - current);
}

} // namespace maid_robot_system
