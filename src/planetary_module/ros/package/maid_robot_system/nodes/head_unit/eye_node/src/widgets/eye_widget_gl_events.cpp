/**
 * @file eye_widget_gl_events.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "widgets/eye_widget.hpp"

namespace maid_robot_system
{
void EyeWidget::initializeGL()
{
#if DEBUG_OUTPUT_OPEN_GL
    printf("* initializeGL\n");
#endif
    int current = this->current_time.elapsed();
    // initializeOpenGLFunctions();
    //  connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &EyeWidget::cleanup);

    this->_flag_running = true;
    this->_flag_start   = false;
    this->eyelid->init(MIENS::miens_close, NEXT_EMOTION_INIT, current + EYE_BLINK_TIME_START_TIME_MS);
    this->eyeball->init();
    this->last_ros_msg_time = current;

    this->_flag_reload = true;
#if DEBUG_OUTPUT_OPEN_GL
    printf("* initializeGL end\n");
#endif
}

void EyeWidget::resizeGL(int w, int h)
{
    this->param.view_size.width  = (double)(w);
    this->param.view_size.height = (double)(h);
#if DEBUG_OUTPUT_OPEN_GL
    printf("* resizeGL [%d, %d]\n", this->param.view_size.width, this->param.view_size.height);
#endif
    this->reload_param();
    this->_flag_reload = true;
#if DEBUG_OUTPUT_OPEN_GL
    printf("* resizeGL end\n");
#endif
}

void EyeWidget::paintGL()
{
#if DEBUG_OUTPUT_OPEN_GL
    // printf("* paintGL\n");
#endif
    if (true == this->_flag_reload) {
        this->_flag_reload = false;
        this->eyeball->set_state_eyeball(-2);
        this->eyeball->set_state_cornea(-2);

        this->load();
#if DEBUG_OUTPUT_OPEN_GL
        printf("* paintGL : load image\n");
#endif
    }
    int current = this->current_time.elapsed();
    this->_screen_calculate();
    this->_screen_update();
    this->logger->set_index(this->logger->ST_INDEX_UPDATE, this->current_time.elapsed() - current);
}

} // namespace maid_robot_system
