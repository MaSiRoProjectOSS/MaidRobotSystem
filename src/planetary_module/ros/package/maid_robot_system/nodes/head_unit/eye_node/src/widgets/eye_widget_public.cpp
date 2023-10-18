/**
 * @file eye_widget_public.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "widgets/eye_widget.hpp"

namespace maid_robot_system
{
void EyeWidget::cornea_order()
{
    this->eyeball->set_state_cornea(PartsEyeball::CorneaState::Receiving);

    this->last_voiceId_time = this->current_time.elapsed();
    this->flag_voice_id     = true;
}

void EyeWidget::emotion(MIENS value)
{
    this->eyelid->set_emotion(value);
}
void EyeWidget::dimensions(float dimensions)
{
    this->eyeball->set_dimensions(dimensions);
}

void EyeWidget::stare(float distance, float left_y, float left_z, float right_y, float right_z)
{
    this->_request_left.set(0, left_y, left_z);
    this->_request_right.set(0, right_y, right_z);
    this->_request_distance = distance;
    this->last_ros_msg_time = this->current_time.elapsed();
}

bool EyeWidget::is_start()
{
    return this->_flag_start;
}
bool EyeWidget::request_update()
{
    this->logger->set_index(this->logger->ST_INDEX_REQUEST_UPDATE, 0);
    this->update();
    return this->_flag_running;
}

// =============================
// Constructor
// =============================
EyeWidget::EyeWidget(QWidget *parent) : QOpenGLWidget(parent), eyeball(), eyelid(), logger()
{
    this->logger  = new LogStore();
    this->eyeball = new PartsEyeball();
    this->eyelid  = new PartsEyelid();
}
EyeWidget::~EyeWidget()
{
    this->_flag_running = false;
}

} // namespace maid_robot_system
