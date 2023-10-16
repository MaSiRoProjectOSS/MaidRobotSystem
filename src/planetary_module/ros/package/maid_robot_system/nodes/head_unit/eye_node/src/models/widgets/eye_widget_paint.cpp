/**
 * @file eye_widget_paint.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

namespace maid_robot_system
{
#define DEBUG_VIEW 1

void EyeWidget::calculate()
{
    int current = current_time.elapsed();
    if ((0 != this->eyeball->right_eye.target.x) || (0 != this->eyeball->right_eye.target.y)  //
        || (0 != this->eyeball->left_eye.target.x) || (0 != this->eyeball->left_eye.target.y) //
        || (false == flag_first_request)) {
        int elapsed_times = (current - this->last_ros_msg_time);

        if ((LOST_ROS_MSG_TIMEOUT_SECONDS * 1000) < elapsed_times) {
            if (false == flag_first_request) {
                this->eyelid->set_emotion(NEXT_EMOTION_INIT);
                flag_first_request = true;
            }

            this->eyeball->set_default();
            this->last_ros_msg_time = current;
        }
        if (0 > elapsed_times) {
            this->last_ros_msg_time = current;
        }
    }
    //********************************************************************************//
    if (true == this->flag_voice_id) {
        int voiceId_time = (current - this->last_voiceId_time);
        if (VOICE_MESSAGE_TIMEOUT_MS < voiceId_time) {
            this->flag_voice_id = false;
            this->eyeball->set_state_cornea(PartsEyeball::CorneaState::Normal);
        }
        if (0 > voiceId_time) {
            voiceId_time = current;
        }
    }
    this->update();
}

void EyeWidget::_update_screen()
{
    int current    = current_time.elapsed();
    int start_time = current;
    QPainter painter(this);

    if (true != painter.isActive()) {
        painter.begin(this);
    }
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    // painter.setViewport(0,0,this->window_size_x * 0.5, this->window_size_y * 0.5);
    painter.scale(this->param.screen_resolution, this->param.screen_resolution);

    logger.set_index(logger.ST_INDEX_INIT, current_time.elapsed() - start_time);
    /* ============================================= */
    // Pre-calculation
    /* ============================================= */
    start_time       = current_time.elapsed();
    int eyelid_index = this->eyelid->calc_animation(current);
    this->eyeball->calculate(current, eyelid_index);
    logger.set_index(logger.ST_INDEX_PRE_CALCULATION, current_time.elapsed() - start_time);

    /* ============================================= */
    // Make cornea outside
    /* ============================================= */
    start_time = current_time.elapsed();
    painter.fillRect(this->param.screen_size.x, //
                     this->param.screen_size.y,
                     this->param.screen_size.width,
                     this->param.screen_size.height,
                     this->eyelid->color);

    painter.fillRect((int)this->eyelid->left_eye.rect.x, //
                     (int)this->eyelid->left_eye.rect.y,
                     (int)this->eyelid->left_eye.rect.width,
                     (int)this->eyelid->left_eye.rect.height,
                     this->eyeball->left_eye.color);
    painter.fillRect((int)this->eyelid->right_eye.rect.x, //
                     (int)this->eyelid->right_eye.rect.y,
                     (int)this->eyelid->right_eye.rect.width,
                     (int)this->eyelid->right_eye.rect.height,
                     this->eyeball->right_eye.color);
    logger.set_index(logger.ST_INDEX_DRAW_BACKGROUND, current_time.elapsed() - start_time);

    /* ============================================= */
    // Draw : eye
    /* ============================================= */
    start_time = current_time.elapsed();
    if (true == this->eyeball->right_eye.store.exit_vitreous) {
        painter.drawPixmap(this->eyeball->right_eye.draw_postion.x,
                           this->eyeball->right_eye.draw_postion.y,
                           this->eyeball->right_eye.draw_postion.width,
                           this->eyeball->right_eye.draw_postion.height,
                           this->eyeball->right_eye.eyeball);
    }
    if (true == this->eyeball->left_eye.store.exit_vitreous) {
        painter.drawPixmap(this->eyeball->left_eye.draw_postion.x,
                           this->eyeball->left_eye.draw_postion.y,
                           this->eyeball->left_eye.draw_postion.width,
                           this->eyeball->left_eye.draw_postion.height,
                           this->eyeball->left_eye.eyeball);
    }
    logger.set_index(logger.ST_INDEX_DRAW_EYEBALL, current_time.elapsed() - start_time);

#if DRAW_CORNEA_OUTSIDE
    /* ============================================= */
    // Draw : cornea outside
    /* ============================================= */
    start_time = current_time.elapsed();
    if (true == this->param.right_eye.cornea_outside.enable) {
        if (true == this->eyeball->right_eye.store.exit_cornea_outside) {
            painter.drawPixmap(this->eyeball->right_eye.draw_cornea_anime.x, this->eyeball->right_eye.draw_cornea_anime.y, this->eyeball->right_eye.cornea_outside);
        }
    }
    if (true == this->param.left_eye.cornea_outside.enable) {
        if (true == this->eyeball->left_eye.store.exit_cornea_outside) {
            painter.drawPixmap(this->eyeball->left_eye.draw_cornea_anime.x, this->eyeball->left_eye.draw_cornea_anime.y, this->eyeball->left_eye.cornea_outside);
        }
    }
    logger.set_index(logger.ST_INDEX_DRAW_CORNEA_OUTSIDE, current_time.elapsed() - start_time);
#endif
#if DRAW_CORNEA_INSIDE
    /* ============================================= */
    // Draw : cornea inside
    /* ============================================= */
    start_time = current_time.elapsed();
    if (true == this->param.right_eye.cornea_inside.enable) {
        if (true == this->eyeball->right_eye.store.exit_cornea_inside) {
            painter.drawPixmap(this->eyeball->right_eye.draw_cornea_anime2.x, this->eyeball->right_eye.draw_cornea_anime2.y, this->eyeball->right_eye.cornea_inside);
        }
    }
    if (true == this->param.left_eye.cornea_inside.enable) {
        if (true == this->eyeball->left_eye.store.exit_cornea_inside) {
            painter.drawPixmap(this->eyeball->left_eye.draw_cornea_anime2.x, this->eyeball->left_eye.draw_cornea_anime2.y, this->eyeball->left_eye.cornea_inside);
        }
    }
    logger.set_index(logger.ST_INDEX_DRAW_CORNEA_INSIDE, current_time.elapsed() - start_time);
#endif
    /* ============================================= */
    // Draw : eyelid
    /* ============================================= */
    start_time = current_time.elapsed();
    if (true == this->eyelid->right_eye.exit_eyelid) {
        painter.drawPixmap(this->eyelid->right_eye.rect.x,
                           this->eyelid->right_eye.rect.y,
                           this->eyelid->right_eye.rect.width,
                           this->eyelid->right_eye.rect.height,
                           this->eyelid->get_eye_id_right());
    }
    if (true == this->eyelid->left_eye.exit_eyelid) {
        painter.drawPixmap(this->eyelid->left_eye.rect.x,
                           this->eyelid->left_eye.rect.y,
                           this->eyelid->left_eye.rect.width,
                           this->eyelid->left_eye.rect.height,
                           this->eyelid->get_eye_id_left());
    }
    logger.set_index(logger.ST_INDEX_DRAW_EYELID, current_time.elapsed() - start_time);

    /* ============================================= */
    // final
    /* ============================================= */
    start_time = current_time.elapsed();
    //painter.endNativePainting();
    painter.end();
    logger.set_index(logger.ST_INDEX_FIN, current_time.elapsed() - start_time);
    //********************************************************************************//
}

} // namespace maid_robot_system
