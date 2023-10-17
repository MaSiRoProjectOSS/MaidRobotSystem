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

void EyeWidget::_screen_calculate()
{
    const int min_per_eye_distance = 8;
    const int limit_y              = 190;
    static int previous            = 0;
    int current                    = this->current_time.elapsed();
    /* ============================================= */
    // Calculation / Step 1
    /* ============================================= */
    int start_time = current;
    if (previous < this->last_ros_msg_time) {
        previous              = this->last_ros_msg_time;
        float get_target_y    = ((-this->_request_left.z / 200.0) * param.left_eye.eyelid.height);
        float get_target_x    = ((-this->_request_left.y / 230.0) * param.left_eye.eyelid.width);
        float distance_change = min_per_eye_distance - ((this->_request_distance / 1500) * min_per_eye_distance);

        if (distance_change < 0) {
            distance_change = 0;
        }
        if (distance_change > min_per_eye_distance) {
            distance_change = min_per_eye_distance;
        }
        if (get_target_y > limit_y) {
            get_target_y = limit_y;
        }
        if (get_target_y < -limit_y) {
            get_target_y = -limit_y;
        }

        if (abs(get_target_x) < 500 && abs(get_target_y) < 400) {
            switch (this->thinking_flag_notAccepted) {
                case control_state::STATE_NOT_ACCEPTED:
                    if (this->_thinking_next_time_notAccepted < current) {
                        this->thinking_flag_notAccepted = control_state::STATE_FREE;
#if DEBUG_OUTPUT_WIDGET
                        printf("Thinking blink : FREE\n");
#endif
                    }
                    break;
                case control_state::STATE_ACCEPTED:
                    if (this->_thinking_next_time_notAccepted < current) {
                        this->eyelid->set_eye_blink(PartsEyelid::blink_type::BLINK_TYPE_MIN_MAX, false);
                        this->thinking_flag_notAccepted = control_state::STATE_NOT_ACCEPTED;
                        this->_thinking_next_time_notAccepted
                                = current + (int)func_rand(EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_LOWER, EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_UPPER);
#if DEBUG_OUTPUT_WIDGET
                        printf("Thinking blink : NOT ACCEPTED\n");
#endif
                        break;
                    }
                case control_state::STATE_FREE:
                default:
                    if (abs(this->eyeball->right_eye.target.y - get_target_y) > 150 || abs(this->eyeball->right_eye.target.x - get_target_x) > 150) {
                        this->eyelid->set_eye_blink(PartsEyelid::blink_type::BLINK_TYPE_QUICKLY, true);

                        if (control_state::STATE_FREE == this->thinking_flag_notAccepted) {
                            this->_thinking_next_time_notAccepted
                                    = current + (int)func_rand(EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_LOWER, EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_UPPER);
#if DEBUG_OUTPUT_WIDGET
                            printf("Thinking blink : ACCEPTED\n");
#endif
                        }

                        this->thinking_flag_notAccepted = control_state::STATE_ACCEPTED;
                    }

                    break;
            }

            if (abs(this->eyeball->right_eye.target.x - get_target_x) > 0) {
                this->eyeball->left_eye.target.x  = get_target_x - ((param.left_eye.eyeball.width * distance_change) / 100.0);
                this->eyeball->right_eye.target.x = get_target_x + ((param.right_eye.eyeball.width * distance_change) / 100.0);
            }

            if (abs(this->eyeball->right_eye.target.y - get_target_y) > 0) {
                this->eyeball->right_eye.target.y = get_target_y;
                this->eyeball->left_eye.target.y  = get_target_y;
            }
        }
    } else {
        if ((0 != this->eyeball->right_eye.target.x) || (0 != this->eyeball->right_eye.target.y) //
            || (0 != this->eyeball->left_eye.target.x) || (0 != this->eyeball->left_eye.target.y)) {
            if (LOST_ROS_MSG_TIMEOUT_SECONDS < (current - this->last_ros_msg_time)) {
                this->eyeball->set_default();
            }
        }
    }
    //********************************************************************************//
    if (true == this->flag_voice_id) {
        if (VOICE_MESSAGE_TIMEOUT_MS < (current - this->last_voiceId_time)) {
            this->flag_voice_id = false;
            this->eyeball->set_state_cornea(PartsEyeball::CorneaState::Normal);
        }
    }
    //********************************************************************************//
    this->logger->set_index(this->logger->ST_INDEX_CALCULATION_STEP1, this->current_time.elapsed() - start_time);
    /* ============================================= */
    // Calculation / Step 2
    /* ============================================= */
    start_time       = this->current_time.elapsed();
    int eyelid_index = this->eyelid->calculate(current);
    this->eyeball->calculate(current, eyelid_index);
    this->logger->set_index(this->logger->ST_INDEX_CALCULATION_STEP2, this->current_time.elapsed() - start_time);
}

void EyeWidget::_screen_update()
{
    int start_time = this->current_time.elapsed();
    QPainter painter(this);
    /* ============================================= */
    // BEGIN
    /* ============================================= */
    if (true != painter.isActive()) {
        painter.begin(this);
    }
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    // painter.setViewport(0,0,this->window_size_x * 0.5, this->window_size_y * 0.5);
    painter.scale(this->param.screen_resolution, this->param.screen_resolution);
    this->logger->set_index(this->logger->ST_INDEX_INIT, this->current_time.elapsed() - start_time);

    /* ============================================= */
    // Draw : foundation
    /* ============================================= */
    start_time = this->current_time.elapsed();
    this->eyelid->update_background(painter, this->param.screen_size);
    this->eyeball->update_background(painter);
    this->logger->set_index(this->logger->ST_INDEX_DRAW_FOUNDATION, this->current_time.elapsed() - start_time);

    /* ============================================= */
    // Draw : eye
    /* ============================================= */
    start_time = this->current_time.elapsed();
    this->eyeball->update_eyeball(painter);
    this->logger->set_index(this->logger->ST_INDEX_DRAW_EYEBALL, this->current_time.elapsed() - start_time);

#if DRAW_CORNEA_OUTSIDE
    /* ============================================= */
    // Draw : cornea outside
    /* ============================================= */
    start_time = this->current_time.elapsed();
    this->eyeball->update_cornea_outside(painter);
    this->logger->set_index(this->logger->ST_INDEX_DRAW_CORNEA_OUTSIDE, this->current_time.elapsed() - start_time);
#endif
#if DRAW_CORNEA_INSIDE
    /* ============================================= */
    // Draw : cornea inside
    /* ============================================= */
    start_time = this->current_time.elapsed();
    this->eyeball->update_cornea_inside(painter);
    this->logger->set_index(this->logger->ST_INDEX_DRAW_CORNEA_INSIDE, this->current_time.elapsed() - start_time);
#endif
    /* ============================================= */
    // Draw : eyelid
    /* ============================================= */
    start_time = this->current_time.elapsed();
    this->eyelid->update(painter);
    this->logger->set_index(this->logger->ST_INDEX_DRAW_EYELID, this->current_time.elapsed() - start_time);

    /* ============================================= */
    // END
    /* ============================================= */
    start_time = this->current_time.elapsed();
    //painter.endNativePainting();
    painter.end();
    this->logger->set_index(this->logger->ST_INDEX_FIN, this->current_time.elapsed() - start_time);
    //********************************************************************************//
}

} // namespace maid_robot_system
