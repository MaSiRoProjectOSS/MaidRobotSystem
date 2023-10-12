/**
 * @file eye_widget_public.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

namespace maid_robot_system
{
bool EyeWidget::reload_param()
{
    bool result = false;
    if (true == this->_flag_initialized) {
#if DEBUG_OUTPUT_WIDGET
        printf("  ---- reload_param ----\n");
#endif
        this->eyelid.set_param(this->param);
        this->param.eyeball_center_left.x  = this->eyelid.left.pos_center.x + this->param.eyeball_position_l_x;
        this->param.eyeball_center_left.y  = this->eyelid.left.pos_center.y + this->param.eyeball_position_l_y;
        this->param.eyeball_center_right.x = this->eyelid.right.pos_center.x + this->param.eyeball_position_r_x;
        this->param.eyeball_center_right.y = this->eyelid.right.pos_center.y + this->param.eyeball_position_r_y;
        this->eyeball.set_param(this->param);
        result = true;
    }
    return result;
}

void EyeWidget::pupil_order()
{
    this->eyeball.set_state_pupil(PartsEyeball::PupilState::Receiving);

    this->last_voiceId_time = current_time.elapsed();
    this->flag_voice_id     = true;
}

void EyeWidget::emotion(MIENS value)
{
    this->eyelid.set_emotion(value);
}

void EyeWidget::stare(float size, float distance, float left_x, float left_y, float right_x, float right_y)
{
    if (false == flag_first_request) {
        flag_first_request = true;
    }
    const int limit_y = 190;
    /* 瞳の大きさ */
    this->eyeball.set_dimensions(size);
    /* ============================================= */
    float get_target_y = ((left_y / 200.0) * param.eyelid_size_x);
    float get_target_x = ((-left_x / 230.0) * param.eyelid_size_y);
    /* ============================================= */
    const int min_per_eye_distance = 8;
    /* 近くなるほど寄り目になる */
    float distance_change = min_per_eye_distance - ((distance / 1500) * min_per_eye_distance);

    if (distance_change < 0) {
        distance_change = 0;
    }

    if (distance_change > min_per_eye_distance) {
        distance_change = min_per_eye_distance;
    }

    /* ============================================= */
    if (get_target_y > limit_y) {
        get_target_y = limit_y;
    }

    if (get_target_y < -limit_y) {
        get_target_y = -limit_y;
    }

    /* ============================================= */
#if DEBUG_OUTPUT_WIDGET
    printf("eyeball POS: %5.3f, %5.3f / %5.3f", get_target_x, get_target_y, distance_change);
#endif

    /* ============================================= */
    if (abs(get_target_x) < 500 && abs(get_target_y) < 400) {
        switch (this->thinking_flag_notAccepted) {
            case control_state::STATE_NOT_ACCEPTED:
                if (param.thinking_next_time_notAccepted < current_time.elapsed()) {
                    this->thinking_flag_notAccepted = control_state::STATE_FREE;
#if DEBUG_OUTPUT_WIDGET
                    printf("Thinking blink : FREE\n");
#endif
                }
                break;
            case control_state::STATE_ACCEPTED:
                if (param.thinking_next_time_notAccepted < current_time.elapsed()) {
                    this->eyelid.set_eye_blink(PartsEyelid::blink_type::BLINK_TYPE_MIN_MAX, false);
                    this->thinking_flag_notAccepted = control_state::STATE_NOT_ACCEPTED;
                    param.thinking_next_time_notAccepted
                            = current_time.elapsed() + (int)func_rand(EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_LOWER, EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_UPPER);
#if DEBUG_OUTPUT_WIDGET
                    printf("Thinking blink : NOT ACCEPTED\n");
#endif
                    break;
                }
            case control_state::STATE_FREE:
            default:
                if (abs(this->eyeball.right.target.y - get_target_y) > 150 || abs(this->eyeball.right.target.x - get_target_x) > 150) {
                    this->eyelid.set_eye_blink(PartsEyelid::blink_type::BLINK_TYPE_QUICKLY, true);

                    if (control_state::STATE_FREE == this->thinking_flag_notAccepted) {
                        param.thinking_next_time_notAccepted
                                = current_time.elapsed() + (int)func_rand(EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_LOWER, EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_UPPER);
#if DEBUG_OUTPUT_WIDGET
                        printf("Thinking blink : ACCEPTED\n");
#endif
                    }

                    this->thinking_flag_notAccepted = control_state::STATE_ACCEPTED;
                }

                break;
        }

        if (abs(this->eyeball.right.target.x - get_target_x) > 0) {
            this->eyeball.right.target.x = get_target_x + ((param.eyeball_size_x * distance_change) / 100.0);
            this->eyeball.left.target.x  = get_target_x - ((param.eyeball_size_x * distance_change) / 100.0);
        }

        if (abs(this->eyeball.right.target.y - get_target_y) > 0) {
            this->eyeball.right.target.y = get_target_y;
            this->eyeball.left.target.y  = get_target_y;
        }
    }

    /* ============================================= */
    this->last_ros_msg_time = current_time.elapsed();
}

// =============================
// Constructor
// =============================
EyeWidget::EyeWidget(QWidget *parent) : QOpenGLWidget(parent), eyeball(), eyelid(), logger()
{
}
EyeWidget::~EyeWidget()
{
}

} // namespace maid_robot_system
