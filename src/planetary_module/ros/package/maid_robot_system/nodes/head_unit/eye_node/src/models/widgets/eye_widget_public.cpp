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
/**
 * @brief voiceIDの受信
 *
 */
void EyeWidget::CMD_voiceId()
{
    this->eyeball.set_state_pupil(PartsEyeball::PupilState::Receiving);

    last_voiceId_time = current_time.elapsed();
    flag_voiceId      = true;
}

/**
 * @brief
 * @param get_eye_cmd_angle
*/
void EyeWidget::CMD_eye_input(const maid_robot_system_interfaces::msg::MrsEye msg)
{
    if (false == flag_first_request) {
        flag_first_request = true;
    }
    const int limit_y = 190;
    // printf("eye_cmd_input");
    /* 感情の設定*/
    this->eyelid.set_emotion((MIENS)((int)msg.emotions));
    /* 瞳の大きさ */
    this->eyeball.set_dimensions(msg.size);
    /* ============================================= */
    float get_target_y = ((msg.left_y / 200.0) * this->calibration_eyelid_size_x);
    float get_target_x = ((-msg.left_x / 230.0) * this->calibration_eyelid_size_x);
    /* ============================================= */
    const int min_per_eye_distance = 8;
    /* 近くなるほど寄り目になる */
    float distance_change = min_per_eye_distance - ((msg.distance / 1500) * min_per_eye_distance);

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
#if DEBUG_PRINT
    printf("eyeball POS: %5.3f, %5.3f / %5.3f", get_target_x, get_target_y, distance_change);
#endif

    /* ============================================= */
    if (abs(get_target_x) < 500 && abs(get_target_y) < 400) {
        switch (this->thinking_flag_notAccepted) {
            case control_state::STATE_NOT_ACCEPTED:
                if (this->thinking_next_time_notAccepted < current_time.elapsed()) {
                    this->thinking_flag_notAccepted = control_state::STATE_FREE;
#if DEBUG_PRINT
                    printf("Thinking blink : FREE\n");
#endif
                }

                break;

            case control_state::STATE_ACCEPTED:
                if (this->thinking_next_time_notAccepted < current_time.elapsed()) {
                    this->eyelid.set_eye_blink(PartsEyelid::blink_type::BLINK_TYPE_MIN_MAX, false);
                    this->thinking_flag_notAccepted = control_state::STATE_NOT_ACCEPTED;
                    this->thinking_next_time_notAccepted
                            = current_time.elapsed() + (int)func_rand(EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_LOWER, EYE_BLINK_FREQUENT_NOT_ACCEPTED_MILLISECOND_UPPER);
#if DEBUG_PRINT
                    printf("Thinking blink : NOT ACCEPTED\n");
#endif
                    break;
                }

            case control_state::STATE_FREE:
            default:
                if (abs(this->eyeball.right.target.y - get_target_y) > 150 || abs(this->eyeball.right.target.x - get_target_x) > 150) {
                    this->eyelid.set_eye_blink(PartsEyelid::blink_type::BLINK_TYPE_QUICKLY, true);

                    if (control_state::STATE_FREE == this->thinking_flag_notAccepted) {
                        this->thinking_next_time_notAccepted
                                = current_time.elapsed() + (int)func_rand(EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_LOWER, EYE_BLINK_FREQUENT_ACCEPTED_MILLISECOND_UPPER);
#if DEBUG_PRINT
                        printf("Thinking blink : ACCEPTED\n");
#endif
                    }

                    this->thinking_flag_notAccepted = control_state::STATE_ACCEPTED;
                }

                break;
        }

        if (abs(this->eyeball.right.target.x - get_target_x) > 0) {
            this->eyeball.right.target.x = get_target_x + ((this->eyeball_size_x * distance_change) / 100.0);
            this->eyeball.left.target.x  = get_target_x - ((this->eyeball_size_x * distance_change) / 100.0);
        }

        if (abs(this->eyeball.right.target.y - get_target_y) > 0) {
            this->eyeball.right.target.y = get_target_y;
            this->eyeball.left.target.y  = get_target_y;
        }
    }

    /* ============================================= */
    last_ros_msg_time = current_time.elapsed();
}

} // namespace maid_robot_system
