/**
 * @file posture_manager_neck.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Manage posture of neck.
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/manager/posture_manager_neck.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

PostureManager_Neck::PostureManager_Neck()
{
}

PostureManager_Neck::~PostureManager_Neck()
{
}

bool PostureManager_Neck::_begin()
{
    bool result = true;
    this->_setup();
    return result;
}

bool PostureManager_Neck::_calculate()
{
    static TimeCheck ROS_neck_time;
    bool result = true;

    this->_resident(this->_args);
    /////////////////////////////////////////////

    if (true == ROS_neck_time.check_passing(5000)) {
        this->_args->states.mode_flag_face_track = 0;
    }

    if (PostureManagerArguments::MODE_NECK_FACE_TRACKING == this->_args->input.neck_mode) {
        /* If there is face recognition information, */

        ROS_neck_time.update();

        if (this->_args->states.flag_face_track_enable == 1) {
            if (abs(this->_args->input.neck_angle.roll) < this->_QUARTER_CIRCLE_DEGREE && abs(this->_args->input.neck_angle.pitch) < this->_QUARTER_CIRCLE_DEGREE
                && abs(this->_args->input.neck_angle.yaw) < this->_QUARTER_CIRCLE_DEGREE) {
                this->_args->states.mode_flag_face_track = 1;

                this->_args->output.neck_face_track.yaw = this->_args->output.neck_current.yaw - this->_args->input.neck_angle.yaw;
                float calc_yaw_rate                     = constrain(this->_args->output.neck_current.yaw / (float)this->_QUARTER_CIRCLE_DEGREE * 100, -100, 100) / 100.0;
                float calc_yaw_rad                      = this->_args->output.neck_current.yaw * this->_DEG_TO_RAD_FACTOR;
                float calc_roll  = (this->_args->input.neck_angle.pitch * sin(calc_yaw_rad) / 3.0) + (this->_args->input.neck_angle.roll * cos(calc_yaw_rad));
                float calc_pitch = this->_args->input.neck_angle.pitch * cos(calc_yaw_rad);

                this->_args->output.neck_face_track.pitch = (this->_args->output.neck_current.pitch + calc_pitch)
                                                          * map(abs(this->_args->output.neck_current.yaw), 0, NECK_LIMIT_YAW[1], 90, 0) / (float)this->_QUARTER_CIRCLE_DEGREE;
                this->_args->output.neck_face_track.roll = this->_args->output.neck_current.roll + calc_roll;
            }
        }
    }

    return result;
}

void PostureManager_Neck::_set_hand_track(PostureManagerArguments *args, float _hand_x, float _hand_y, float _hand_z)
{
    this->_arm_track_pitch         = map(_hand_z, 0, 300, 0, 20);
    this->_arm_track_yaw           = atan2(_hand_y, _hand_x) * this->_RAD_TO_DEG_FACTOR;
    this->_arm_track_roll          = map(-this->_arm_track_yaw, 0, 45, 0, 10);
    float sum_hand_track_check_now = this->_arm_track_pitch + this->_arm_track_roll + this->_arm_track_yaw;

    if (args->states.mode_flag_face_track == 1) {
        if (true == this->_timer_hand_track_check.check_passing(1500)) {
            this->_mode_flag_arm_track = true;

            if (abs(this->_sum_hand_track_check_old - sum_hand_track_check_now) > 30) {
                this->_mode_flag_arm_track = true;
            } else {
                this->_mode_flag_arm_track = false;
            }

            this->_sum_hand_track_check_old = sum_hand_track_check_now;
        }
    } else {
        this->_mode_flag_arm_track = true;
    }
}

void PostureManager_Neck::_setup()
{
    this->_random_look_timer.update();
}

void PostureManager_Neck::set_target(float _yaw, float _pitch, float _roll)
{
    this->_target_yaw   = _yaw;
    this->_target_pitch = _pitch;
    this->_target_roll  = _roll;
}

void PostureManager_Neck::_look_random(PostureManagerArguments *args)
{
    float S_curve = this->_random_look_timer.get_s_curve_flag(this->_random_look_sendtime);

    if (this->_random_look_timer.get_elapsed_time() > this->_random_look_sendtime) {
        this->_random_start_look_yaw   = args->output.neck_current.yaw;
        this->_random_start_look_pitch = args->output.neck_current.pitch;
        this->_random_start_look_roll  = args->output.neck_current.roll;

        this->_mode                     = RANDOM_MOVE;
        this->_random_target_look_yaw   = random(this->_TARGET_LOOK_YAW_RANDOM_RANGE_MIN, this->_TARGET_LOOK_YAW_RANDOM_RANGE_MAX);
        this->_random_target_look_pitch = random(this->_TARGET_LOOK_PITCH_RANDOM_RANGE_MIN, this->_TARGET_LOOK_PITCH_RANDOM_RANGE_MAX);
        this->_random_target_look_roll  = random(this->_TARGET_LOOK_ROLL_RANDOM_RANGE_MIN, this->_TARGET_LOOK_ROLL_RANDOM_RANGE_MAX);
        this->_random_look_sendtime     = random(this->_LOOK_SEND_TIME_RANDOM_RANGE_MIN, this->_LOOK_SEND_TIME_RANDOM_RANGE_MAX);
        this->_random_look_timer.update();
    } else {
        this->_random_look_yaw  = constrain(this->_random_start_look_yaw + this->_random_target_look_yaw * S_curve, this->_RANDOM_LOOK_YAW_MIN, this->_RANDOM_LOOK_YAW_MAX);
        this->_random_look_roll = constrain(this->_random_start_look_roll + this->_random_target_look_roll * S_curve, this->_RANDOM_LOOK_ROLL_MIN, this->_RANDOM_LOOK_ROLL_MAX);
        this->_random_look_pitch
                = constrain(this->_random_start_look_pitch + this->_random_target_look_pitch * S_curve, this->_RANDOM_LOOK_PITCH_MIN, this->_RANDOM_LOOK_PITCH_MAX);
    }
}

void PostureManager_Neck::reset(PostureManagerArguments *args)
{
    set_target(0, 0, 0);
    args->states.mode_flag_face_track = 0;
    this->_mode_flag_arm_track        = false;
    this->_sum_hand_track_check_old   = 0;
    this->_timer_reset.update();
}

void PostureManager_Neck::_resident(PostureManagerArguments *args)
{
    float dt = (micros() - this->_resident_time) / this->_SEC_TO_MICRO_SEC_FACTOR;

    /* Reset if the neck doesn't move gor a while when the mode is FACE_TRACK.  */
    if (this->_mode != FACE_TRACK || abs(args->output.neck_current.yaw) > this->_NECK_YAW_RESET_LIMIT) {
        if (true == timer_check_delta_pose.check_passing(30000)) {
            int sum_pose            = args->output.neck_current.pitch + args->output.neck_current.roll + args->output.neck_current.yaw;
            args->states.delta_pose = abs(sum_pose - this->_old_delta_pose);
            this->_old_delta_pose   = sum_pose;
            if (args->states.delta_pose < this->_NECK_DELTA_POSE_LOW_LIMIT) {
                reset(args);
                args->states.flag_face_track_enable = 0;
            }
        }
    } else {
        timer_check_delta_pose.update();
    }

    /* Wait for a while, and reset flag_face_track_enable */
    if (args->states.flag_face_track_enable == 0) {
        if (true == this->_timer_reset.check_time_over(2000)) {
            args->states.flag_face_track_enable = 1;
        }
    }

    pose_manager(args);

    if (true == this->_mode_flag_command_move) {
        this->_mode = COMMAND_MOVING;
    } else {
        if (args->states.mode_flag_face_track == 0 && false == this->_mode_flag_arm_track) {
            this->_mode = RANDOM_MOVE;
        }
        if (args->states.mode_flag_face_track == 1 && this->_mode_flag_arm_track == 0) {
            this->_mode = FACE_TRACK;
        }
        if (true == this->_mode_flag_arm_track) {
            this->_mode = ARM_TRACK;
        }
    }

    if (this->_mode == RANDOM_MOVE) {
        this->_speed_gain = 1.2;
        this->_look_random(args);
    } else {
        this->_random_look_yaw   = 0;
        this->_random_look_pitch = 0;
        this->_random_look_roll  = 0;
        this->_random_look_timer.update();
    }
    if (this->_mode == COMMAND_MOVING) {
        this->_send_yaw   = this->_target_yaw;
        this->_send_pitch = this->_target_pitch + this->_stabilize_pitch;
        this->_send_roll  = this->_target_roll;
    }

    if (this->_mode == FACE_TRACK) {
        this->_speed_gain = 1.5;
        this->_send_yaw   = this->_target_yaw + args->output.neck_face_track.yaw;
        this->_send_pitch = this->_target_pitch + args->output.neck_face_track.pitch + this->_stabilize_pitch;
        this->_send_roll  = this->_target_roll + args->output.neck_face_track.roll;
    } else if (this->_mode == ARM_TRACK) {
        this->_send_yaw   = this->_target_yaw + this->_arm_track_yaw;
        this->_send_pitch = this->_target_pitch + this->_arm_track_pitch + this->_stabilize_pitch;
        this->_send_roll  = this->_target_roll + this->_arm_track_roll;
    } else {
        this->_send_yaw   = this->_target_yaw + this->_random_look_yaw;
        this->_send_pitch = this->_target_pitch + this->_random_look_pitch + this->_stabilize_pitch;
        this->_send_roll  = this->_target_roll + this->_random_look_roll;
    }

    args->output.neck_current.pitch += this->_speed_gain * (this->_send_pitch - args->output.neck_current.pitch) * dt;
    args->output.neck_current.yaw += this->_speed_gain * (this->_send_yaw - args->output.neck_current.yaw) * dt;
    args->output.neck_current.roll += this->_speed_gain * (this->_send_roll - args->output.neck_current.roll) * dt;

    args->output.neck_current.pitch
            = constrain(args->output.neck_current.pitch * this->_N_SCALE, NECK_LIMIT_PITCH[0] * this->_N_SCALE, NECK_LIMIT_PITCH[1] * this->_N_SCALE) / this->_N_SCALE;
    float roll_pitch_limit         = 1.0 - constrain(abs(args->output.neck_current.pitch), 0, this->_ROLL_PITCH_LIMIT) / (float)this->_ROLL_PITCH_LIMIT;
    args->output.neck_current.roll = constrain(args->output.neck_current.roll * this->_N_SCALE,
                                               NECK_LIMIT_ROLL[0] * roll_pitch_limit * this->_N_SCALE,
                                               NECK_LIMIT_ROLL[1] * roll_pitch_limit * this->_N_SCALE)
                                   / this->_N_SCALE;
    args->output.neck_current.yaw
            = constrain(args->output.neck_current.yaw * this->_N_SCALE, NECK_LIMIT_YAW[0] * this->_N_SCALE, NECK_LIMIT_YAW[1] * this->_N_SCALE) / this->_N_SCALE;
    this->_resident_time = micros();
}

void PostureManager_Neck::pose_manager(PostureManagerArguments *args)
{
    if (args->states.pose_command == "unazuku") {
        unazuku(args);
    }
    if (args->states.pose_command == "kasige") {
        kasige(args);
    }
}

void PostureManager_Neck::unazuku(PostureManagerArguments *args)
{
    if (this->_pose_sequence_num == 0) {
        this->_timer_pose_sequence.update();
        this->_pose_sequence_num++;
    }
    if (this->_pose_sequence_num == 1) {
        this->_target_pitch = this->_UNAZUKU_TARGET_PITCH_FACTOR_1 * (1.0 - this->_timer_pose_sequence.get_cos_cycle(1000)) / this->_UNAZUKU_TARGET_PITCH_FACTOR_2;
        if (true == this->_timer_pose_sequence.check_passing(1000)) {
            this->_pose_sequence_num++;
        }
    }
    if (this->_pose_sequence_num == 2) {
        args->states.pose_command = "stay";
        this->_pose_sequence_num  = 0;
    }
}

void PostureManager_Neck::kasige(PostureManagerArguments *args)
{
    if (this->_pose_sequence_num == 0) {
        this->_timer_pose_sequence.update();
        this->_pose_sequence_num++;
    }
    if (this->_pose_sequence_num == 1) {
        this->_target_roll = 4;
        if (true == this->_timer_pose_sequence.check_passing(3500)) {
            this->_target_roll = 0;
            this->_pose_sequence_num++;
        }
    }
    if (this->_pose_sequence_num == 2) {
        args->states.pose_command = "stay";
        this->_pose_sequence_num  = 0;
    }
}

void PostureManager_Neck::set_posture_address(PostureManagerArguments *args)
{
    this->_args = args;
}

bool PostureManager_Neck::_end()
{
    return true;
}

void PostureManager_Neck::set_pose_reset_flag(bool flag)
{
    this->_pose_reset_flag = flag;
}

bool PostureManager_Neck::get_pose_reset_flag()
{
    return this->_pose_reset_flag;
}

void PostureManager_Neck::set_mode_flag_arm_track(bool flag)
{
    this->_mode_flag_arm_track = flag;
}

void PostureManager_Neck::set_target_yaw(float target_yaw)
{
    this->_target_yaw = target_yaw;
}

void PostureManager_Neck::set_mode_flag_command_move(bool flag)
{
    this->_mode_flag_command_move = flag;
}

} // namespace arm_unit
} // namespace maid_robot_system
