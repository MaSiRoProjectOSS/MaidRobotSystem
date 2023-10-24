/**
 * @file movable_arm.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Control movable arm
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/controller/movable_arm.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace arm_unit
{

MovableArm::MovableArm()
{
}

void MovableArm::setup(const int _IDs[JOINT_NUM], const int _ini_pos[JOINT_NUM], const float _scale[JOINT_NUM])
{
    IcsHardSerialClass *krs_pointer;
    this->_krs_hardware.get_krs_address(krs_pointer);

    this->_shoulder_pitch.setup(_IDs[0], _ini_pos[0], _scale[0], krs_pointer);
    this->_shoulder_roll.setup(_IDs[1], _ini_pos[1], _scale[1], krs_pointer);
    this->_elbow_roll.setup(_IDs[2], _ini_pos[2], _scale[2], krs_pointer);
    this->_elbow_pitch.setup(_IDs[3], _ini_pos[3], _scale[3], krs_pointer);
    this->_wrist_roll.setup(_IDs[4], _ini_pos[4], _scale[4], krs_pointer);
    this->_wrist_pitch.setup(_IDs[5], _ini_pos[5], _scale[5], krs_pointer);
    this->_finger_fours.setup(_IDs[6], _ini_pos[6], _scale[6], krs_pointer);
    this->_finger_thumb.setup(_IDs[7], _ini_pos[7], _scale[7], krs_pointer);

    this->_joint[0] = &this->_shoulder_pitch;
    this->_joint[1] = &this->_shoulder_roll;
    this->_joint[2] = &this->_elbow_roll;
    this->_joint[3] = &this->_elbow_pitch;
    this->_joint[4] = &this->_wrist_roll;
    this->_joint[5] = &this->_wrist_pitch;
    this->_joint[6] = &this->_finger_fours;
    this->_joint[7] = &this->_finger_thumb;
    set_all_free();
}

void MovableArm::_kinetic_calc()
{
    float link_twist[JOINT_NUM] = { -this->_joint[0]->get_now_deg() + (float)this->_SEMI_CIRCLE_DEGREE,
                                    0,
                                    -this->_joint[2]->get_now_deg() - (float)this->_QUARTER_CIRCLE_DEGREE,
                                    (float)this->_QUARTER_CIRCLE_DEGREE,
                                    this->_joint[4]->get_now_deg(),
                                    0,
                                    0,
                                    0 }; /* Axial rotation torsion of the link [deg] */
    float link_sita[JOINT_NUM]
            = { (float)this->_QUARTER_CIRCLE_DEGREE, -this->_joint[1]->get_now_deg(), 0, this->_joint[3]->get_now_deg(), 0, 0, 0, 0 }; /* rotation of the link [deg] */

    Matrix<DIMENSION_OF_HOMO_MATRIX> T_m[JOINT_NUM];

    for (int c = 0; c < JOINT_NUM; c++) {
        T_m[c].trans_matrix(this->_link_arm[c], link_twist[c], 0, link_sita[c]);
    }

    Matrix<DIMENSION_OF_HOMO_MATRIX> T_st;
    Matrix<DIMENSION_OF_HOMO_MATRIX> T_cal;
    Vector<DIMENSION_OF_HOMO_MATRIX> arm_root_point;

    float root_point[DIMENSION_OF_HOMO_MATRIX] = { 0, 0, 0, 1 };
    arm_root_point.set_elements(root_point);
    T_st = T_m[0];
    Tensor<DIMENSION_OF_HOMO_MATRIX> tensor_operator;

    hand_point = tensor_operator.Mat_vec_product(arm_root_point, T_st);

    for (int c = 1; c < JOINT_NUM; c++) {
        T_cal.Matrix_product(T_st, T_m[c]);
        T_st       = T_cal;
        hand_point = tensor_operator.Mat_vec_product(arm_root_point, T_st);
    }

    hand_point       = tensor_operator.Mat_vec_product(arm_root_point, T_st);
    this->_hand_y    = hand_point.data[0] * this->_HAND_SCALING_FACTOR;
    this->_hand_z    = hand_point.data[1] * this->_HAND_SCALING_FACTOR;
    this->_hand_x    = hand_point.data[2] * this->_HAND_SCALING_FACTOR;
    this->_hand_sita = atan2(this->_hand_y, this->_hand_x) / PI * (float)this->_SEMI_CIRCLE_DEGREE;
    this->_hand_r    = sqrt(this->_hand_x * this->_hand_x + this->_hand_y * this->_hand_y);
}

void MovableArm::set_all_free()
{
    if (this->_state != HANDSHAKE) {
        this->_state = FREE;
        //this->_arm_posing_finish_flag = 1;
    }
    for (int servo_num = 0; servo_num < JOINT_NUM; servo_num++) {
        this->_joint[servo_num]->set_free();
    }
}

void MovableArm::_save_last_moving_pos()
{
    this->_arm_posing_finish_flag = 1;
    for (int servo_num = 0; servo_num < JOINT_NUM; servo_num++) {
        last_moving_pos[servo_num] = this->_joint[servo_num]->get_now_deg();
    }
}

void MovableArm::move_pos(float tar_pos[JOINT_NUM], int set_time)
{
    if (this->_state != HANDSHAKE) {
        this->_state               = MOVING;
        _MOVING_STATUS moving_flag = _MOVING_STATUS_FREE;

        /* Check if it is free or not. */
        for (int servo_num = 0; servo_num < JOINT_NUM; servo_num++) {
            if (tar_pos[servo_num] != FREE) {
                moving_flag = _MOVING_STATUS_MOVE;
            }
        }
        /* If it is not free, move it. */
        if (moving_flag == _MOVING_STATUS_MOVE) {
            for (int servo_num = 0; servo_num < JOINT_NUM; servo_num++) {
                this->_joint[servo_num]->set_move_time(set_time, tar_pos[servo_num]);
            }
            this->_state = MOVING;
        } else { /* all FREE  ->free mode */

            set_all_free();
        }
    }
}

void MovableArm::set_external_deg(float deg[JOINT_NUM])
{
    for (int servo_num = 0; servo_num < JOINT_NUM; servo_num++) {
        if (deg[servo_num] != 0) {
            this->_joint[servo_num]->set_now_deg(deg[servo_num]);
        }
    }
}

void MovableArm::set_parts_strc(int set_str[JOINT_NUM])
{
    for (int servo_num = 0; servo_num < JOINT_NUM; servo_num++) {
        this->_joint[servo_num]->set_tar_strc(set_str[servo_num]);
        this->_joint[servo_num]->set_strc(set_str[servo_num]);
    }
}

void MovableArm::_set_all_strc(int _strc)
{
    for (int servo_num = 0; servo_num < JOINT_NUM; servo_num++) {
        this->_joint[servo_num]->set_tar_strc(_strc);
        this->_joint[servo_num]->set_strc(_strc);
    }
}

///////////////////////////////////

int MovableArm::_wait_handshake()
{
    this->_set_all_strc(2);

    if (true == this->_waittime_handshake.check_passing(300)) {
        float all_move = 0;
        for (int c = 0; c < JOINT_NUM; c++) {
            all_move += this->_joint[c]->get_now_deg();
        }
        if (this->_old_move != this->_ARM_SERVO_FREE) {
            if (abs(all_move - this->_old_move) > this->_ARM_SERVO_FREE_LIMIT) {
                set_all_free();
                this->_set_all_strc(-1);
                this->_old_move = this->_ARM_SERVO_FREE;
                return 1;
            }
        }
        this->_old_move = all_move;
    }
    return 0;
}

void MovableArm::hand_grip(bool hand_grip_on_off_ref)
{
    if (true == hand_grip_on_off_ref) {
        if (this->_grip_flag == 0) {
            this->_grip_flag         = 1;
            int send_strc[JOINT_NUM] = { -1, -1, -1, -1, 1, -1, 50, 1 };
            set_parts_strc(send_strc);
#ifdef CIRO
            this->_joint[4]->set_move_time(this->_CIRO_JOINT_4_SET_TIME, this->_CIRO_JOINT_4_TAR_POS);
            this->_joint[6]->set_move_time(this->_CIRO_JOINT_6_SET_TIME, this->_CIRO_JOINT_6_TAR_POS);
#endif
#ifdef CIYA
            this->_joint[4]->set_move_time(this->_CIYA_JOINT_4_SET_TIME, this->_CIYA_JOINT_4_TAR_POS);
            this->_joint[6]->set_move_time(this->_CIYA_JOINT_6_SET_TIME, this->_CIYA_JOINT_6_TAR_POS);
#endif
        }
    }
    if (false == hand_grip_on_off_ref && this->_grip_flag == 1) {
        this->_grip_flag         = 2;
        int send_strc[JOINT_NUM] = { -1, -1, -1, -1, 3, -1, 10, 1 };
        set_parts_strc(send_strc);
        this->_joint[4]->set_move_time(2000, FREE);
        this->_joint[6]->set_move_time(1000, 0);
        this->_timer_hand_free.update();
    }
    if (this->_grip_flag == 2 && (true == this->_timer_hand_free.check_passing(2500))) {
        this->_grip_flag = 0;
        this->_joint[6]->set_move_time(2000, FREE);
    }
}

int MovableArm::_wait_posing()
{
    if (this->_pose_lock_flag == 0) {
        float hand_pos[JOINT_NUM] = { FREE, FREE, FREE, FREE, FREE, FREE, FREE, FREE };
        move_pos(hand_pos, 1);

        if (true == this->_waittime_pose_lock.check_passing(500)) {
            float all_move = 0;
            for (int c = 0; c < JOINT_NUM; c++) {
                all_move += this->_joint[c]->get_now_deg();
            }
            if (this->_old_lock_move != this->_ARM_SERVO_FREE) {
                if (abs(all_move - this->_old_lock_move) < 1) {
                    float tar_pos[JOINT_NUM];
                    for (int c = 0; c < JOINT_NUM; c++) {
                        tar_pos[c] = this->_joint[c]->get_now_deg();
                    }
                    this->_set_all_strc(3);
                    move_pos(tar_pos, 1000);
                    this->_old_move       = this->_ARM_SERVO_FREE;
                    this->_pose_lock_flag = 1;
                }
            }
            this->_old_lock_move = all_move;
        }
    }
    if (this->_pose_lock_flag == 1) {
        if (this->_wait_handshake()) {
            this->_pose_lock_flag = 0;
        }
    }
    return this->_pose_lock_flag;
}

void MovableArm::send(PostureManagerArguments *args)
{
    /* start KNDO servo */
    int error_checker = 0;
    for (int c = 0; c < JOINT_NUM; c++) {
        if (this->_joint[c]->get_error_status() == 1) {
            error_checker = 1;
        }
    }
    if (error_checker == 0) {
        for (int c = 0; c < JOINT_NUM; c++) {
            if (this->_joint[c]->get_mode() == FREE) {
                this->_joint[c]->get_deg();
            }
            if (this->_joint[c]->get_mode() == MOVING) {
                float send_data = this->_joint[c]->servo_move_time();
                this->_joint[c]->set_deg(send_data);

                if (this->_joint[c]->get_posing_finish_flag() == 0) {
                    //this->_arm_posing_finish_flag = -1;
                }
            }
        }

        if (true == this->_get_servo_state_timer.check_passing(353)) {
            for (int c = 0; c < JOINT_NUM; c++) {
                this->_joint[c]->get_temp();
            }
        }
    } else {
        /* If there is at least one error, send stop signal at regular intervals.  */

        if (true == this->_send_servo_timer.check_passing(500)) {
            for (int c = 0; c < JOINT_NUM; c++) {
                this->_joint[c]->set_free();
                this->_joint[c]->check_unlock_error();
            }
            this->_state = FREE;
            for (int c = 0; c < JOINT_NUM; c++) {
                if (this->_joint[c]->get_error_status() == 1) {
                }
            }
        }
    }
}

bool MovableArm::_begin()
{
    return true;
}
bool MovableArm::_end()
{
    return true;
}
bool MovableArm::_calculate()
{
    /* mode management */

    if (this->_state != MOVING) {
        if (this->_hand_z > START_Z_UPPER + this->_START_Z_MOVING_OFFSET) {
            this->_state = HANDSHAKE;
        }
    }
    if (this->_state == HANDSHAKE) {
        if (this->_hand_z < START_Z_UPPER - this->_START_Z_HANDSHAKE_OFFSET) {
            this->_state = FREE;
        }
    }
    if (this->_state == MOVING) {
        if (this->_arm_posing_finish_flag == 1) {
            int sum_pose_error = 0;
            for (int ccc = 0; ccc < JOINT_NUM; ccc++) {
                sum_pose_error += abs(last_moving_pos[ccc] - this->_joint[ccc]->get_now_deg());
            }
            if (sum_pose_error > this->_CUMULATIVE_ERROR_FREEING_LIMIT) {
                set_all_free();
                this->_state = FREE;
            }
        }
    }
    this->_kinetic_calc();

    return true;
}

void MovableArm::set_posture_address(PostureManagerArguments *args)
{
    this->_args = args;
}

void MovableArm::set_arm_posing_finish_flag(int arm_posing_finish_flag)
{
    this->_arm_posing_finish_flag = arm_posing_finish_flag;
}

int MovableArm::get_state()
{
    return this->_state;
}

void MovableArm::set_state(int state)
{
    this->_state = state;
}

float MovableArm::get_hand_r()
{
    return this->_hand_r;
}

float MovableArm::get_hand_sita()
{
    return this->_hand_sita;
}

float MovableArm::get_hand_z()
{
    return this->_hand_z;
}

float MovableArm::get_each_joint_now_deg(int joint_num)
{
    return this->_joint[joint_num]->get_now_deg();
}

void MovableArm::set_each_joint_move_time(int joint_num, int set_time, float tar_pos)
{
    this->_joint[joint_num]->set_move_time(set_time, tar_pos);
}

} // namespace arm_unit
} // namespace maid_robot_system
