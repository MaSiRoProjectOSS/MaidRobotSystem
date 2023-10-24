/**
 * @file posture_manager.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Manage whole posture.
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/manager/posture_manager.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

void PostureManager::set_pose(String _target_pose)
{
    this->_old_pose_command = this->_pose_command;
    this->_pose_command     = _target_pose;
    if (this->_old_pose_command != this->_pose_command)
        this->_pose_sequence_num = 0;
}

void PostureManager::pose_manager(MovableArm *active_arm, MovableArm *passive_arm, MovableArm *R_arm, MovableArm *L_arm, CtrlHand *new_hand)
{
    /* hand shaking walk program */
    if (passive_arm->get_state() == HANDSHAKE) {
        this->_robot_mode = MODE_FOLLOW;
    }

    if (this->_robot_mode == MODE_FOLLOW) {
        if (passive_arm->get_state() == HANDSHAKE) {
            this->_args.output.neck_angle.set(0, 0, 0);
            this->_wheel.set_follow_target(&this->_args, passive_arm->get_hand_r(), passive_arm->get_hand_sita(), passive_arm->get_hand_z());

#ifdef CIRO
            this->l_walking(passive_arm, active_arm);
#elif CIYA
            passive_arm->hand_grip(true);
            new_hand->move_along_trajectory(this->_HANDSHAKE_CTRL_HAND_TARGET_ANGLE, this->_HANDSHAKE_CTRL_HAND_TARGET_SPEED);

            this->r_walking(active_arm, passive_arm);
#endif
            this->_neck.set_pose_reset_flag(true);
        } else {
#ifdef CIRO
            passive_arm->set_state((int)FREE);
#endif
        }

        if (passive_arm->get_state() != HANDSHAKE) {
            if (true == this->_neck.get_pose_reset_flag()) {
                this->_neck.reset(&this->_args);
                this->_neck.set_pose_reset_flag(false);
#ifdef CIYA
                this->_args.states.pose_command = "kasige";
                active_arm->hand_grip(false);
                passive_arm->hand_grip(false);
#endif
                active_arm->set_all_free();
            }
#ifdef CIYA
            new_hand->move_along_trajectory(this->_NOT_HANDSHAKE_CTRL_HAND_TARGET_ANGLE, this->_NOT_HANDSHAKE_CTRL_HAND_TARGET_SPEED);
#endif

            this->_neck.set_mode_flag_arm_track(false);
#ifdef CIRO
            this->_args.output.neck_angle.set(this->_args.output.neck_current.yaw,
                                              this->_args.output.neck_current.pitch,
                                              this->_args.output.neck_current.roll / this->_CIRO_NECK_ANGLE_FACTOR);
#elif CIYA
            this->_args.output.neck_angle.set(this->_args.output.neck_current.yaw, this->_args.output.neck_current.pitch, this->_args.output.neck_current.roll);
#endif
            this->_wheel.set_follow_target(&this->_args, this->_wheel.get_target_r(), this->_wheel.get_target_s(), START_Z_UPPER - this->_FOLLOW_TARGET_Z_OFFSET);
            this->_wheel.move_stop(&this->_args);
        }
        this->_wheel.handshake_follow(&this->_args);
    }
    if (passive_arm->get_hand_z() < START_Z_UPPER - this->_MOVE_STOP_OFFSET_OF_HAND_Z) {
        this->_wheel.move_stop(&this->_args);
    }

    /* Don't make pose at the first time for transporting. */
    if (true == this->_timer_pose_stopper.check_passing(60000 * 3)) {
        this->_flag_pose_stopper = true;
    }
    if (false == this->_flag_pose_stopper) {
        return;
    }

    if (this->_args.states.leg_mode != STAND) {
        this->_pose_command = "stay";
    }

    if (passive_arm->get_state() == HANDSHAKE) {
        if (this->_pose_command != "stay") {
            this->_pose_command = "reset";
        }
    }

    if (active_arm->get_state() != HANDSHAKE && this->_pose_lock_status == _POSE_UNLOCKED) {
        if (true == this->_timer_pose_random.check_passing(this->_time_random_pose)) {
            this->_time_random_pose = random(this->_NOT_HANDSHAKE_POSE_UNLOCKED_RANDOM_MIN, this->_NOT_HANDSHAKE_POSE_UNLOCKED_RANDOM_MAX);
            switch (random(0, 5)) {
                case 0:
                    set_pose("move_setup");
                    break;
                case 1:
                    set_pose("pose_seiso");
                    break;
                case 2:
                    set_pose("pose_udeyoko");
                    break;
                case 3:
                    set_pose("pose_udekata");
                    break;
                case 4:
                    set_pose("pose_udemise");
                    break;
            }
        }
    } else {
        this->_timer_pose_random.update();
    }

    if (this->_pose_command == "turn_setup") {
        this->turn_setup(R_arm, L_arm);
    }
    if (this->_pose_command == "turn") {
        this->turn(R_arm, L_arm);
    }
    if (this->_pose_command == "move_setup") {
        /* pose_onepose(this->_pose_move_setup, this->_strc_pose_move_setup, 1500); */
        this->pose_onepose(active_arm);
    }
    if (this->_pose_command == "pose_seiso") {
        this->pose_onepose(active_arm, this->_pose_seiso, this->_strc_pose_seiso, this->_POSE_SEISO_PASSING_TIME);
    }
    if (this->_pose_command == "pose_udeyoko") {
        /* pose_onepose(this->_pose_udeyoko , this->_strc_pose_udeyoko,1500); */
    }
    if (this->_pose_command == "pose_udekata") {
        this->pose_onepose(active_arm, this->_pose_udekata, this->_strc_pose_udekata, this->_POSE_UDEKATA_PASSING_TIME);
    }
    if (this->_pose_command == "pose_udemise") {
        /* pose_onepose(_pose_udemise , _strc_pose_udemise,1800); */
    }
    if (this->_pose_command == "pose_memory") {
        this->pose_memory(active_arm);
    }

    if (this->_pose_command == "reset") {
        this->_pose_command = "stay";
        this->_wheel.set_flag_yaw_control(false);
        this->_args.states.waist_target.yaw = 0;
        this->_neck.set_target_yaw((float)0.0);
        this->_neck.set_mode_flag_command_move(false);
        this->_args.states.waist_speed_gain = 1.0;
        R_arm->set_all_free();
        this->_pose_sequence_num = 0;
    }
    if (this->_pose_command == "stay") {
        this->_robot_mode = MODE_FOLLOW;
    } else {
        this->_robot_mode = MODE_POSING;
    }
    this->_robot_mode = MODE_FOLLOW; /* For Robocap. Somehow it starts with POSING. */
}

void PostureManager::pose_onepose(MovableArm *active_arm, float pose1[JOINT_NUM], int send_strc[JOINT_NUM], int time)
{
    if (this->_pose_sequence_num == 0) {
        this->_timer_pose_sequence.update();
        active_arm->set_arm_posing_finish_flag(0);
        this->_pose_sequence_num++;
        active_arm->set_parts_strc(this->_strc_pose_move_setup);
        active_arm->move_pos(this->_pose_move_setup, this->_POSE_MOVE_SETUP_TIME);
    }

    if (this->_pose_sequence_num == 1) {
        if (true == this->_timer_pose_sequence.check_passing(time)) {
            this->_pose_sequence_num++;
            active_arm->set_parts_strc(send_strc);
            active_arm->move_pos(pose1, time);
        }
    }
    if (this->_pose_sequence_num == 2) {
        if (true == this->_timer_pose_sequence.check_passing(time * this->_POSING_PASSING_TIME_FACTOR)) {
            this->_pose_sequence_num++;
        }
    }
    if (this->_pose_sequence_num == 3) {
        float vital_move[JOINT_NUM] = { (float)(this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                        (float)(this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                        (float)(this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                        (float)(this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                        (float)(this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                        (float)(0.00),
                                        (float)(137.19),
                                        (float)(0.00) };
        float send_move[JOINT_NUM];
        for (int cc = 0; cc < JOINT_NUM; cc++) {
            send_move[cc] = pose1[cc] + vital_move[cc];
        }
        active_arm->move_pos(send_move, 1);
    }
}

void PostureManager::pose_onepose(MovableArm *active_arm)
{
    if (this->_pose_sequence_num == 0) {
        this->_timer_pose_sequence.update();
        active_arm->set_arm_posing_finish_flag(0);
        this->_pose_sequence_num++;
        active_arm->set_parts_strc(this->_strc_pose_move_setup);
        active_arm->move_pos(this->_pose_move_setup, this->_POSE_MOVE_SETUP_TIME);
    }

    if (this->_pose_sequence_num == 1) {
        if (true == this->_timer_pose_sequence.check_passing(1500)) {
            this->_pose_sequence_num++;
        }
    }

    if (this->_pose_sequence_num == 2) {
        this->_pose_sequence_num++;
        float _pose[JOINT_NUM]       = { (float)(-8.27 + random(0, 8)),     //
                                         (float)(8.94 + random(-5, 10)),    //
                                         (float)(135.40 + random(-15, 15)), //
                                         (float)(22.11 + random(-15, 15)),  //
                                         (float)(3.81 + random(-20, 20)),   //
                                         (float)(0.00),                     //
                                         (float)(137.19),                   //
                                         (float)(0.00) };
        this->_pose_random_move_time = this->_args.states.breath_speed / this->_BREATH_SPEED_TO_MOVE_TIME_FACTOR;
        active_arm->set_parts_strc(this->_strc);
        active_arm->move_pos(_pose, this->_pose_random_move_time);
    }
    if (this->_pose_sequence_num == 3) {
        if (true == this->_timer_pose_sequence.check_passing(this->_pose_random_move_time)) {
            this->_pose_sequence_num++;
        }
    }
    if (this->_pose_sequence_num == 4) {
        float _pose[JOINT_NUM] = { (float)(-8.27 + this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                   (float)(8.94 + this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                   (float)(135.40 + this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                   (float)(22.11 + this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                   (float)(3.81 + this->_POSE_SEQUENCE_VITAL_FACTOR * this->_args.states.vital),
                                   (float)(0.00, 137.19),
                                   (float)(0.00) };

        active_arm->set_parts_strc(this->_strc);
        active_arm->move_pos(_pose, 1);
    }
}

void PostureManager::pose_memory(MovableArm *active_arm)
{
    static int sum_pose_error = 0;

    if (active_arm->get_hand_z() > this->_POSE_MEMORY_HAND_Z_LIMIT) {
        if (millis() - this->_pose_memory_time > this->_POSE_MEMORY_TIME_DURATION) {
            this->_pose_memory_time = millis();
            for (int ccc = 0; ccc < JOINT_NUM; ccc++) {
                this->_joint_save_pos[ccc] = active_arm->get_each_joint_now_deg(ccc);
            }
            sum_pose_error = 0;
            for (int ccc = 0; ccc < JOINT_NUM; ccc++) {
                sum_pose_error += abs(this->_joint_save_pos[ccc] - this->_joint_old_pos[ccc]);
                this->_joint_old_pos[ccc] = this->_joint_save_pos[ccc];
            }
            if (this->_pose_lock_status == _POSE_UNLOCKED) {
                if (active_arm->get_hand_z() > this->_POSE_MEMORY_HAND_Z_LIMIT) {
                    if (sum_pose_error < this->_POSE_DIFF_SMALL_LIMIT) {
                        active_arm->get_state() == POSE_LOCK;
                        for (int ccc = 0; ccc < JOINT_NUM; ccc++) {
                            _send_pos[ccc] = this->_joint_save_pos[ccc];
                        }
                        active_arm->set_state((int)MOVING);
                        int send_strc[JOINT_NUM] = { 2, 2, 3, 3, 1, 1, 1, 5 };
                        active_arm->set_parts_strc(send_strc);
                        active_arm->move_pos(_send_pos, 2000);
                        active_arm->set_each_joint_move_time(6, 1000, 50);
                        active_arm->set_arm_posing_finish_flag(0);
                        this->_pose_lock_status = _POSE_LOCKED;
                        this->_timer_memory_wait.update();
                    }
                }
            } else if ((true == this->_timer_memory_wait.check_passing(2000)) && this->_pose_lock_status == _POSE_LOCKED) {
                this->_pose_lock_status = _POSE_LOCK_CANCELLING;
            } else if (this->_pose_lock_status == _POSE_LOCK_CANCELLING) {
                if (active_arm->get_hand_z() > this->_POSE_MEMORY_HAND_Z_LIMIT) {
                    if (sum_pose_error > 20) {
                        active_arm->get_state() == FREE;
                        active_arm->set_all_free();
                        active_arm->set_each_joint_move_time(6, 1000, 0);
                        this->_pose_lock_status = _POSE_UNLOCKED;
                    }
                }
            }
        }
    }
}
////////////////////////////////////////////

void PostureManager::r_walking(MovableArm *R_arm, MovableArm *L_arm)
{
    int dt       = this->_timer_walking.get_lap_time();
    int now_step = this->_args.input.leg_step_percentage;

    float arm_swing;

    if (this->_walking_seq == 0 || this->_walking_seq == 1) {
        arm_swing = map(this->_args.input.leg_step_percentage,
                        this->_LEG_STEP_ARM_SWING_X_MIN,
                        this->_LEG_STEP_ARM_SWING_X_MAX,
                        this->_LEG_STEP_ARM_SWING_Y_MIN,
                        this->_LEG_STEP_ARM_SWING_Y_MAX_FIRST);
        if (this->_walking_seq == 0 && this->_args.input.leg_step_percentage > this->_R_LEG_STEP_INCREASE_PERCENTAGE_LIMIT) {
            this->_walking_seq++;
        }
        if (this->_walking_seq == 1 && this->_args.input.leg_step_percentage < -this->_R_LEG_STEP_INCREASE_PERCENTAGE_LIMIT) {
            this->_walking_seq++;
        }
    }
    if (this->_walking_seq == 2 || this->_walking_seq == 3) {
        arm_swing = map(this->_args.input.leg_step_percentage,
                        this->_LEG_STEP_ARM_SWING_X_MIN,
                        this->_LEG_STEP_ARM_SWING_X_MAX,
                        this->_LEG_STEP_ARM_SWING_Y_MIN,
                        this->_LEG_STEP_ARM_SWING_Y_MAX_SECOND);
        if (this->_walking_seq == 2 && this->_args.input.leg_step_percentage > this->_R_LEG_STEP_INCREASE_PERCENTAGE_LIMIT) {
            this->_walking_seq++;
        }
        if (this->_walking_seq == 3 && this->_args.input.leg_step_percentage < -this->_R_LEG_STEP_INCREASE_PERCENTAGE_LIMIT) {
            this->_walking_seq = 0;
        }
    }
    float send_pos[JOINT_NUM];
    for (int ccc = 0; ccc < JOINT_NUM; ccc++) {
        send_pos[ccc] = L_arm->get_each_joint_now_deg(ccc);
    }

    /* Swing the arm while walking. */
    send_pos[0] = this->_R_CENTER_POSE[0] + 35 * arm_swing / 100.0; //map (leg.now_step_percentage , 0,100,0,20);
    send_pos[1] = this->_R_CENTER_POSE[1] + map(this->_args.input.wheel_target_w * 10.0, 0, 10, 0, -20);
    send_pos[2] = this->_R_CENTER_POSE[2];
    send_pos[3] = this->_R_CENTER_POSE[3] + constrain(map(arm_swing, 0, 100, 0, 30), 0, 30);
    send_pos[4] = this->_R_CENTER_POSE[4];

    R_arm->set_state((int)MOVING);

    R_arm->set_parts_strc(this->_send_strc);
    R_arm->move_pos(send_pos, 5);
}

void PostureManager::l_walking(MovableArm *R_arm, MovableArm *L_arm)
{
    set_pose("l_walking");
    int dt       = this->_timer_walking.get_lap_time();
    int now_step = this->_args.input.leg_step_percentage;

    float arm_swing;

    if (this->_walking_seq == 0 || this->_walking_seq == 1) {
        arm_swing = map(this->_args.input.leg_step_percentage,
                        this->_LEG_STEP_ARM_SWING_X_MIN,
                        this->_LEG_STEP_ARM_SWING_X_MAX,
                        this->_LEG_STEP_ARM_SWING_Y_MIN,
                        this->_LEG_STEP_ARM_SWING_Y_MAX_FIRST);
        if (this->_walking_seq == 0 && this->_args.input.leg_step_percentage > this->_L_LEG_STEP_INCREASE_PERCENTAGE_LIMIT) {
            this->_walking_seq++;
        }
        if (this->_walking_seq == 1 && this->_args.input.leg_step_percentage < -this->_L_LEG_STEP_INCREASE_PERCENTAGE_LIMIT) {
            this->_walking_seq++;
        }
    }
    if (this->_walking_seq == 2 || this->_walking_seq == 3) {
        arm_swing = map(this->_args.input.leg_step_percentage,
                        this->_LEG_STEP_ARM_SWING_X_MIN,
                        this->_LEG_STEP_ARM_SWING_X_MAX,
                        this->_LEG_STEP_ARM_SWING_Y_MIN,
                        this->_LEG_STEP_ARM_SWING_Y_MAX_SECOND);
        if (this->_walking_seq == 2 && this->_args.input.leg_step_percentage > this->_L_LEG_STEP_INCREASE_PERCENTAGE_LIMIT) {
            this->_walking_seq++;
        }
        if (this->_walking_seq == 3 && this->_args.input.leg_step_percentage < -this->_L_LEG_STEP_INCREASE_PERCENTAGE_LIMIT) {
            this->_walking_seq = 0;
        }
    }

    float send_pos[JOINT_NUM];
    for (int ccc = 0; ccc < JOINT_NUM; ccc++) {
        send_pos[ccc] = L_arm->get_each_joint_now_deg(ccc);
    }

    /* Swing the arm while walking. */

    send_pos[0] = this->_L_CENTER_POSE[0] + 45 * arm_swing / 100.0; //map (leg.now_step_percentage , 0,100,0,20);
    send_pos[1] = this->_L_CENTER_POSE[1] + map(this->_args.input.wheel_target_w * 10.0, 0, 10, 0, -20);
    send_pos[2] = this->_L_CENTER_POSE[2];
    send_pos[3] = this->_L_CENTER_POSE[3] + constrain(map(arm_swing, 0, 100, 0, 30), 0, 30);
    send_pos[4] = this->_L_CENTER_POSE[4];

    L_arm->set_state((int)MOVING);
    L_arm->set_parts_strc(this->_send_strc);
    L_arm->move_pos(send_pos, 5);
}

void PostureManager::turn_setup(MovableArm *R_arm, MovableArm *L_arm)
{
    if (this->_pose_sequence_num == 0) {
        this->_timer_pose_sequence.update();
        this->_args.states.pose_command = "unazuku";
        this->_pose_sequence_num++;
        float pose1[JOINT_NUM] = { -3.51, 10.34, 123.53, 3.31, 181.07, 0.00, 131.15, 0.00 };
        R_arm->move_pos(pose1, this->_TURN_SETUP_MOVE_POS_TIME);
    }
    if (this->_pose_sequence_num == 1) {
        if (true == this->_timer_pose_sequence.check_passing(1500)) {
            this->_pose_sequence_num++;
        }
    }
    if (this->_pose_sequence_num == 2) {
        this->_args.states.waist_target.yaw = -40;
        this->_neck.set_mode_flag_command_move(true);
        this->_neck.set_target(40, this->_args.output.neck_current.pitch, 0);
        float pose1[JOINT_NUM] = { 56.67, -11.07, 26.70, 103.11, 50.74, 0.00, 125.15, 0.00 };
        R_arm->move_pos(pose1, this->_TURN_SETUP_MOVE_POS_TIME);
        this->_leg.set_target_pos(-50);

        this->_pose_sequence_num++;
    }
    if (this->_pose_sequence_num == 3) {
        if (true == this->_timer_pose_sequence.check_passing(3000)) {
            this->_pose_sequence_num++;
        }
    }
    if (this->_pose_sequence_num == 4) {
        this->_pose_command = "standby";

        this->_pose_sequence_num = 0;
    }
}

void PostureManager::turn(MovableArm *R_arm, MovableArm *L_arm)
{
    if (this->_pose_sequence_num == 0) {
        this->_timer_pose_sequence.update();
        this->_neck.set_mode_flag_command_move(true);
        this->_neck.set_target(this->_TURN_NECK_TARGET_YAW_ANGLE, this->_args.output.neck_current.pitch, 0);

        this->_wheel.set_target_theta(this->_args.input.wheel_theta + this->_ONE_REVOLUTION_DEG);
        this->_wheel.set_flag_yaw_control(true);
        float pose1[JOINT_NUM] = { -30.11, 35.50, 123.83, 37.02, 10.89, 0.00, 125.15, 0.00 };
        R_arm->move_pos(pose1, 700);
        this->_leg.set_target_pos(0);

        this->_pose_sequence_num++;
    }
    if (this->_pose_sequence_num == 1) {
        if (true == this->_timer_pose_sequence.check_passing(100)) {
            this->_args.states.waist_speed_gain = 3.0;
            this->_args.states.waist_target.yaw = 40;
            this->_neck.set_target_yaw(this->_TURN_NECK_TARGET_YAW_ANGLE);

            this->_pose_sequence_num++;
        }
    }

    if (this->_pose_sequence_num == 2) {
        int error_theta                     = abs(this->_wheel.get_target_theta() - this->_args.input.wheel_theta);
        this->_args.states.waist_target.yaw = constrain(error_theta * 0.5, 0, 40);
        this->_neck.set_target_yaw(constrain(error_theta * 0.5, 0, 40));

        if (error_theta < this->_THETA_DIFF_SMALL_LIMIT || (true == this->_timer_pose_sequence.check_passing(5000))) {
            this->_pose_sequence_num++;
            float pose1[JOINT_NUM] = { -3.51, 10.34, 123.53, 3.31, 181.07, 0.00, 131.15, 0.00 };
            R_arm->move_pos(pose1, 1500);
        }
    }
    if (this->_pose_sequence_num == 3) {
        if (true == this->_timer_pose_sequence.check_passing(3000)) {
            R_arm->set_all_free();
            this->_pose_sequence_num++;
        }
    }
    if (this->_pose_sequence_num == 4) {
        if (true == this->_timer_pose_sequence.check_passing(3000)) {
            this->_pose_sequence_num++;
        }
    }

    if (this->_pose_sequence_num == 5) {
        this->_pose_command = "stay";
        this->_wheel.set_flag_yaw_control(false);
        this->_args.states.waist_target.yaw = 0;
        this->_neck.set_target_yaw((float)0);
        this->_neck.set_mode_flag_command_move(false);
        this->_args.states.waist_speed_gain = 1.0;
        R_arm->set_all_free();
        this->_pose_sequence_num = 0;
    }
}

PostureManager::PostureManager()
{
}
PostureManager::~PostureManager()
{
}

bool PostureManager::_begin()
{
    this->_pose_command = "move_setup";

    this->_arm.begin();
    this->_leg.begin();
    this->_neck.begin();
    this->_wheel.begin();
    this->_waist.begin();

    this->_args.states.vital_timer.update();
    return true;
}

bool PostureManager::_end()
{
    return true;
}

bool PostureManager::calculate_first()
{
    bool result = true;
    this->check_voiceID(this->_args.input.voice_command);
    return result;
}
bool PostureManager::_calculate()
{
    this->_neck.set_posture_address(&this->_args);
    this->_neck.calculate();

    this->_arm.set_posture_address(&this->_args);
    this->_arm.calculate();

    this->_leg.set_posture_address(&this->_args);
    this->_leg.calculate();

    this->_wheel.set_posture_address(&this->_args);
    this->_wheel.calculate();

    /* Waist uses neck information, so it is done after neck. */
    /* Reading the various settings that have been calculated for the poses? */
    this->_waist.set_posture_address(&this->_args);
    this->_waist.calculate();

    return true;
}

void PostureManager::check_voiceID(int voice_command)
{
    static int next_movement = 0;
    static TimeCheck timer_next_movement_reset;
    static int voiceID_flag_turn       = 15;
    static int voiceID_name_ciro       = 23;
    static int voiceID_name_ciya       = 24;
    static int voiceID_flag_move_start = 25;

    ////////////////////////////////////
    if (true == timer_next_movement_reset.check_passing(30000)) {
        next_movement = 0;
    }

    if (this->_args.input.voice_command != 0) {
        if (voice_command == voiceID_name_ciya) {
            this->_args.states.pose_command = "kasige";
        }
        if (voice_command == voiceID_name_ciro) {
            this->_args.states.pose_command = "kasige";
        }

        if (voice_command == voiceID_flag_turn) {
            next_movement = voiceID_flag_turn;
            timer_next_movement_reset.update();

            this->_pose_command = "turn_setup";
        }

        if (voice_command == voiceID_flag_move_start) {
            if (next_movement == voiceID_flag_turn) {
                this->_pose_command = "turn";
            }
        }

        ///////////////////////////////////////////////////
        this->_args.input.voice_command = 0;
    }
}

void PostureManager::update_vital()
{
    this->_args.states.vital = this->_args.states.vital_timer.get_sin_cycle(this->_args.states.breath_speed);
}

PostureManagerArguments *PostureManager::get_posture_args_address()
{
    return &this->_args;
}

} // namespace arm_unit
} // namespace maid_robot_system
