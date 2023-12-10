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
#ifndef ARM_CONTROLLER_POSTURE_MANAGER_HPP
#define ARM_CONTROLLER_POSTURE_MANAGER_HPP

#include "maid_robot_system/arm_controller/config.hpp"
#include "maid_robot_system/arm_controller/controller/ctrl_hand.hpp"
#include "maid_robot_system/arm_controller/controller/movable_arm.hpp"
#include "maid_robot_system/arm_controller/manager/posture_manager_arguments.hpp"
#include "maid_robot_system/common/interface/control_if.hpp"
#include "maid_robot_system/common/time_check.hpp"
#include "posture_manager_arm.hpp"
#include "posture_manager_leg.hpp"
#include "posture_manager_neck.hpp"
#include "posture_manager_waist.hpp"
#include "posture_manager_wheel.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Manage whole posture.
 *
 */
class PostureManager : public ControlIF {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    PostureManager();

    /*********************************************************
     * Destructor
     *********************************************************/
    ~PostureManager();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief calculate voice in advance.
     *
     * @return
     */
    bool calculate_first();

    /**
     * @brief manage pose
     *
     * @param active_arm : active arm
     * @param passive_arm : passive arm
     * @param R_arm : right arm
     * @param L_arm : left arm
     * @param new_hand : new hand
     */
    void pose_manager(MovableArm *active_arm, MovableArm *passive_arm, MovableArm *R_arm, MovableArm *L_arm, CtrlHand *new_hand);

    /**
     * @brief update vital breath speed
     *
     */
    void update_vital();

    /**
     * @brief Get posture args address
     *
     * @return PostureManagerArguments address
     */
    PostureManagerArguments *get_posture_args_address();

protected:
    /*********************************************************
     * Inherited function
     *********************************************************/
    virtual bool _begin() override;

    virtual bool _end() override;

    virtual bool _calculate() override;

private:
    /*********************************************************
     * Private function
     *********************************************************/
    /**
     * @brief check voice ID
     *
     * @param voice command
     */
    void check_voiceID(int voice_command);

    /**
     * @brief Set pose
     *
     * @param target pose
     */
    void set_pose(String _target_pose);

    /**
     * @brief R walk
     *
     * @param R_arm : right arm
     * @param L_arm : left arm
     */
    void r_walking(MovableArm *R_arm, MovableArm *L_arm);

    /**
     * @brief L walk
     *
     * @param R_arm : right arm
     * @param L_arm : left arm
     */
    void l_walking(MovableArm *R_arm, MovableArm *L_arm);

    /**
     * @brief pose one pose
     *
     * @param active_arm : active arm
     * @param pose1 : pose 1
     * @param send_strc : send strc
     * @param time : pose time
     */
    void pose_onepose(MovableArm *active_arm, float pose1[JOINT_NUM], int send_strc[JOINT_NUM], int time);

    /**
     * @brief pose one pose
     *
     * @param active_arm : active arm
     */
    void pose_onepose(MovableArm *active_arm);

    /**
     * @brief pose memory
     *
     * @param active_arm : active arm
     */
    void pose_memory(MovableArm *active_arm);

    /**
     * @brief set up to turn
     *
     * @param R_arm : right arm
     * @param L_arm : left arm
     */
    void turn_setup(MovableArm *R_arm, MovableArm *L_arm);

    /**
     * @brief turn
     *
     * @param R_arm : right arm
     * @param L_arm : left arm
     */
    void turn(MovableArm *R_arm, MovableArm *L_arm);

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    PostureManagerArguments _args; /*!< PostureManagerArguments object */
    PostureManager_Neck _neck;     /*!< neck object */
    PostureManager_Arm _arm;       /*!< arm object */
    PostureManager_Waist _waist;   /*!< waist object */
    PostureManager_Leg _leg;       /*!< leg object */
    PostureManager_Wheel _wheel;   /*!< wheel object */

    typedef enum _ENUM_POSE_LOCK_STATUS
    {
        _POSE_UNLOCKED = 0,
        _POSE_LOCKED,
        _POSE_LOCK_CANCELLING
    } _POSE_LOCK_STATUS;
    _POSE_LOCK_STATUS _pose_lock_status = _POSE_UNLOCKED; /*!< pose lock status */

    float _pose_move_setup[JOINT_NUM] = {
        -8.27, 8.94, 135.40, 22.11, 3.81, 0.00, 137.19, 0.00,
    };                                                                 /*!< pose move set up */
    int _strc_pose_move_setup[JOINT_NUM] = { 2, 2, 2, 2, 5, 1, 1, 1 }; /*!< strc pose move set up */

    float _pose_seiso[JOINT_NUM] = {
        0.95, 5.97, 57.58, 119.34, 120.0, 0.00, 85.69, 0.00,
    };                                                            /*!< pose of seiso */
    int _strc_pose_seiso[JOINT_NUM] = { 4, 2, 4, 4, 1, 3, 1, 1 }; /*!< strc pose of seiso */

    float _pose_udekata[JOINT_NUM] = {
        26.09, -3.65, 135.74, 119.68, 19.91, 0.00, 70.28, 0.00,
    };                                                               /*!< pose of udekata */
    int _strc_pose_udekata[JOINT_NUM] = { 10, 5, 2, 4, 3, 1, 1, 1 }; /*!< strc pose of udekata */

    int _robot_mode = 0; /*!< robot mode */

    String _pose_command     = "stay"; /*!< pose command */
    String _old_pose_command = "stay"; /*!< previous pose command */
    TimeCheck _timer_pose_sequence;    /*!< timer pose sequence */
    int _pose_sequence_num = 0;        /*!< number of pose sequence */

    TimeCheck _timer_pose_random;            /*!< timer check of random pose */
    unsigned long _time_random_pose = 10000; /*!< time for random pose */

    TimeCheck _timer_pose_stopper;   /*!< time check for pose stopper */
    TimeCheck _timer_memory_wait;    /*!< timer check for memory wait */
    TimeCheck _timer_walking;        /*!< timer check for walking */
    bool _flag_pose_stopper = false; /*!< pose stopper flag */

    int _walking_seq = 0; /*!< walking sequence */

    int _strc[JOINT_NUM] = { 10, 5, 2, 4, 3, 1, 1, 1 }; /*!< strc */

    int _send_strc[JOINT_NUM] = { 20, 10, 10, 5, 5, 1, 10, 5 }; /*!< send strc */

    int _pose_random_move_time = 0; /*!< random pose moving time */

    unsigned long _pose_memory_time = 0;                           /*!< pose memory time */
    float _send_pos[JOINT_NUM];                                    /*!< send position */
    float _joint_save_pos[JOINT_NUM] = { 0, 0, 0, 0, 0, 0, 0, 0 }; /*!< joint save position */
    float _joint_old_pos[JOINT_NUM]  = { 0, 0, 0, 0, 0, 0, 0, 0 }; /*!< previous joint position */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _HANDSHAKE_CTRL_HAND_TARGET_ANGLE = 0; /*!<  */
    const int _HANDSHAKE_CTRL_HAND_TARGET_SPEED = 5; /*!<  */

    const int _NOT_HANDSHAKE_CTRL_HAND_TARGET_ANGLE = 60; /*!<  */
    const int _NOT_HANDSHAKE_CTRL_HAND_TARGET_SPEED = 10; /*!<  */

    const float _CIRO_NECK_ANGLE_FACTOR = 1.5; /*!<  */

    const float _FOLLOW_TARGET_Z_OFFSET = 5.0; /*!<  */

    const int _MOVE_STOP_OFFSET_OF_HAND_Z = 30; /*!<  */

    const int _NOT_HANDSHAKE_POSE_UNLOCKED_RANDOM_MIN = 8000;  /*!<  */
    const int _NOT_HANDSHAKE_POSE_UNLOCKED_RANDOM_MAX = 15000; /*!<  */

    const int _POSE_SEISO_PASSING_TIME   = 1500; /*!<  */
    const int _POSE_UDEKATA_PASSING_TIME = 1800; /*!<  */

    const int _POSE_MOVE_SETUP_TIME         = 1500; /*!<  */
    const int _POSING_PASSING_TIME_FACTOR   = 2;    /*!<  */
    const float _POSE_SEQUENCE_VITAL_FACTOR = 8.0;  /*!<  */

    const int _BREATH_SPEED_TO_MOVE_TIME_FACTOR = 2; /*!<  */

    const float _POSE_MEMORY_HAND_Z_LIMIT = -200.0; /*!<  */

    const int _POSE_MEMORY_TIME_DURATION = 2000; /*!<  */

    const float _POSE_DIFF_SMALL_LIMIT = 10; /*!<  */

    const int _LEG_STEP_ARM_SWING_X_MIN        = -100; /*!<  */
    const int _LEG_STEP_ARM_SWING_X_MAX        = 100;  /*!<  */
    const int _LEG_STEP_ARM_SWING_Y_MIN        = 0;    /*!<  */
    const int _LEG_STEP_ARM_SWING_Y_MAX_FIRST  = -100; /*!<  */
    const int _LEG_STEP_ARM_SWING_Y_MAX_SECOND = 100;  /*!<  */

    const int _R_LEG_STEP_INCREASE_PERCENTAGE_LIMIT = 90; /*!<  */
    const int _L_LEG_STEP_INCREASE_PERCENTAGE_LIMIT = 75; /*!<  */

    const float _R_CENTER_POSE[JOINT_NUM] = {
        0, 24.87, 80.90, 4.93, 11.71, 0.00, 152.96, 0.00,
    }; /*!<  */

    const float _L_CENTER_POSE[JOINT_NUM] = {
        -5, 24.87, 80.90, 4.93, 11.71, 0.00, 152.96, 0.00,
    }; /*!<  */

    const int _TURN_SETUP_MOVE_POS_TIME = 1500; /*!<  */

    const float _ONE_REVOLUTION_DEG = 360.0; /*!<  */

    const float _TURN_NECK_TARGET_YAW_ANGLE = 40;

    const float _THETA_DIFF_SMALL_LIMIT = 10; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
