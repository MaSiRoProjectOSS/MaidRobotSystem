/**
 * @file movable_arm.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Control movable arm
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_MOVABLE_ARM_HPP
#define ARM_CONTROLLER_MOVABLE_ARM_HPP

#include "maid_robot_system/arm_controller/config.hpp"
#include "maid_robot_system/arm_controller/controller/driver/krs_hardware.hpp"
#include "maid_robot_system/arm_controller/controller/driver/krs_servo.hpp"
#include "maid_robot_system/arm_controller/manager/posture_manager_arguments.hpp"
#include "maid_robot_system/common/interface/control_if.hpp"
#include "maid_robot_system/common/move_average.hpp"
#include "maid_robot_system/common/time_check.hpp"
#include "maid_robot_system/math/matrix.hpp"

#include <Arduino.h>


namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief MovableArm Class
 *
 */
class MovableArm : public ControlIF {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    MovableArm();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief set up
     *
     * @param _IDs : ID
     * @param _ini_pos : initial position
     * @param _scale : scale
     */
    void setup(const int _IDs[JOINT_NUM], const int _ini_pos[JOINT_NUM], const float _scale[JOINT_NUM]);

    /**
     * @brief Set PostureManagerArguments address
     *
     * @param PostureManagerArguments address
     */
    void set_posture_address(PostureManagerArguments *args);

    /**
     * @brief send signal
     *
     * @param PostureManagerArguments object
     */
    void send(PostureManagerArguments *args);

    /**
     * @brief Set external deg
     *
     * @param external deg
     */
    void set_external_deg(float deg[JOINT_NUM]);

    /**
     * @brief hand grip
     *
     * @param reference of gripping
     */
    void hand_grip(bool hand_grip_on_off_ref);

    /**
     * @brief Set all free
     *
     */
    void set_all_free();

    /**
     * @brief Set parts strc
     *
     * @param set_str :
     */
    void set_parts_strc(int set_str[JOINT_NUM]);

    /**
     * @brief move position
     *
     * @param tar_pos : target position
     * @param set_time : set time
     */
    void move_pos(float tar_pos[JOINT_NUM], int set_time);

    /**
     * @brief Set arm posing finish flag
     *
     * @param arm_posing_finish_flag
     */
    void set_arm_posing_finish_flag(int arm_posing_finish_flag);

    /**
     * @brief Get state
     *
     * @return state
     */
    int get_state();

    /**
     * @brief Set state
     *
     * @param state
     */
    void set_state(int state);

    /**
     * @brief Get the hand r
     *
     * @return hand_r
     */
    float get_hand_r();

    /**
     * @brief Get the hand sita
     *
     * @return hand_sta
     */
    float get_hand_sita();

    /**
     * @brief Get the hand z
     *
     * @return hand_z
     */
    float get_hand_z();

    /**
     * @brief Get the each joint current angle
     *
     * @param joint_num : number of each joint
     * @return joint current angle
     */
    float get_each_joint_now_deg(int joint_num);

    /**
     * @brief Set the each joint move time
     *
     * @param joint_num : number of each joint
     * @param set_time : set time
     * @param tar_pos : target position
     */
    void set_each_joint_move_time(int joint_num, int set_time, float tar_pos);

protected:
    /*********************************************************
     * Inherited function
     *********************************************************/
    bool _begin() override;
    bool _end() override;
    bool _calculate() override;

private:
    /*********************************************************
     * Private function
     *********************************************************/
    /**
     * @brief calculate kinematic model.
     *
     */
    void _kinetic_calc();

    /**
     * @brief save last moving position
     *
     */
    void _save_last_moving_pos();

    /**
     * @brief set all strc
     *
     * @param _strc
     */
    void _set_all_strc(int _strc);

    /**
     * @brief wait handshake
     *
     * @return
     */
    int _wait_handshake();

    /**
     * @brief wait posing
     *
     * @return
     */
    int _wait_posing();

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    typedef enum ENUM_MOVING_STATUS
    {
        _MOVING_STATUS_FREE = 0,
        _MOVING_STATUS_MOVE
    } _MOVING_STATUS;

    KRSServo _shoulder_pitch; /*!<  */
    KRSServo _shoulder_roll;  /*!<  */
    KRSServo _elbow_roll;     /*!<  */
    KRSServo _elbow_pitch;    /*!<  */
    KRSServo _wrist_roll;     /*!<  */
    KRSServo _wrist_pitch;    /*!<  */
    KRSServo _finger_fours;   /*!<  */
    KRSServo _finger_thumb;   /*!<  */

    KRSServo *_joint[JOINT_NUM] = { nullptr }; /*!<  */
    TimeCheck _send_servo_timer;               /*!<  */
    int _arm_posing_finish_flag = 1;           /*!<  */

    float last_moving_pos[JOINT_NUM] = { 0.0f }; /*!<  */

    float joint_speed[JOINT_NUM] = { 0.0f }; /*!<  */
    float sum_joint_speed        = 0;        /*!<  */

    float joint_temp[JOINT_NUM]    = { 0.0f }; /*!<  */
    float joint_current[JOINT_NUM] = { 0.0f }; /*!<  */
    int _state                     = 0;        /*!<  */

    Vector<DIMENSION_OF_HOMO_MATRIX> hand_point; /*!<  */
    float _hand_x    = 0;                        /*!<  */
    float _hand_y    = 0;                        /*!<  */
    float _hand_z    = 0;                        /*!<  */
    float _hand_sita = 0;                        /*!<  */
    float _hand_r    = 0;                        /*!<  */

    int tar_strc = -1; /*!<  */

    /* Kinematic model calculation */
    const float _link_arm[JOINT_NUM] = { 0.0, 0.0, 0.134, 0.0, 0.145, 0, 0, 0 }; /*!< link arm length [m] */

    TimeCheck _waittime_handshake; /*!<  */
    float _old_move = 111111;      /*!<  */

    TimeCheck _waittime_pose_lock; /*!<  */
    float _old_lock_move = 111111; /*!<  */
    int _pose_lock_flag  = 0;      /*!<  */

    int _grip_flag = 0; /*!<  */

    TimeCheck _timer_hand_free; /*!<  */

    TimeCheck _get_servo_state_timer; /*!<  */

    KRSHardware _krs_hardware;      /*!<  */
    PostureManagerArguments *_args; /*!<  */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _SEMI_CIRCLE_DEGREE    = 180; /*!<  */
    const int _QUARTER_CIRCLE_DEGREE = 90;  /*!<  */

    const float _HAND_SCALING_FACTOR = 1000.0; /*!<  */

    const int _ARM_SERVO_FREE       = 111111; /*!<  */
    const int _ARM_SERVO_FREE_LIMIT = 7;      /*!<  */

    const int _CIRO_JOINT_4_SET_TIME = 2000; /*!<  */
    const int _CIRO_JOINT_4_TAR_POS  = 10;   /*!<  */
    const int _CIRO_JOINT_6_SET_TIME = 2000; /*!<  */
    const int _CIRO_JOINT_6_TAR_POS  = 100;  /*!<  */
    const int _CIYA_JOINT_4_SET_TIME = 2000; /*!<  */
    const int _CIYA_JOINT_4_TAR_POS  = 180;  /*!<  */
    const int _CIYA_JOINT_6_SET_TIME = 2000; /*!<  */
    const int _CIYA_JOINT_6_TAR_POS  = 90;   /*!<  */

    const int _START_Z_MOVING_OFFSET    = 30; /*!<  */
    const int _START_Z_HANDSHAKE_OFFSET = 10; /*!<  */

    const int _CUMULATIVE_ERROR_FREEING_LIMIT = 25; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
