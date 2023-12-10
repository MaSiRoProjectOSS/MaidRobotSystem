/**
 * @file posture_manager_neck.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Manage posture of neck.
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_POSTURE_MANAGER_NECK_HPP
#define ARM_CONTROLLER_POSTURE_MANAGER_NECK_HPP

#include "maid_robot_system/arm_controller/config.hpp"
#include "maid_robot_system/common/interface/control_if.hpp"
#include "maid_robot_system/common/time_check.hpp"
#include "maid_robot_system/common/types/coordinate_euler.hpp"
#include "posture_manager_arguments.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Manage posture of neck.
 *
 */
class PostureManager_Neck : public ControlIF {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    PostureManager_Neck();

    /*********************************************************
     * Destructor
     *********************************************************/
    ~PostureManager_Neck();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief Set the target object
     * 
     * @param _yaw 
     * @param _pitch 
     * @param _roll 
     */
    void set_target(float _yaw, float _pitch, float _roll);

    /**
     * @brief 
     * 
     * @param args 
     */
    void reset(PostureManagerArguments *args);

    /**
     * @brief 
     * 
     * @param args 
     */
    void pose_manager(PostureManagerArguments *args);

    /**
     * @brief 
     * 
     * @param args 
     */
    void unazuku(PostureManagerArguments *args);

    /**
     * @brief 
     * 
     * @param args 
     */
    void kasige(PostureManagerArguments *args);

    /**
     * @brief Set the posture address object
     *
     * @param PostureManagerArguments address
     */
    void set_posture_address(PostureManagerArguments *args);

    /**
     * @brief Set the pose reset flag object
     * 
     * @param flag 
     */
    void set_pose_reset_flag(bool flag);

    /**
     * @brief Get the pose reset flag object
     * 
     * @return true 
     * @return false 
     */
    bool get_pose_reset_flag();

    /**
     * @brief Set the mode flag arm track object
     * 
     * @param flag 
     */
    void set_mode_flag_arm_track(bool flag);

    /**
     * @brief Set the target yaw object
     * 
     * @param target_yaw 
     */
    void set_target_yaw(float target_yaw);

    /**
     * @brief Set the mode flag command move object
     * 
     * @param flag 
     */
    void set_mode_flag_command_move(bool flag);

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
     * @brief 
     * 
     * @param args 
     * @param _hand_x 
     * @param _hand_y 
     * @param _hand_z 
     */
    void _set_hand_track(PostureManagerArguments *args, float _hand_x, float _hand_y, float _hand_z);

    /**
     * @brief 
     * 
     * @param args 
     */
    void _look_random(PostureManagerArguments *args);

    /**
     * @brief 
     * 
     */
    void _setup();

    /**
     * @brief 
     * 
     * @param args 
     */
    void _resident(PostureManagerArguments *args);

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    int _mode = RANDOM_MOVE; /*!<  */

    bool _mode_flag_arm_track    = false; /*!<  */
    bool _mode_flag_command_move = false; /*!<  */

    float _speed_gain = 2.5; /*!<  */

    float _target_yaw      = 0; /*!<  */
    float _target_pitch    = 0; /*!<  */
    float _target_roll     = 0; /*!<  */
    float _arm_track_yaw   = 0; /*!<  */
    float _arm_track_pitch = 0; /*!<  */
    float _arm_track_roll  = 0; /*!<  */

    float _stabilize_pitch = 0; /*!<  */
    float _stabilize_roll  = 0; /*!<  */

    float _send_yaw                 = 0; /*!<  */
    float _send_pitch               = 0; /*!<  */
    float _send_roll                = 0; /*!<  */
    int _random_look_yaw            = 0; /*!<  */
    float _random_look_pitch        = 0; /*!<  */
    float _random_look_roll         = 0; /*!<  */
    int _random_target_look_yaw     = 0; /*!<  */
    float _random_target_look_pitch = 0; /*!<  */
    float _random_target_look_roll  = 0; /*!<  */
    int _random_start_look_yaw      = 0; /*!<  */
    float _random_start_look_pitch  = 0; /*!<  */
    float _random_start_look_roll   = 0; /*!<  */

    unsigned long _random_look_sendtime = 6000; /*!< mode time of look around */
    TimeCheck _random_look_timer;               /*!<  */

    TimeCheck _timer_get_gyro; /*!<  */
    TimeCheck _timer_reset;    /*!<  */

    TimeCheck _timer_hand_track_check;   /*!<  */
    float _sum_hand_track_check_old = 0; /*!<  */

    bool _pose_reset_flag = false; /*!<  */

    int _old_delta_pose = 0;          /*!<  */
    TimeCheck timer_check_delta_pose; /*!<  */
    unsigned long _resident_time = 0; /*!<  */

    float _old_sum_data       = 0; /*!<  */
    float _reset_flag_counter = 0; /*!<  */

    int _gyro_reset_flag     = 0;         /*!<  */
    int _gyro_reset_sequence = 0;         /*!<  */
    TimeCheck _timer_gyro_reset_sequence; /*!<  */
    TimeCheck _timer_pose_sequence;       /*!<  */
    int _pose_sequence_num = 0;           /*!<  */

#ifdef CIRO
    int _neck_ini_pos[3] = { 0, -8, -6 };  /*!< yaw prL prR */
    int _neck_offset[3]  = { 0, -30, -8 }; /*!< yaw pitch roll */

#endif

#ifdef CIYA
    int _neck_ini_pos[3] = { -10, -8, -8 }; /*!< yaw prL prR */
    int _neck_offset[3]  = { 0, -30, 0 };   /*!< yaw pitch roll */

#endif

private:
    PostureManagerArguments *_args; /*!< PostureManagerArguments address */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _SEMI_CIRCLE_DEGREE    = 180; /*!<  */
    const int _QUARTER_CIRCLE_DEGREE = 90;  /*!<  */

    const float _HAND_SCALING_FACTOR = 1000.0; /*!<  */

    const float _DEG_TO_RAD_FACTOR = PI / 180.0; /*!<  */
    const float _RAD_TO_DEG_FACTOR = 180.0 / PI; /*!<  */

    const int _TARGET_LOOK_YAW_RANDOM_RANGE_MIN   = -20; /*!<  */
    const int _TARGET_LOOK_YAW_RANDOM_RANGE_MAX   = 20;  /*!<  */
    const int _TARGET_LOOK_PITCH_RANDOM_RANGE_MIN = -8;  /*!<  */
    const int _TARGET_LOOK_PITCH_RANDOM_RANGE_MAX = 8;   /*!<  */
    const int _TARGET_LOOK_ROLL_RANDOM_RANGE_MIN  = -2;  /*!<  */
    const int _TARGET_LOOK_ROLL_RANDOM_RANGE_MAX  = 2;   /*!<  */

    const int _LOOK_SEND_TIME_RANDOM_RANGE_MIN = 3000; /*!<  */
    const int _LOOK_SEND_TIME_RANDOM_RANGE_MAX = 5000; /*!<  */

    const int _RANDOM_LOOK_YAW_MIN   = -30; /*!<  */
    const int _RANDOM_LOOK_YAW_MAX   = 30;  /*!<  */
    const int _RANDOM_LOOK_PITCH_MIN = -20; /*!<  */
    const int _RANDOM_LOOK_PITCH_MAX = 20;  /*!<  */
    const int _RANDOM_LOOK_ROLL_MIN  = -10; /*!<  */
    const int _RANDOM_LOOK_ROLL_MAX  = 10;  /*!<  */

    const float _SEC_TO_MICRO_SEC_FACTOR = 1000000.0; /*!<  */

    const int _NECK_YAW_RESET_LIMIT      = 45; /*!<  */
    const int _NECK_DELTA_POSE_LOW_LIMIT = 10; /*!<  */

    const float _N_SCALE = 100.0; /*!<  */

    const int _ROLL_PITCH_LIMIT = 50; /*!<  */

    const float _UNAZUKU_TARGET_PITCH_FACTOR_1 = -40.0;
    const float _UNAZUKU_TARGET_PITCH_FACTOR_2 = 2.0;
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
