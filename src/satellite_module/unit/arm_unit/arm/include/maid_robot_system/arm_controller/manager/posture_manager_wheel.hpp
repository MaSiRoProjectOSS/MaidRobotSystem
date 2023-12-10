/**
 * @file posture_manager_wheel.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Menage posture of wheel.
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_POSTURE_MANAGER_WHEEL_HPP
#define ARM_CONTROLLER_POSTURE_MANAGER_WHEEL_HPP

#include "maid_robot_system/common/interface/control_if.hpp"
#include "posture_manager_arguments.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Menage posture of wheel.
 *
 */
class PostureManager_Wheel : public ControlIF {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    PostureManager_Wheel();

    /*********************************************************
     * Destructor
     *********************************************************/
    ~PostureManager_Wheel();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief stop moving
     *
     * @param args : PostureManagerArguments object address
     */
    void move_stop(PostureManagerArguments *args);

    /**
     * @brief Set target_data to follow target
     *
     * @param args : PostureManagerArguments object address
     * @param r :
     * @param sita :
     * @param z :
     */
    void set_follow_target(PostureManagerArguments *args, float r, float sita, float z);

    /**
     * @brief handshake and follow
     *
     * @param args : PostureManagerArguments object address
     */
    void handshake_follow(PostureManagerArguments *args);

    /**
     * @brief Set target theta
     *
     * @param target_theta
     */
    void set_target_theta(float target_theta);

    /**
     * @brief Get target theta
     *
     * @return target theta
     */
    float get_target_theta();

    /**
     * @brief Get the target r
     *
     * @return target r
     */
    int get_target_r();

    /**
     * @brief Get the target s
     *
     * @return target s
     */
    int get_target_s();

    /**
     * @brief Set the flag of yaw control
     *
     * @param flag_yaw_control : flag of yaw control
     */
    void set_flag_yaw_control(bool flag_yaw_control);

    /**
     * @brief Set the posture address
     *
     * @param PostureManagerArguments address
     */
    void set_posture_address(PostureManagerArguments *args);

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
     * @brief control yaw angle
     *
     * @param args : PostureManagerArguments address
     */
    void _yaw_control(PostureManagerArguments *args);

    /**
     * @brief handshake and follow
     *
     * @param args : PostureManagerArguments address
     * @param r :
     * @param sita :
     * @param z :
     */
    void _handshake_follow(PostureManagerArguments *args, float r, float sita, float z);

private:
    struct _AXIS {
        float target = 0;
        float now    = 0;
    };
    _AXIS _v; /*!<  */
    _AXIS _w; /*!<  */

    int _z_count = 0; /*!<  */

    const float _wheel_diameter = 200.0; /*!< wheel diameter [mm] */
    const float _wheel_tread    = 250.0; /*!< wheel tread [mm] */

    long _yaw_rotation = 0; /*!<  */

    float _target_theta = 0; /*!< target theta */

    int _mode     = STOP; /*!< mode */
    int _target_r = 150;  /*!< target r */
    int _target_s = 15;   /*!< target s */
    float _K_v    = -1.8; /*!< gain of v */
    float _K_w    = 0.03; /*!< gain of w */

    float _R_target_pwm = 0; /*!< R target PWM */
    float _L_target_pwm = 0; /*!< L target PWM */

    int _max_speed        = 20000; /*!< max speed */
    float _max_turn_speed = 1.2;   /*!< max turn speed */

    TimeCheck _timer_check_inertia; /*!< TimeCheck */

    float _follow_target_data[3] = { 0, 0, 0 }; /*!< target data */

    float _speed_inertia   = 0.1; /*!< speed inertia  */
    bool _flag_yaw_control = 0;   /*!< yaw control flag */
    float _old_gy_yaw      = 0;   /*!< previous value of gy_yaw */

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    PostureManagerArguments *_args; /*!< PostureManagerArguments address */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _MAX_S_ERROR_TO_V = 90; /*!<  */

    const float _YAW_CHANGED_LIMIT_MAX = 300.0;  /*!<  */
    const float _YAW_CHANGED_LIMIT_MIN = -300.0; /*!<  */

    const int _R_V_FACTOR = 10;    /*!<  */
    const int _S_W_FACTOR = 10000; /*!<  */

    const int _V_MIN = 0;   /*!<  */
    const int _V_MAX = 100; /*!<  */

    const int _Z_COUNT_DECREASE_RATE = 3;   /*!<  */
    const int _Z_COUNT_MIN           = 0;   /*!<  */
    const int _Z_COUNT_MAX           = 100; /*!<  */

    const float _SPEED_INERTIA_INCREASE_RATE = 50.0 / 1500.0; /*!<  */
    const float _SPEED_INERTIA_DECREASE_RATE = 50.0 / 2500.0; /*!<  */

    const float _V_DECREASE_FACTOR = 1.5;

    const float _R_MOVING_LIMIT = 100.0; /*!<  */
    const float _S_MOVING_LIMIT = 0.1;   /*!<  */

    const float _SPEED_INERTIA_CONSTRAIN_MIN = 10.0;  /*!<  */
    const float _SPEED_INERTIA_CONSTRAIN_MAX = 100.0; /*!<  */

    const float _PI = 3.14; /*!<  */

    const float _WHEEL_TO_PWM_FACTOR = 200.0; /*!<  */
    const float _ONE_REVOLUTION_DEG  = 360.0; /*!<  */

    const float _YAW_CONTROL_MAX_TURN_SPEED = 10.0; /*!<  */

    const float _W_CONSTRAIN_FACTOR = 10000.0; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
