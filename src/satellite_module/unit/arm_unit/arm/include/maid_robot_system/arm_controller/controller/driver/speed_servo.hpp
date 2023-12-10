/**
 * @file speed_servo.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Control Servo motor to control speed and position along with specified trajectory.
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_SPEED_SERVO_HPP
#define ARM_CONTROLLER_SPEED_SERVO_HPP

#include <Arduino.h>
#include <Servo.h>

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Control Servo motor to control speed and position along with specified trajectory.
 * 
 */
class SpeedServo {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    SpeedServo();

public:
    /*********************************************************
     * public function
     *********************************************************/
    /**
     * @brief initialize Servo.
     *
     * @param servo_port : servo port number.
     * @param ini_pos : initial position.
     * @param direction : direction.
     */
    void setup(int servo_port, int ini_pos, int direction);

    /**
     * @brief Run servo.detach()
     *
     */
    void detach();

    /**
     * @brief Get current position
     *
     * @return current position
     */
    float get_pos();

    /**
     * @brief move along trajectory
     *
     * @param tar_deg : target angle[deg]
     * @return current position
     */
    int move_along_trajectory(int tar_deg);

    /**
     * @brief move along trajectory
     *
     * @param tar_deg : target angle[deg]
     * @param tar_speed : target speed
     * @return current position
     */
    int move_along_trajectory(float tar_deg, int tar_speed);

    /**
     * @brief rotate the servo motor to the reference angle.
     *
     * @param deg : reference angle[deg]
     * @return reference angle[deg]
     */
    float servo_write_deg(float deg);

    /**
     * @brief Run move_along_trajectory()
     *
     * @param tar_pos : target position
     * @return current position
     */
    int move_p(int tar_pos);

    /**
     * @brief Run move_along_trajectory()
     *
     * @return current position
     */
    int move_p();

private:
    Servo _servo;
    int _center_pos = 90;  /*!< center position */
    int _speed      = 100; /*!< speed */

    float _now_pos      = 0; /*!< current position */
    float _true_tar_pos = 0; /*!< true target position */

    int _target_pos = 0; /*!< target position */

    float _p_gain     = 0.05; /*!< P gain for controlling servo */
    float _deg_to_pwm = 1.0;  /*!< degree to PWM */

    int _direction = 1; /*!< direction */

    unsigned long _servo_send_time = 0; /*!< interval time of sending servo signal */

private:
    const int _MAX_DELTA_TIME     = 100000; /*!<  */
    const int _SUB_MAX_DELTA_TIME = 10000;  /*!<  */

    const int _SPEED_MIN    = 0;    /*!<  */
    const int _SPEED_MAX    = 255;  /*!<  */
    const int _POSITION_MIN = 0;    /*!<  */
    const int _POSITION_MAX = 2000; /*!<  */

    float _PERCENT_TO_RATE = 100.0; /*!<  */

    const int _DIRECTION_SCALE = 100; /*!<  */

    const int _SERVO_SCALE_FACTOR = 90;   /*!<  */
    const int _SERVO_SEND_MIN     = 800;  /*!<  */
    const int _SERVO_SEND_MAX     = 2200; /*!<  */

    const int _MOVE_P_SPEED_MIN = 1;   /*!<  */
    const int _MOVE_P_SPEED_MAX = 255; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
