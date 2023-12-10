/**
 * @file driver_stepper_motor.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Operate stepper motor
 * @version 0.1
 * @date 2023-02-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MOBILITY_UNIT_DRIVER_STEPPER_MOTOR_HPP
#define MOBILITY_UNIT_DRIVER_STEPPER_MOTOR_HPP

#include "common/types/pid_struct.hpp"

namespace mobility_unit
{
namespace driver
{

/**
 * @brief Operate stepper motor
 *
 */
class DriverStepperMotor {
public:
    DriverStepperMotor(int pin_left,
                       int pin_right,
                       int pin_sensor,
                       int pin_servo_plus,
                       int pin_servo_dir,
                       int pin_servo_enable,
                       int pin_servo_error,
                       int pwm_limit,
                       int position_max,
                       const int position_limit[2],
                       char fw_rw_sensor,
                       char fw_rw_motor);

public:
    void setup();
    void resident(bool leg_enable);
    void set_PID(float proportional, float integral, float differential);

private:
    void _PID(float _target, float _now, float __p, float __i, float __d);
    void _PID_drive(bool leg_enable);
    void _driver_check();

    int _get_pos();
    float _get_rps();

    void _send_tone(int speed, bool leg_enable);

private:
#if 0
    void _set_target(int target);
#endif

public:
    int leg_target_speed = 0;     /*!<  */
    bool smart_move_flag = false; /*!<  */
    //////////////////////////////////////////////////

    float set_speed;        /*!<  */
    int target_pos;         /*!<  */
    int now_pos;            /*!<  */
    int sensor_data;        /*!<  */
    int now_percentage;     /*!<  */
    float output_freq;      /*!<  */
    int motor_power  = 0;   /*!<  */
    float target_rps = 0.0; /*!<  */
    //////////////////////////////////////////////////

#if 0
    int SPD_limit = 255; /*!< [通信]A側モータの制限速度 */
    int zero_point;      /*!<  */
#endif
#if 1
    int _old_target_pos; /*!< TODO : いる？ */
#endif
private:
    int _sensor_pin;   /*!<  */
    int _motor_pin[2]; /*!< [R_pwm/L_pwm] */
    int _limit_pos[2]; /*!<  */

    PIDstruct *_pid_k  = new PIDstruct(0.7, 0.4f, 0.01f); /*!< TODO : *0.3;とは？ */
    PIDstruct *_pid_kv = new PIDstruct(-170, -250, 0);    /*!< TODO : *0.3;とは？ */
    PIDstruct *_pid    = new PIDstruct(0, 0, 0);

    bool _output_flag       = false; /*!<  */
    int _error_flag         = 0;     /*!<  */
    bool _driver_reset_flag = false; /*!<  */

    float _speed_rate = 1.0; /*!<  */
    float _old_pos    = 0;   /*!<  */

    char _sensor_fw_rw; /*!<  */
    char _motor_fw_rw;  /*!<  */

    unsigned long _preTime; /*!<  */

    int _check_old_pos = 0; /*!<  */
    int _check_seq     = 0; /*!<  */

    float _delta_ms;          /*!<  */
    float _limit_pwm  = 2000; /*!<  */
    int _position_max = 190;  /*!<  */
    float _i_pwm      = 0;    /*!<  */

    int _now_speed = 0; /*!<  */
    float _now_rps;     /*!<  */
#if 0

    float _move_average_delta(int new_data);

    // TODO MoveAverageに同じ関数がある
    unsigned long _dt_old_time;
    int _data_size = 15;
    int _data_array[20];
    int _dt_array[20];
    float _average_delta;
#endif
//////////////////////////////////////////////////
#if 0
    float _now_deg;
    float _accel_rate = 0.0;

    float _duty = 0;

    unsigned long _older_time;
#endif

private:
    int8_t _pin_servo_plus   = __INT8_MAX__;
    int8_t _pin_servo_dir    = __INT8_MAX__;
    int8_t _pin_servo_enable = __INT8_MAX__;
    int8_t _pin_servo_error  = __INT8_MAX__;
};

} // namespace driver
} // namespace mobility_unit

#endif
