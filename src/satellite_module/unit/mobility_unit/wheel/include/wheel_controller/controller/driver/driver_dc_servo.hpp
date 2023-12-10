/**
 * @file driver_dc_servo.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Operate DC servo
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MOBILITY_UNIT_DRIVER_DC_SERVO_HPP
#define MOBILITY_UNIT_DRIVER_DC_SERVO_HPP

#include "common/types/pid_struct.hpp"

#include <PID_v1.h>
#include <Servo.h>

namespace mobility_unit
{
namespace driver
{

/**
 * @brief Operate DC servo
 *
 */
class DriverDCServo {
public:
    DriverDCServo();

public:
    void setup_pid(char m_fw_rw);
    float set_target_motor_rps(float rps);
    int set_motor_info(int16_t velo, int16_t position, int16_t temp, int16_t trq_current);

public:
    float target_rps;           /*!<  */
    int target_pwm         = 0; /*!<  */
    float target_motor_rps = 0; /*!<  */
    int motor_power        = 0; /*!<  */
    float now_rps;              /*!<  */

private:
    int data_get_flag = 0;                           /*!<  */
    float gear_ratio  = 64.0 * 70.0 * (54.0 / 25.0); /*!< POLORU */

    char motor_fw_rw;        /*!<  */
    float now_current;       /*!<  */
    double now_motor_rps;    /*!<  */
    int now_pos;             /*!<  */
    int now_temp;            /*!<  */
    PID *pid_velo;           /*!<  */
    float raw_rps;           /*!<  */
    int state = STATE_ERROR; /*!<  */

public:
    const int STATE_OK    = 10; /*!<  */
    const int STATE_ERROR = -1; /*!<  */
    const int STATE_STOP  = 0;  /*!<  */

#if 0
public:
    /////////////////////////////////////////////////////////////////////////////
public:
#if 0
 // TODO 未使用
    void set_PID(float _kp, float _ki, float _kd);
    void set_vPID(float _kp, float _ki, float _kd);
#endif

private:
#if 0
 // TODO 未使用
    PIDstruct pid_k;
   PIDstruct pid_kv;
#endif
    PIDstruct pid;

private:
    //  float gear_ratio = 19.0* (54.0/25.0);     //ROBOMAS

    double pid_output = 0, target_motor_rpm = 0;
    double final_output;

    int SPD_limit = 255; //[通信]A側モータの制限速度
    int sensor_pin;
    int motor_pin[2]; //R_pwm/L_pwm

    long rotation_count;
    long total_pos;

    int old_get_pos;
    int target_pos;
    int old_target_pos;
    int limit_pos[2];

    float now_deg;

    float sin_now_motor_rpm;

    float cut_off_rpm = 10;

    float output_pwm;
    float limit_pwm = 255;

    float i_pwm = 0;

    float duty = 0;
    float delta_t;
    unsigned long preTime;


    //正転 逆転
    char sensor_fw_rw;

    int data_size = 15;
    int data_array[20];
    int dt_array[20];
    float average_delta;

    unsigned long dt_old_time;
    int mill_dt                    = 0;
    unsigned long rpm_control_time = 0;
    Servo pwm_motor;

private:
    void set_target(int target);

    void PID_rpm_drive();

    int get_pos();

    void MOT_drive(int A_pwm);
    void _setup(int m_pin_R, int m_pin_L, int s_pin, int _limit[2], char S_fw_rw, char M_fw_rw);

    int pid_control();

    void pwm_setup(int _pin);
    int pwm_output();
#endif
};

} // namespace driver
} // namespace mobility_unit

#endif
