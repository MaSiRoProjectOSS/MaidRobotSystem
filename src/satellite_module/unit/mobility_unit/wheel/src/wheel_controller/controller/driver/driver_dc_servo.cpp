/**
 * @file driver_dc_servo.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Operate DC servo
 * @version 0.1
 * @date 2023-02-17
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "wheel_controller/controller/driver/driver_dc_servo.hpp"

namespace mobility_unit
{
namespace driver
{

DriverDCServo::DriverDCServo()
{
#if 0
 // TODO 未使用
    this->pid_k.set(1.2, 4.5, 0.002);
    this->pid_kv.set(-100, -50, 0);
#endif
}

void DriverDCServo::setup_pid(char m_fw_rw)
{
    this->motor_fw_rw = m_fw_rw;

#if 0
    // todo :未使用
    this->average_now_motor_rpm.reset();
#endif

    this->pid_velo->SetMode(AUTOMATIC);
    this->pid_velo->SetOutputLimits(-16384, 16384);
    this->pid_velo->SetSampleTime(10);
}

float DriverDCServo::set_target_motor_rps(float rps)
{
    this->target_rps       = rps;
    this->target_motor_rps = this->motor_fw_rw * this->target_rps * this->gear_ratio;

    return this->target_motor_rps;
}

int DriverDCServo::set_motor_info(int16_t velo, int16_t position, int16_t temp, int16_t trq_current)
{
#if 0
    // todo :未使用
    static MoveAverage<5> average_now_motor_rpm;
    // now_motor_rps = average_now_motor_rpm->move_average(raw_rps);
#endif
    this->raw_rps       = this->motor_fw_rw * velo;
    this->now_motor_rps = this->raw_rps;
    this->now_rps       = this->now_motor_rps / this->gear_ratio;

    this->now_pos       = this->motor_fw_rw * position / this->gear_ratio;
    this->now_temp      = temp;
    this->now_current   = trq_current;
    this->state         = STATE_OK;
    this->data_get_flag = 1;
    return this->state;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

#if 0
#if 0
 // TODO 未使用
void DriverDCServo::set_PID(float _kp, float _ki, float _kd)
{
    this->pid_k.set(_kp, _ki, _kd);
}


void DriverDCServo::set_vPID(float _kp, float _ki, float _kd)
{
    this->pid_kv.set(_kp, _ki, _kd);
}
#endif

void DriverDCServo::set_target(int target)
{
    old_target_pos = target_pos;
    target_pos     = target;
    if (old_target_pos != target_pos) {
        i_pwm   = 0;
        preTime = micros();
    }
}

void DriverDCServo::PID_rpm_drive()
{
}

int DriverDCServo::get_pos()
{
    return 0;
}

void DriverDCServo::MOT_drive(int A_pwm)
{
}

void DriverDCServo::_setup(int m_pin_R, int m_pin_L, int s_pin, int _limit[2], char S_fw_rw, char M_fw_rw)
{
    sensor_pin   = s_pin;
    motor_pin[0] = m_pin_R;
    motor_pin[1] = m_pin_L;
    limit_pos[0] = _limit[0];
    limit_pos[1] = _limit[1];
    sensor_fw_rw = S_fw_rw;
    motor_fw_rw  = M_fw_rw;

    for (int a = 0; a < 20; a++) {
        data_array[a] = 0;
        dt_array[a]   = 0;
    }

    rotation_count = 0;
    preTime        = micros();
    get_pos();
    target_motor_rpm = total_pos;
    pid.I            = 0;
}

int DriverDCServo::pid_control()
{
    //  if(-cut_off_rpm> target_motor_rpm || target_motor_rpm > cut_off_rpm)

    target_motor_rpm = target_rps * 60.0 * gear_ratio;
    if (data_get_flag == 1) {
        data_get_flag = 0;
        pid_velo->Compute();
    }
    final_output = (float)motor_fw_rw * pid_output;

    return final_output;
}

void DriverDCServo::pwm_setup(int _pin)
{
    pwm_motor.attach(_pin);
    pwm_motor.writeMicroseconds(1500);
}

int DriverDCServo::pwm_output()
{
    target_motor_rpm = target_rps * 60.0 * gear_ratio;

    if (target_motor_rpm > 0) {
        final_output = map(target_motor_rpm, 0, 8000, 1520, 1920);
    } else if (target_motor_rpm < 0) {
        final_output = map(target_motor_rpm, -8000, 0, 1080, 1480);
    } else {
        final_output = 1500;
    }

    //  final_output =  map(analogRead(A7),0,1023,1000,2000);
    //  LOG_DEBUGln(final_output);
    final_output = constrain(final_output, 1000, 2000);
    pwm_motor.writeMicroseconds(final_output);
    return final_output;
}

#endif

} // namespace driver
} // namespace mobility_unit
