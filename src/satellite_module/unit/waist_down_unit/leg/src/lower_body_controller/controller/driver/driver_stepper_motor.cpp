/**
 * @file driver_stepper_motor.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Operate stepper motor
 * @version 0.1
 * @date 2023-02-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "lower_body_controller/controller/driver/driver_stepper_motor.hpp"

#include "maid_robot_system/common/log_callback.hpp"
#include "maid_robot_system/common/move_average.hpp"
#include "maid_robot_system/common/time_check.hpp"

#include <Arduino.h>

namespace lower_body_unit
{
namespace driver
{

DriverStepperMotor::DriverStepperMotor(int pin_left,
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
                                       char fw_rw_motor)
{
    this->_old_pos      = this->_get_pos();
    this->_sensor_pin   = pin_sensor;
    this->_motor_pin[0] = pin_right;
    this->_motor_pin[1] = pin_left;
    this->_limit_pos[0] = position_limit[0];
    this->_limit_pos[1] = position_limit[1];
    this->_sensor_fw_rw = fw_rw_sensor;
    this->_motor_fw_rw  = fw_rw_motor;

    this->_pin_servo_plus   = pin_servo_plus;
    this->_pin_servo_dir    = pin_servo_dir;
    this->_pin_servo_enable = pin_servo_enable;
    this->_pin_servo_error  = pin_servo_error;
    this->_limit_pwm        = pwm_limit;
    this->_position_max     = position_max;
}
void DriverStepperMotor::setup()
{
    this->_preTime = micros();
    this->_get_pos();
    this->_get_rps();
    this->_pid->I = 0;
    pinMode(this->_pin_servo_plus, OUTPUT);
    pinMode(this->_pin_servo_dir, OUTPUT);
    pinMode(this->_pin_servo_enable, OUTPUT);
    // pinMode(this->_pin_servo_enable, INPUT);

    digitalWrite(this->_pin_servo_enable, HIGH);
    delay(2000);
    digitalWrite(this->_pin_servo_enable, LOW);
    this->_get_pos();
    target_pos = now_pos;
}

#if 0
void DriverStepperMotor::_driver_check()
{
    static TimeCheck speed_rate_timer;
    static TimeCheck check_output_timer;
    static TimeCheck driver_check_timer;

    if (this->_output_flag == true) {
        if (true == check_output_timer.check_passing(4000)) {
            if (abs(this->_check_old_pos - now_pos) < 1) {
                this->_driver_reset_flag = true;
                LOG_DEBUG("reset");
            }
            this->_check_old_pos = now_pos;
        }
    } else {
        check_output_timer.update();
    }

    if (this->_check_seq == 0) {
        if (this->_driver_reset_flag == true) {
#ifdef msg_debug
            char buffer[255];
            sprintf(buffer, "driver_reset : %d", analogRead(this->_pin_servo_error));
            LOG_DEBUG(buffer);
#endif
            this->_speed_rate = 0.0;
            driver_check_timer.update();
            this->_check_seq++;
        } else {
            this->_speed_rate = (float)constrain(speed_rate_timer.get_elapsed_time(), 0, 5000) / 5000.0;

            if (this->_speed_rate > 1.0) {
                this->_speed_rate = 1.0;
            }
        }
    }
    if (this->_check_seq == 1) {
        this->_error_flag = 1;
        if (true == driver_check_timer.check_passing(1000)) {
            LOG_DEBUG("asasa :");
            digitalWrite(this->_pin_servo_enable, HIGH);
            // digitalWrite(this->_pin_servo_dir, HIGH);
            driver_check_timer.update();
            this->_check_seq++;
        }
    }
    if (this->_check_seq == 2) {
        this->_error_flag = 0;

        if (true == driver_check_timer.check_passing(1000)) {
            LOG_DEBUG("lklk :");
            digitalWrite(this->_pin_servo_enable, LOW);

            this->_speed_rate = 0.0;
            speed_rate_timer.update();
            this->_check_seq         = 0;
            this->_driver_reset_flag = false;
            check_output_timer.update();
        }
    }
}
#endif

void DriverStepperMotor::set_PID(float proportional, float integral, float differential)
{
    this->_pid_k->set(proportional, integral, differential);
}

void DriverStepperMotor::_PID(float _target, float _now, float __p, float __i, float __d)
{
    this->_delta_ms = (micros() - this->_preTime) / 1000.0;

    this->_preTime         = micros();
    this->_pid->P          = _target - _now;
    this->_pid->D          = (this->_pid->P - this->_pid->previous_P) / this->_delta_ms;
    this->_pid->previous_P = this->_pid->P;

    this->_pid->I += this->_pid->P * this->_delta_ms;

    float send_I = constrain(this->_pid->I / 1000.0, -255, 255);
    float send_D = this->_pid->D / 1000.0;

    this->set_speed = constrain(__p * this->_pid->P + __i * send_I + __d * send_D, -this->_limit_pwm, this->_limit_pwm);
}

float DriverStepperMotor::_get_rps()
{
    static MoveAverage<15> rps_mv;
    float __now_pos = this->_get_pos();
    float delta_pos = this->_old_pos - __now_pos;

    if (delta_pos > 1000) {
        delta_pos = delta_pos - 1023;
    }
    if (delta_pos < -1000) {
        delta_pos = delta_pos + 1023;
    }

    if (abs(delta_pos) < 100) { //外れ値をなくす
        this->_old_pos = __now_pos;
        this->_now_rps = rps_mv.set_delta(delta_pos) / 1023.0;
    }
    return this->_now_rps;
}

int DriverStepperMotor::_get_pos()
{
    if (this->_sensor_fw_rw == 1) {
        this->sensor_data = analogRead(this->_sensor_pin);
    } else {
        this->sensor_data = 1023 - analogRead(this->_sensor_pin);
    }

    int get_angle = this->sensor_data - this->_limit_pos[0];
    if (get_angle < -512) {
        get_angle = get_angle + 1024;
    }
    if (get_angle > 512) {
        get_angle = -get_angle + 1024;
    }
    this->now_pos = (get_angle);
    //now_pos = get_angle ;

    this->now_percentage = map(this->now_pos, 0, this->_position_max, 0, 1000);
    return this->now_pos;
}
void DriverStepperMotor::_send_tone(int speed, bool leg_enable)
{
    static TimeCheck accel_rate_timer;
    speed = this->_motor_fw_rw * speed;
    //加減速処理
    if (true == accel_rate_timer.check_passing(1)) {
        if (this->_now_speed >= speed) {
            this->_now_speed -= 10;
        }
        if (this->_now_speed < speed) {
            this->_now_speed += 10;
        }
    }
    this->_now_speed = speed;

    if (this->_now_speed > 0) {
        digitalWrite(this->_pin_servo_dir, 1);

    } else {
        digitalWrite(this->_pin_servo_dir, 0);
    }

    this->output_freq = abs(this->_now_speed * 10.0) * this->_speed_rate;

    if ((true == leg_enable) && this->output_freq > 40 && this->_error_flag == 0) {
        tone(this->_pin_servo_plus, output_freq); //* speed_rate));
        this->_output_flag = true;
    } else {
        noTone(this->_pin_servo_plus);
        this->_output_flag = false;
    }
}
void DriverStepperMotor::_PID_drive(bool leg_enable)
{
    int set_target_pwm;
    if (this->smart_move_flag == true) {
        set_target_pwm = constrain(abs(this->leg_target_speed), 100, this->_limit_pwm);
    } else {
        set_target_pwm = this->_limit_pwm;
    }

    if (0 <= this->now_pos && this->now_pos <= 20) {
        this->_limit_pwm = map(this->now_pos, 0, 20, set_target_pwm / 10.0, set_target_pwm / 3.0);
    } else if (20 < now_pos && this->now_pos <= 80) {
        this->_limit_pwm = map(this->now_pos, 20, 80, set_target_pwm / 3.0, set_target_pwm);
    } else if (this->now_pos >= this->_limit_pwm - 10) {
        this->_limit_pwm = map(this->now_pos, this->_position_max - 10, this->_position_max, set_target_pwm, 100);
    } else {
        this->_limit_pwm = set_target_pwm;
    }

    this->_limit_pwm = constrain(abs(this->_limit_pwm), 100, set_target_pwm);

    char burrer[255];
    sprintf(burrer, "limit_pwm : %f", (double)this->_limit_pwm);
    LOG_DEBUG(burrer);

    this->_PID(this->target_pos, this->now_pos, this->_pid_k->P, this->_pid_k->I, this->_pid_k->D);

    if (this->set_speed > 0) {
        if (this->now_pos > this->_position_max) {
            this->set_speed = 0;
        }
    } else {
        if (now_pos < 0) {
            this->set_speed = 0;
        }
    }

    // set_speed = 100;
    this->_send_tone(this->set_speed, leg_enable);
}
void DriverStepperMotor::resident(bool leg_enable)
{
    this->_get_pos();
    this->_PID_drive(leg_enable);
}

} // namespace driver
} // namespace lower_body_unit
