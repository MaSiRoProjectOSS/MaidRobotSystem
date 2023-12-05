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
#include "wheel_controller/controller/driver/driver_stepper_motor.hpp"

#include "common/log_callback.hpp"
#include "common/move_average.hpp"
#include "common/time_check.hpp"

#include <Arduino.h>

namespace mobility_unit
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

#if 0
    // TODO : LED 制御はここでしない
    if (this->_check_seq != 0) {
        analogWrite(SETTING_PIN_LED, 100.0 * sin(millis() / 1000));
    }
#endif

    if (this->_check_seq == 0) {
        //     if(digitalRead(this->_pin_servo_error) == HIGH){

        //  if(analogRead(this->_pin_servo_error) <900){

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
            this->_speed_rate = constrain(speed_rate_timer.get_elapsed_time(), 0, 5000) / 5000.0;

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

void DriverStepperMotor::set_PID(float proportional, float integral, float differential)
{
    this->_pid_k->set(proportional, integral, differential);
}

#if 0
void DriverStepperMotor::set_vPID(float _kp, float _ki, float _kd)
{
    kv_p = _kp;
    kv_i = _ki;
    kv_d = _kd;
}
#endif

void DriverStepperMotor::_PID(float _target, float _now, float __p, float __i, float __d)
{
    this->_delta_ms = (micros() - this->_preTime) / 1000.0;

    this->_preTime         = micros();
    this->_pid->P          = _target - _now;
    this->_pid->D          = (this->_pid->P - this->_pid->previous_P) / this->_delta_ms;
    this->_pid->previous_P = this->_pid->P;
    /*
    float pre_I = (I + constrain(P * dt,-200 ,200))/1000.0;
    if(pre_I * kv_i >255  || pre_I * kv_i < -255){
    }else{
    }
  */
    this->_pid->I += this->_pid->P * this->_delta_ms;

    float send_I = constrain(this->_pid->I / 1000.0, -255, 255);
    float send_D = this->_pid->D / 1000.0;

    this->set_speed = constrain(__p * this->_pid->P + __i * send_I + __d * send_D, -this->_limit_pwm, this->_limit_pwm);
}

// TODO: [int dt = this->_dt_old_time - now_time;]について
/*
計測時間がマイナスに振れている。
MoveAverage関数は、now_time - _dt_old_time
意図はどっち？
*/
#if 0
float DriverStepperMotor::_move_average_delta(int new_data)
{
    long sum_data          = 0;
    long sum_time          = 0;
    unsigned long now_time = micros();
    int dt                 = this->_dt_old_time - now_time;
    this->_dt_old_time     = now_time;

    for (int i = 0; i < this->_data_size - 1; i++) {
        this->_data_array[i] = this->_data_array[i + 1];
        this->_dt_array[i]   = this->_dt_array[i + 1];
        sum_data += this->_data_array[i];
        sum_time += this->_dt_array[i];
    }
    this->_data_array[this->_data_size - 1] = new_data;
    sum_data += this->_data_array[this->_data_size - 1];

    this->_dt_array[this->_data_size - 1] = dt;
    sum_time += this->_dt_array[this->_data_size - 1];

    this->_average_delta = sum_data / (sum_time / 1000000.0);
    return this->_average_delta;
}

void DriverStepperMotor::_set_target(int target)
{
#if 0
    // TODO : ロジック確認
    // オリジナル
    this->_old_target_pos = target_pos;
    target_pos            = target;
    if (this->_old_target_pos != target_pos) {
        i_pwm   = 0;
        preTime = micros();
    }
#else
    if (this->target_pos != target) {
        this->_i_pwm   = 0;
        this->_preTime = micros();
    }
    this->target_pos = target;
#endif
}
#endif

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
        //  float dt = millis() - older_time ;
        //  older_time = millis();
        //  now_rps = -delta_pos   / (dt/1000000.0)  /1023.0;
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
    //target_pos = map(analogRead(A7), 0, 1023, 0, limit_pos[1] - this->_limit_pos[0]);

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

    // leg_motor.limit_pwm = constrain ( map(leg_motor.now_percentage,0,1000,500,3000),500,3000);

    this->_PID(this->target_pos, this->now_pos, this->_pid_k->P, this->_pid_k->I, this->_pid_k->D);
    /////TEST OUTPUT PWM
    /*
       set_speed = map (analogRead(A7) ,0,1023, -1000 ,1000);
       leg->enable();
       error_flag =0;
       LOG_DEBUG("\t set_speed ");
       LOG_DEBUG(set_speed);
       LOG_DEBUG("\t this->_sensor_pin ");
      LOG_DEBUG(analogRead(SETTING_PIN_LEG_POSITION));
       LOG_DEBUG("\t target_pos ");
      LOG_DEBUG(target_pos);

       LOG_DEBUG("\t now_pos ");
      LOG_DEBUG(now_pos);


      */

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
#if 0
    this->_driver_check();
#endif
}

} // namespace driver
} // namespace mobility_unit
