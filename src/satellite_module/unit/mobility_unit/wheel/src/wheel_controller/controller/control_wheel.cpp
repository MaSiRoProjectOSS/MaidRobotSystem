/**
 * @file control_wheel.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief control the wheel
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "wheel_controller/controller/control_wheel.hpp"

#include "maid_robot_system/common/move_average.hpp"

namespace mobility_unit
{
namespace controller
{

///////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////
ControlWheel::ControlWheel(HardwareSerial *h_serial, uint8_t pin_enable, uint8_t id, int revolution_left, int revolution_right)
{
    this->_revolution_left  = revolution_left;
    this->_revolution_right = revolution_right;

    this->_target_v = 0;

    this->_pin_enable = pin_enable;
    this->step_speed  = abs(this->_now_v);

    this->_roboclaw = new driver::DriverRoboclaw(h_serial, this->_pin_enable, id);
}

#ifdef __AVR__
ControlWheel::ControlWheel(SoftwareSerial *s_serial, uint8_t pin_enable, uint8_t id, int revolution_left, int revolution_right)
{
    this->_revolution_left  = revolution_left;
    this->_revolution_right = revolution_right;

    this->_target_v = 0;

    this->_pin_enable = pin_enable;
    this->step_speed  = abs(this->_now_v);

    this->_roboclaw = new driver::DriverRoboclaw(s_serial, this->_pin_enable, id);
}
#endif

///////////////////////////////////////////////////////////
// Inherited function
///////////////////////////////////////////////////////////
bool ControlWheel::_begin()
{
    bool result = false;
    if (NULL != this->_roboclaw) {
        result = this->_roboclaw->setup();
        if (true == result) {
            delay(200);
            delay(200);
            this->setup_pid(this->_revolution_left);
            delay(200);
            this->setup_pid(this->_revolution_right);
            delay(200);

            this->target_rps = 0;
            this->target_rps = 0;

            this->_roboclaw->enable(false);
            delay(1000);
        }
    }
    return result;
}

bool ControlWheel::_end()
{
    bool result = true;
    return result;
}

bool ControlWheel::_calculate()
{
    static TimeCheck step_period_timer;
    static TimeCheck arm_step_period_timer;
    bool result = false;

    if (abs(this->_now_v) > 0) {
        result = true;
    }

    if (true == result) {
        if (true == step_period_timer.check_passing(this->step_period)) {
            float calc_step_v = (abs(this->now_rps) + abs(this->now_rps)) * this->WHEEL_DIAMETER * 3.14 / 2.0;
            this->step_period = constrain(step_stride / abs(calc_step_v) * 1000.0, 200, 1300);

            step_stride       = constrain(map(abs(calc_step_v), 0, 800, 1250 * 0.1, 1250 * 0.25), 1250 * 0.05, 1250 * 0.55);
            this->step_height = constrain(map(abs(calc_step_v), 0, 800, 60, 150), this->MIN_STEP_HEIGHT, this->STEP_HEIGHT_MAX);
        }

        this->step_height = this->step_height * step_period_timer.get_sin_cycle(this->step_period);
    } else {
        step_period_timer.update();
        arm_step_period_timer.update();
        this->step_period = 1000;
    }

    //////////////////
    this->_resident();
    //////////////////
    this->_step_percentage = (int)(100.0 * this->step_height / this->STEP_HEIGHT_MAX);
    //////////////////
    return result;
}

bool ControlWheel::_is_error()
{
    return false;
}

///////////////////////////////////////////////////////////
// Setter / Getter
///////////////////////////////////////////////////////////
void ControlWheel::set_data(unsigned char rec_data[8])
{
    this->_motor_mode = rec_data[0];
    int16_t get_1     = ((rec_data[1] << 8) & 0xff00) | (rec_data[2] & 0x00ff);
    int16_t get_2     = ((rec_data[3] << 8) & 0xff00) | (rec_data[4] & 0x00ff);

    if (this->_motor_mode == this->CONTROL_WHEEL_MODE_ROBOCLAW) {
        this->_target_v = get_1;
        this->_target_w = -get_2 / 1000.0;
    }
}
int ControlWheel::get_step_percentage()
{
    return this->_step_percentage;
}

///////////////////////////////////////////////////////////
// private function
///////////////////////////////////////////////////////////
void ControlWheel::_resident()
{
    static MoveAverage<5> average_v;
    static MoveAverage<5> average_w;
    static TimeCheck control_rate_timer;

    if (true == control_rate_timer.check_passing(15)) {
        this->_get_motor_status();

        if ((true == this->is_enable()) && (this->_target_v != 0 || this->_target_w != 0)) {
            this->set_target_motor_rps((this->_target_v + this->_target_w * this->WHEEL_TREAD) / (this->WHEEL_DIAMETER * 3.14));
            this->set_target_motor_rps((this->_target_v - this->_target_w * this->WHEEL_TREAD) / (this->WHEEL_DIAMETER * 3.14));

            this->_roboclaw->enable(true);
        } else {
            this->set_target_motor_rps(0);
            this->set_target_motor_rps(0);
            this->_roboclaw->enable(false);
        }

        this->_roboclaw->SpeedM1(this->target_motor_rps);
        this->_roboclaw->SpeedM2(this->target_motor_rps);
    }

    if (this->_motor_mode == this->CONTROL_WHEEL_MODE_ROBOCLAW) {
    } else {
        this->_target_v = 0;
        this->_target_w = 0;
    }

    this->_now_v = average_v.set((this->now_rps + this->now_rps) * this->WHEEL_DIAMETER * 3.14 / 2.0);
    this->_now_w = average_w.set((this->now_rps - this->now_rps) * this->WHEEL_DIAMETER * 3.14 / (2.0 * this->WHEEL_TREAD));
}

void ControlWheel::_get_motor_status()
{
    uint8_t status3;
    uint8_t status4;
    bool valid3;
    bool valid4;

    this->motor1_rps = this->_roboclaw->ReadSpeedM1(&status3, &valid3);
    this->motor2_rps = this->_roboclaw->ReadSpeedM2(&status4, &valid4);

    this->battery_voltage = this->_roboclaw->ReadMainBatteryVoltage();

    if ((true == valid3) && (true == valid4)) {
        this->set_motor_info(this->motor1_rps, 0, 0, 0);
        this->set_motor_info(this->motor2_rps, 0, 0, 0);
    }
}

void ControlWheel::setup_pid(char m_fw_rw)
{
    this->motor_fw_rw = m_fw_rw;
}

float ControlWheel::set_target_motor_rps(float rps)
{
    this->target_rps       = rps;
    this->target_motor_rps = this->motor_fw_rw * this->target_rps * this->gear_ratio;

    return this->target_motor_rps;
}

int ControlWheel::set_motor_info(int16_t velo, int16_t position, int16_t temp, int16_t trq_current)
{
    this->raw_rps       = this->motor_fw_rw * velo;
    this->now_motor_rps = this->raw_rps;
    this->now_rps       = this->now_motor_rps / this->gear_ratio;

    this->now_pos  = this->motor_fw_rw * position / this->gear_ratio;
    this->now_temp = temp;
    this->state    = STATE_OK;
    return this->state;
}

} // namespace controller
} // namespace mobility_unit
