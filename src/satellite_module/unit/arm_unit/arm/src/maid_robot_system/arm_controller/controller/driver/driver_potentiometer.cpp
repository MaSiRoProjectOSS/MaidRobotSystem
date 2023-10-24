/**
 * @file driver_potentiometer.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Handle potentiometer
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/controller/driver/driver_potentiometer.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

DriverPotentiometer::DriverPotentiometer(int center, int direction, int pin)
{
    if (__INT_MAX__ != pin) {
        this->_analog_pin  = pin;
        this->_initialized = true;
    }
    this->_calibration(center, direction);
}

void DriverPotentiometer::setup(int center, int direction, int pin)
{
    if (__INT_MAX__ != pin) {
        this->_analog_pin  = pin;
        this->_initialized = true;
    }
    this->_calibration(center, direction);
    this->_setup();
}

float DriverPotentiometer::get_degree()
{
    return this->_degree;
}

float DriverPotentiometer::get_sensor()
{
    return this->_sensor_data;
}

void DriverPotentiometer::read()
{
    static int previous = 0;
    bool flag_outrange  = true;

    if (true == this->_initialized) {
        int value = analogRead(this->_analog_pin); /* from 0 to 1023 */
        /* n * (1024 / 360 ) */

        if (0 > ((previous - this->_OUTLIER_SENSOR))) {
            /* If the minimum value of the range is negative */
            if (value > (this->_ANALOG_READ_MAX_PLUS_ONE + previous - this->_OUTLIER_SENSOR)) {
                /* If sensor value is greater than "1024 - (minus value)", approved. */
                flag_outrange = false;
            }
        } else if (this->_ANALOG_READ_MAX < ((previous + this->_OUTLIER_SENSOR))) {
            /* If the maximum value of the range is greater than 1023 */
            if (value < (previous + this->_OUTLIER_SENSOR - 1024)) {
                /* If sensor value is smaller than "(Added value) - 1024", approved. */
                flag_outrange = false;
            }
        } else {
            /* If the value is in the range */
            if ((abs(value - previous)) < this->_OUTLIER_SENSOR) {
                /* If the absolute difference is smaller than the limit, approved. */
                flag_outrange = false;
            }
        }

        if (true == flag_outrange) {
            /* Assume it is normal though default number of outliers detected. */
            if (this->_counter < this->_COUNTER_MAX) {
                flag_outrange = false;
                this->_ave_data.reset();
            }
        }

        if (false == flag_outrange) {
            int angle_sensor = (value - this->_center);
            if (angle_sensor < 0) {
                angle_sensor = angle_sensor + this->_ANALOG_READ_MAX_PLUS_ONE;
            }
            if (this->_ANALOG_READ_MAX < angle_sensor) {
                angle_sensor = angle_sensor - this->_ANALOG_READ_MAX_PLUS_ONE;
            }

            int angle = map(angle_sensor, this->_ANALOG_READ_MIN, this->_ANALOG_READ_MAX, this->_ANALOG_ANGLE_MIN, this->_ANALOG_ANGLE_MAX) * this->_direction;

            /* Update values. */
            this->_counter     = 0;
            this->_sensor_data = value;
            previous           = this->_sensor_data;
            this->_degree      = this->_ave_data.set(angle);
        }
    }
}

void DriverPotentiometer::_calibration(int center, int direction)
{
    this->_center    = center;
    this->_direction = direction;
}

void DriverPotentiometer::_setup()
{
    this->_ave_data.reset(0);
    this->_counter = __INT_MAX__;
}

} // namespace arm_unit
} // namespace maid_robot_system
