/**
 * @file driver_potentiometer.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Handle potentiometer
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_DRIVER_POTENTIOMETER_HPP
#define ARM_CONTROLLER_DRIVER_POTENTIOMETER_HPP

#include "maid_robot_system/common/move_average.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Potentiometer
 * Target board : AS5600
 */
class DriverPotentiometer {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    /**
     * @brief Construct a new Driver Potentiometer object
     *
     * @param center : initial center position.
     * @param direction : initial direction.
     * @param pin : analog pin number
     */
    DriverPotentiometer(int center = 0, int direction = 0, int pin = __INT_MAX__);

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief set up the potentiometer.
     *
     * @param center : center position.
     * @param direction : potentiometer direction.
     * @param pin : analog pin number.
     */
    void setup(int center, int direction, int pin = __INT_MAX__);

    /**
     * @brief Get potentiometer angle.
     *
     * @return angle
     */
    float get_degree();

    /**
     * @brief Get potentiometer sensor data
     *
     * @return sensor data
     */
    float get_sensor();

    /**
     * @brief read analog pin and update the angle value.
     *
     */
    void read();

private:
    /*********************************************************
     * Private function
     *********************************************************/
    /**
     * @brief set up the potentiometer.
     *
     */
    void _setup();

    /**
     * @brief calibrate potentiometer.
     *
     * @param center : center position.
     * @param direction : potentiometer direction.
     */
    void _calibration(int center, int direction);

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    bool _initialized = false; /*!<  */
    int _analog_pin;           /*!<  */
    int _center;               /*!<  */
    int _direction;            /*!<  */

    float _degree = 0;          /*!<  */
    int _sensor_data;           /*!<  */
    int _counter = __INT_MAX__; /*!<  */

    MoveAverage<3> _ave_data; /*!<  */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _COUNTER_MAX    = 100; /*!<  */
    const int _OUTLIER_SENSOR = 142; /*!< this value is calculated from "50 * (1024 / 360 )" */

    const int _ANALOG_READ_MIN          = 0;    /*!<  */
    const int _ANALOG_READ_MAX          = 1023; /*!<  */
    const int _ANALOG_READ_MAX_PLUS_ONE = 1024; /*!<  */

    const int _ANALOG_ANGLE_MIN = -179; /*!<  */
    const int _ANALOG_ANGLE_MAX = 180;  /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
