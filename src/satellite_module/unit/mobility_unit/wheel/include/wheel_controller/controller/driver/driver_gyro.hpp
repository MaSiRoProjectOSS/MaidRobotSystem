/**
 * @file driver_gyro.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Operate Gyro device
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MOBILITY_UNIT_DRIVER_GYRO_HPP
#define MOBILITY_UNIT_DRIVER_GYRO_HPP

#include "maid_robot_system/common/time_check.hpp"
#include "maid_robot_system/common/types/coordinate_euler.hpp"

#include <Adafruit_BNO055.h>

namespace mobility_unit
{
namespace driver
{

/**
 * @brief Operate Gyro device
 *
 */
class DriverGyro {
public:
    DriverGyro(int32_t sensorID, uint8_t address);

public:
    bool setup();
    bool restart();
    void receive();
    CoordinateEuler get();

private:
    Adafruit_BNO055 *_bno; /*!<  */

    CoordinateEuler _current; /*!<  */
    int8_t _temperature = 0;  /*!<  */

private:
    bool _flag_initialized = false; /*!<  */

    TimeCheck _intermittent_short;                /*!<  */
    const int INTERMITTENT_SHORT_MS = 50;         /*!<  */
    TimeCheck _intermittent_long;                 /*!<  */
    const int INTERMITTENT_LONG_MS = 15 * (1000); /*!<  */
};

} // namespace driver
} // namespace mobility_unit

#endif
