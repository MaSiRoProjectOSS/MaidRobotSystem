/**
 * @file control_communication.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief control the communication
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MOBILITY_UNIT_CONTROL_COMMUNICATION_HPP
#define MOBILITY_UNIT_CONTROL_COMMUNICATION_HPP

#include "wheel_controller/controller/driver/driver_gyro.hpp"
#include "wheel_controller/controller/sub_controller/controller_can.hpp"

namespace mobility_unit
{
namespace controller
{

/**
 * @brief control the communication
 *
 */
class ControlCommunication : public ControlIF {
public:
    ControlCommunication(uint8_t can_cs, int32_t sensorID, uint8_t address);

protected:
    bool _begin() override;
    bool _end() override;
    bool _calculate() override;

    bool _is_error() override;

public:
    void receive(ControlWheel *wheel);
    bool send(ControlWheel *wheel);

private:
    ControllerCAN *_can;       /*!<  */
    driver::DriverGyro *_gyro; /*!<  */

    uint8_t _can_cs             = __UINT8_MAX__; /*!<  */
    bool _flag_initialized_can  = false;         /*!<  */
    bool _flag_initialized_gyro = false;         /*!<  */
};

} // namespace controller
} // namespace mobility_unit

#endif
