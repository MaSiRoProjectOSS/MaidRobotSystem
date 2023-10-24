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
#ifndef LOWER_BODY_UNIT_CONTROL_COMMUNICATION_HPP
#define LOWER_BODY_UNIT_CONTROL_COMMUNICATION_HPP

#include "lower_body_controller/controller/sub_controller/controller_can.hpp"

namespace lower_body_unit
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
    void receive(ControlWaist *waist, ControlLegMotor *leg_motor, CoordinateEuler *gyro);
    bool send(ControlWaist *waist, ControlLegMotor *leg_motor);

private:
    ControllerCAN *_can; /*!<  */

    bool _flag_initialized_can = false; /*!<  */
};

} // namespace controller
} // namespace lower_body_unit

#endif
