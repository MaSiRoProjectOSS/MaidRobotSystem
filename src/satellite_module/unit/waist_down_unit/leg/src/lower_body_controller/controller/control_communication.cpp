/**
 * @file control_communication.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief control the communication
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "lower_body_controller/controller/control_communication.hpp"

namespace lower_body_unit
{
namespace controller
{

ControlCommunication::ControlCommunication(uint8_t can_cs, //
                                           int32_t sensorID,
                                           uint8_t address)
{
    if (NUM_DIGITAL_PINS > can_cs) {
        this->_can = new ControllerCAN(can_cs);
    }
}

bool ControlCommunication::_begin()
{
    if (false == this->_flag_initialized_can) {
        if (NULL != this->_can) {
            this->_flag_initialized_can = this->_can->setup();
        }
    }

    return this->_flag_initialized_can;
}

bool ControlCommunication::_end()
{
    bool result = true;
    return result;
}

bool ControlCommunication::_calculate()
{
    bool result = true;
    return result;
}

bool ControlCommunication::_is_error()
{
    bool result = false;
    if (false == this->_flag_initialized_can) {
        result = true;
    } else {
        result = !this->_can->is_running();
    }
    return result || this->_flag_initialized_can;
}

void ControlCommunication::receive(ControlWaist *waist, ControlLegMotor *leg_motor, CoordinateEuler *gyro)
{
    // can
    if (true == this->_flag_initialized_can) {
        this->_can->receive(leg_motor, waist, gyro);
    }
}

bool ControlCommunication::send(ControlWaist *waist, ControlLegMotor *leg_motor)
{
    bool result = false;
    if (true == this->_flag_initialized_can) {
        result = this->_can->send(leg_motor, waist);
    }
    return result;
}

} // namespace controller
} // namespace lower_body_unit
