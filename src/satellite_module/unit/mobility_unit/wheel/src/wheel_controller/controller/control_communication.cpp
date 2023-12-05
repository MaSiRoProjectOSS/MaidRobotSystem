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
#include "wheel_controller/controller/control_communication.hpp"

namespace mobility_unit
{
namespace controller
{

ControlCommunication::ControlCommunication(uint8_t can_cs, int32_t sensorID, uint8_t address)
{
    this->_can_cs = can_cs;
    if (NUM_DIGITAL_PINS > this->_can_cs) {
        this->_can = new ControllerCAN(can_cs);
    }
    this->_gyro = new driver::DriverGyro(sensorID, address);
}

bool ControlCommunication::_begin()
{
    if (false == this->_flag_initialized_can) {
        if (NULL != this->_can) {
            this->_flag_initialized_can = this->_can->setup();
        }
    }

    if (false == this->_flag_initialized_gyro) {
        if (NULL != this->_gyro) {
            this->_flag_initialized_gyro = this->_gyro->setup();
        }
    }

    return this->_flag_initialized_can && this->_flag_initialized_gyro;
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
    return result || this->_flag_initialized_can || this->_flag_initialized_gyro;
}

void ControlCommunication::receive(ControlWheel *wheel)
{
    // can
    if (true == this->_flag_initialized_can) {
        this->_can->receive(wheel);
    }
    // gyro
    if (true == this->_flag_initialized_gyro) {
        this->_gyro->receive();
    }
}

bool ControlCommunication::send(ControlWheel *wheel)
{
    static CoordinateEuler null_coordinate = CoordinateEuler(0, 0, 0);

    bool result = false;
    if (true == this->_flag_initialized_can) {
        if (true == this->_flag_initialized_gyro) {
            this->_can->send(wheel, this->_gyro->get());
            result = true;
        } else {
            this->_can->send(wheel, null_coordinate);
            result = true;
        }
    }
    return result;
}

} // namespace controller
} // namespace mobility_unit
