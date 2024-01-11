/**
 * @file modbus_lib_arduino.hpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-11
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#if PIO_UNIT_TESTING
#ifndef MODBUS_LIB_ARDUINO_HPP
#define MODBUS_LIB_ARDUINO_HPP
#include "stab_MessageFrame.hpp"

class HardwareSerial {
public:
    HardwareSerial()
    {
    }
};

class ModbusLibArduino {
public:
    ModbusLibArduino(HardwareSerial *serial)
    {
    }
    MessageFrame pub_reception(MessageFrame frame)
    {
        return this->reception(frame);
    }

protected:
    virtual MessageFrame reception(MessageFrame frame) = 0;
};
#endif
#endif
