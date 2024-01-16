/**
 * @file modbus_lib_arduino.hpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-11
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#if 1
#ifndef MODBUS_LIB_ARDUINO_HPP
#define MODBUS_LIB_ARDUINO_HPP
#include "stab_MessageFrame.hpp"

#include <Arduino.h>

class ModbusLibArduino {
public:
    ModbusLibArduino(HardwareSerial *serial)
    {
    }

public:
    MessageFrame pub_reception(MessageFrame frame)
    {
        MessageFrame reception = this->_reception(frame);
        for (int i = reception.data_length; i < 255; i++) {
            reception.data[i] = 0;
        }
        return reception;
    }

public:
    bool begin(int address, MessageFrame::MODBUS_TYPE type = MessageFrame::MODBUS_TYPE_RTU, unsigned long baud = 115200)
    {
        return true;
    }
    bool init(int address, MessageFrame::MODBUS_TYPE type = MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU_EX)
    {
        return this->_init();
    }

protected:
    virtual MessageFrame _reception(MessageFrame frame) = 0;
    virtual bool _init()
    {
        return true;
    }
};
#endif
#endif
