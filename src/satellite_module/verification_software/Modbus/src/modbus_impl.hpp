/**
 * @file modbus_impl.hpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef MODBUS_IMPL_HPP
#define MODBUS_IMPL_HPP

#include "modbus_lib_arduino.hpp"

class ModbusImpl : public ModbusLibArduino {
public:
    ModbusImpl(HardwareSerial *serial);
    ~ModbusImpl(void);

protected:
    MessageFrame _reception(MessageFrame frame) override;
};

#endif
