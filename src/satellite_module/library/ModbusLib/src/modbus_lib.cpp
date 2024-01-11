/**
 * @file modbus_lib.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-06
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "modbus_lib.hpp"

// =============================
// PUBLIC : Function
// =============================
bool ModbusLib::init(int address, MessageFrame::MODBUS_TYPE type)
{
    this->_type = type;
    if ((0 <= address) || (address <= this->SLAVE_ADDRESS_MAX)) {
        this->_address = address;
    } else {
        this->_type = MessageFrame::MODBUS_TYPE::MODBUS_TYPE_NONE;
    }

    ///////////////////////////////////////////
    // TODO : Not support TCP
    if (this->_type == MessageFrame::MODBUS_TYPE::MODBUS_TYPE_TCP) {
        this->_type = MessageFrame::MODBUS_TYPE::MODBUS_TYPE_NONE;
    }
    ///////////////////////////////////////////

    if (this->_type == MessageFrame::MODBUS_TYPE::MODBUS_TYPE_NONE) {
        this->_address = -1;
        return false;
    } else {
        return true;
    }
}

// =============================
// PUBLIC : Function : Getter/Setter
// =============================
int ModbusLib::get_address(void)
{
    return this->_address;
}
MessageFrame::MODBUS_TYPE ModbusLib::get_type(void)
{
    return this->_type;
}

// =============================
// PROTECTED : Function
// =============================
bool ModbusLib::is_range_slave_address()
{
    bool result = false;
    if (this->SLAVE_ADDRESS_MIN <= this->_address && this->_address <= this->SLAVE_ADDRESS_MAX) {
        result = true;
    }
    return result;
}

// =============================
// Constructor
// =============================
ModbusLib::ModbusLib()
{
}
ModbusLib::~ModbusLib()
{
}
