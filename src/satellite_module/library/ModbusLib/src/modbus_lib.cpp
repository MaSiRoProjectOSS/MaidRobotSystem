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

/////////////////////////////////////////////////////////////////////
// ModbusLib
/////////////////////////////////////////////////////////////////////
ModbusLib::ModbusLib()
{
}
ModbusLib::~ModbusLib()
{
}
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
int ModbusLib::get_address(void)
{
    return this->_address;
}
MessageFrame::MODBUS_TYPE ModbusLib::get_type(void)
{
    return this->_type;
}
bool ModbusLib::is_range_slave_address()
{
    bool result = false;
    if (this->SLAVE_ADDRESS_MIN <= this->_address && this->_address <= this->SLAVE_ADDRESS_MAX) {
        result = true;
    }
    return result;
}

/////////////////////////////////////////////////////////////////////
// MessageFrame
/////////////////////////////////////////////////////////////////////
MessageFrame::MessageFrame(MessageFrame::MODBUS_TYPE type)
{
    this->_type = type;
}

MessageFrame::~MessageFrame(void)
{
}

unsigned int MessageFrame::_ccitt(unsigned int *data, int len, int seed)
{
    const unsigned int POLY = 0xA001;

    unsigned int crc16 = seed;
    for (int i = 0; i < len; i++) {
        crc16 ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc16 & 0x01) {
                crc16 = (crc16 >> 1) ^ POLY;
            } else {
                crc16 >>= 1;
            }
        }
    }
    return (crc16 & 0xFFFF);
}
void MessageFrame::_calc_crc(bool first_generate)
{
    unsigned int crc    = 0xFFFF;
    unsigned int buf[3] = { this->address, this->function, (unsigned int)this->data_length };
    this->valid         = first_generate;

    crc = this->_ccitt(buf, (MODBUS_TYPE::MODBUS_TYPE_RTU_EX == this->_type) ? 3 : 2, crc);
    crc = this->_ccitt(this->data, this->data_length, crc);

    crc = ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF);

    if (this->footer == crc) {
        this->valid = true;
    }
    this->footer = crc;
}
void MessageFrame::_calc_lrc(bool first_generate)
{
    unsigned int lrc = (this->address + this->function) & 0xFF;
    this->valid      = first_generate;

    for (int i = 0; i < this->data_length; i++) {
        lrc = (lrc + this->data[i]) & 0xFF;
    }
    lrc = ~lrc;
    lrc = (lrc + 1) & 0xFF;

    if (this->footer == lrc) {
        this->valid = true;
    }
    this->footer = lrc;
}

void MessageFrame::make_frame(unsigned int address, unsigned int function, unsigned int *data, int len)
{
    this->address     = address;
    this->function    = function;
    this->data_length = len;
    for (int i = 0; i < this->data_length; i++) {
        this->data[i] = data[i];
    }
    this->calc_footer(true);
}
void MessageFrame::calc_footer(bool first_generate)
{
    switch (this->_type) {
        case MODBUS_TYPE::MODBUS_TYPE_ASCII:
            this->_calc_lrc(first_generate);
            break;
        case MODBUS_TYPE::MODBUS_TYPE_RTU:
        case MODBUS_TYPE::MODBUS_TYPE_RTU_EX:
            this->_calc_crc(first_generate);
            break;
        case MODBUS_TYPE::MODBUS_TYPE_TCP:
        default:
            this->_calc_crc(first_generate);
            break;
    }
}

void MessageFrame::happened_error(EXCEPTION_CODE error_code)
{
    this->error_code = error_code;
    if (0x80 > this->function) {
        this->function = this->function + 0x80;
    }
    this->data_length = 1;
    this->data[0]     = (unsigned int)this->error_code;
    this->calc_footer(true);
}
