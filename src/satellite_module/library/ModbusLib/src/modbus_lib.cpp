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

ModbusLib::ModbusLib()
{
}
ModbusLib::~ModbusLib()
{
}
bool ModbusLib::init(int address, MODBUS_TYPE type)
{
    this->_type = type;
    if (0 <= address || address <= this->SLAVE_ADDRESS_MAX) {
        this->_address = address;
    } else {
        this->_type = MODBUS_TYPE::MODBUS_TYPE_NONE;
    }

    ///////////////////////////////////////////
    // TODO : Not support TCP
    if (this->_type == MODBUS_TYPE::MODBUS_TYPE_TCP) {
        this->_type = MODBUS_TYPE::MODBUS_TYPE_NONE;
    }
    ///////////////////////////////////////////

    if (this->_type == MODBUS_TYPE::MODBUS_TYPE_NONE) {
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
bool ModbusLib::is_range_slave_address()
{
    bool result = false;
    if (this->SLAVE_ADDRESS_MIN <= this->_address && this->_address <= this->SLAVE_ADDRESS_MAX) {
        result = true;
    }
    return result;
}

unsigned int ModbusLib::ccitt(unsigned int *data, int len, int seed)
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
void ModbusLib::calc_crc(ModbusLib::MessageFrame &frame, bool first_generate)
{
    unsigned int crc    = 0xFFFF;
    unsigned int buf[3] = { frame.address, frame.function, (unsigned int)frame.data_length };
    frame.valid         = first_generate;

    crc = this->ccitt(buf, (MODBUS_TYPE::MODBUS_TYPE_RTU_EX == this->_type) ? 3 : 2, crc);
    crc = this->ccitt(frame.data, frame.data_length, crc);

    crc = ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF);

    if (frame.crc_lrc == crc) {
        frame.valid = true;
    }
    frame.crc_lrc = crc;
}
void ModbusLib::calc_lrc(ModbusLib::MessageFrame &frame, bool first_generate)
{
    unsigned int lrc = (frame.address + frame.function) & 0xFF;
    frame.valid      = first_generate;

    for (int i = 0; i < frame.data_length; i++) {
        lrc = (lrc + frame.data[i]) & 0xFF;
    }
    lrc = ~lrc;
    lrc = (lrc + 1) & 0xFF;

    if (frame.crc_lrc == lrc) {
        frame.valid = true;
    }
    frame.crc_lrc = lrc;
}

ModbusLib::MessageFrame ModbusLib::make_frame(unsigned int address, unsigned int function, unsigned int *data, int len)
{
    MessageFrame frame;
    frame.address     = address;
    frame.function    = function;
    frame.data_length = len;
    for (int i = 0; i < frame.data_length; i++) {
        frame.data[i] = data[i];
    }
    switch (this->_type) {
        case MODBUS_TYPE::MODBUS_TYPE_ASCII:
            this->calc_lrc(frame, true);
            break;
        case MODBUS_TYPE::MODBUS_TYPE_RTU:
        case MODBUS_TYPE::MODBUS_TYPE_RTU_EX:
            this->calc_crc(frame, true);
            break;
        case MODBUS_TYPE::MODBUS_TYPE_TCP:
        default:
            this->calc_crc(frame, true);
            break;
    }
    return frame;
}

void ModbusLib::receive(ModbusLib::MessageFrame &frame)
{
    frame.error_code = 0;
    //frame.data_length = frame.function;
    // return frame;
}
void ModbusLib::receive_error(ModbusLib::MessageFrame &frame)
{
    frame.error_code = 1;
    //frame.data_length = 0;
}
