/**
 * @file neck_controller.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-06
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "neck_controller.hpp"

// =============================
// Constructor
// =============================
NeckController::NeckController(HardwareSerial *serial, int pwm_size) : ModbusLibArduino(serial)
{
    this->pwm_servo         = new int[pwm_size];
    this->pwm_servo_request = new int[pwm_size];
    this->pwm_servo_count   = pwm_size;
}

NeckController::~NeckController(void)
{
    delete[] this->pwm_servo;
    delete[] this->pwm_servo_request;
}

// =============================
// PUBLIC : Function
// =============================
MessageFrame NeckController::reception(MessageFrame frame)
{
    /*
| Function type |               |                                                 | Function name                    | Function code | Comment     |
| ------------- | ------------- | ----------------------------------------------- | -------------------------------- | :-----------: | ----------- |
| Data Access   | Bit access    | Physical Discrete Inputs                        | Read Discrete Inputs             |    2(0x02)    | --          |
| ^             | ^             | Internal Bits or Physical Coils                 | Read Coils                       |    1(0x01)    | --          |
| ^             | ^             | ^                                               | Write Single Coil                |    5(0x05)    | --          |
| ^             | ^             | ^                                               | Write Multiple Coils             |   15(0x0F)    | --          |
| ^             | 16-bit access | Physical Input Registers                        | Read Input Registers             |    4(0x04)    | --          |
| ^             | ^             | Internal Registers or Physical Output Registers | Read Multiple Holding Registers  |    3(0x03)    | --          |
| ^             | ^             | ^                                               | Write Single Holding Register    |    6(0x06)    | --          |
| ^             | ^             | ^                                               | Write Multiple Holding Registers |   16(0x10)    | --          |
| ^             | ^             | ^                                               | Read/Write Multiple Registers    |   23(0x17)    | --          |
| ^             | ^             | ^                                               | Mask Write Register              |   22(0x16)    | --          |
| ^             | ^             | ^                                               | Read FIFO Queue                  |   24(0x18)    | --          |
| ^             | ^             | File Record Access                              | Read File Record                 |   20(0x14)    | --          |
| ^             | ^             | ^                                               | Write File Record                |   21(0x15)    | --          |
| Diagnostics   |               |                                                 | Read Exception Status            |    7(0x07)    | serial only |
| ^             | ^             | ^                                               | Diagnostic                       |    8(0x08)    | serial only |
| ^             | ^             | ^                                               | Get Com Event Counter            |   11(0x0B)    | serial only |
| ^             | ^             | ^                                               | Get Com Event Log                |   12(0x0C)    | serial only |
| ^             | ^             | ^                                               | Report Server ID                 |   17(0x11)    | serial only |
| ^             | ^             | ^                                               | Read Device Identification       |   43(0x2B)    | --          |
| Other         |               |                                                 | Encapsulated Interface Transport |   43(0x2B)    | --          |
    */
    int length = 0;
    switch (frame.function) {
        ///////////////////////////////////
        // Diagnostics
        ///////////////////////////////////
        case 0x07: // read_exception_status (serial line only)
            length            = this->_exception_queue.length();
            frame.data_length = length;
            for (int i = length; i >= 0; i--) {
                frame.data[i] = this->_exception_queue.pop();
            }
            break;
        case 0x08: // diagnostics (serial line only)
            break;
        case 0x0b: // get_comm_event_counter (serial line only)
            break;
        case 0x0c: // get_comm_event_log (serial line only)
            break;
        case 0x11: // report_server_id (serial line only)
            break;
        ///////////////////////////////////
        // Data Access
        // - 16-bit access
        //   - Internal Registers or Physical Output Registers
        ///////////////////////////////////
        case 0x03: // read_holding_registers
            this->read_holding_registers(frame);
            break;
        case 0x06: // write_single_register
            this->write_single_register(frame);
            break;
        ///////////////////////////////////
        // Data Access
        // - 16-bit access
        //   - Internal Registers or Physical Output Registers
        ///////////////////////////////////
        case 0x10: // write_multiple_registers
            this->write_multiple_registers(frame);
            break;
        case 0x17: // read/write_multiple_registers
        case 0x16: // mask_write_register
        case 0x18: // read_fifo_queue
        ///////////////////////////////////
        // Data Access
        // - 16-bit access
        //   - Physical Discrete Inputs
        ///////////////////////////////////
        case 0x04: // read_input_registers
        ///////////////////////////////////
        // Data Access
        // - 16-bit access
        //   - File Record Access
        ///////////////////////////////////
        case 0x14: // read_file_record
        case 0x15: // write_file_record
        ///////////////////////////////////
        // Data Access
        // - Bit access
        //   - Internal Bits or Physical Coils
        ///////////////////////////////////
        case 0x01: // read_coils
        case 0x05: // write_single_coil
        case 0x0f: // write_multiple_coils
        ///////////////////////////////////
        // Other
        ///////////////////////////////////
        case 0x2b: // can_open_general reference request and response
        ///////////////////////////////////
        // Data Access
        // - Bit access
        //   - Physical Discrete Inputs
        ///////////////////////////////////
        case 0x02: // read_discrete_inputs
        default:
            this->_exception_queue.push(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
            frame.happened_error(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
            break;
    }
    return frame;
}
void NeckController::set_accel(float x, float y, float z)
{
    this->accel.x = (uint16_t)(x * 1000);
    this->accel.y = (uint16_t)(y * 1000);
    this->accel.z = (uint16_t)(z * 1000);
}
void NeckController::set_gyro(float x, float y, float z)
{
    this->gyro.x = (uint16_t)(x * 1000);
    this->gyro.y = (uint16_t)(y * 1000);
    this->gyro.z = (uint16_t)(z * 1000);
}
bool NeckController::set_pwm_servo(int index, int value)
{
    bool result = false;
    if ((0 <= index) && (index < this->pwm_servo_count)) {
        this->pwm_servo_request[index] = value;
        result                         = true;
    }
    return result;
}

void NeckController::read_holding_registers(MessageFrame &frame)
{
    int length      = 0;
    int start_index = 1;
    if (0x68 == frame.data[0]) {
        length = this->_response_accel_and_gyro(frame, frame.data[1], (frame.data[2] << 8) | frame.data[3], start_index);
        start_index += length;
    }
    if (0x70 == frame.data[0]) {
        length = this->_response_pwm_servo(frame, frame.data[1], (frame.data[2] << 8) | frame.data[3], start_index);
        start_index += length;
    }
    frame.data_length = length;
    frame.data[0]     = length;
}

void NeckController::write_single_register(MessageFrame &frame)
{
    int length = 0;
    if (0x70 == frame.data[0]) {
        if (0x00 == frame.data[1]) {
            for (int i = 0; i < this->pwm_servo_count; i++) {
                this->set_pwm_servo(i, (int)((frame.data[2] << 8) | frame.data[3]));
                length++;
            }
        } else {
            this->set_pwm_servo((int)frame.data[1], (int)((frame.data[2] << 8) | frame.data[3]));
            length++;
        }
    }
    frame.data_length = 1;
    frame.data[0]     = length;
}
void NeckController::write_multiple_registers(MessageFrame &frame)
{
    int length = 0;
    if (0x70 == frame.data[0]) {
        if (0x00 == frame.data[1]) {
            length        = this->pwm_servo_count;
            frame.data[1] = 0x01;
        } else {
            length = (frame.data[2] << 8) | frame.data[3];
        }
        int count = 4;
        for (int i = frame.data[1]; (i < this->pwm_servo_count) && (i <= length); i++) {
            this->set_pwm_servo(i, (int)((frame.data[count++] << 8) | frame.data[count++]));
            length++;
        }
    }
    frame.data_length = 1;
    frame.data[0]     = length;
}

// =============================
// PRIVATE : Function
// =============================
int NeckController::_response_accel_and_gyro(MessageFrame &frame, unsigned int sub_address, unsigned int length, int start_index)
{
    bool is_skip = true;
    /*
        | Num    | Device type | --      | Note |
        | ------ | ----------- | ------- | ---- |
        | 0x6810 | IMU         | accel.x | --   |
        | 0x6811 | IMU         | accel.y | --   |
        | 0x6812 | IMU         | accel.z | --   |
        | 0x6813 | IMU         | gyro.x  | --   |
        | 0x6814 | IMU         | gyro.y  | --   |
        | 0x6815 | IMU         | gyro.z  | --   |
    */
    // accel
    if ((0x10 == sub_address) || ((false == is_skip) && (0 < length))) {
        frame.data[start_index++] = (this->accel.x >> 8) & 0xFF;
        frame.data[start_index++] = (this->accel.x >> 0) & 0xFF;
        length--;
        is_skip = false;
    }
    if ((0x11 == sub_address) || ((false == is_skip) && (0 < length))) {
        frame.data[start_index++] = (this->accel.y >> 8) & 0xFF;
        frame.data[start_index++] = (this->accel.y >> 0) & 0xFF;
        length--;
        is_skip = false;
    }
    if ((0x12 == sub_address) || ((false == is_skip) && (0 < length))) {
        frame.data[start_index++] = (this->accel.z >> 8) & 0xFF;
        frame.data[start_index++] = (this->accel.z >> 0) & 0xFF;
        length--;
        is_skip = false;
    }
    // gyro
    if ((0x13 == sub_address) || ((false == is_skip) && (0 < length))) {
        frame.data[start_index++] = (this->gyro.x >> 8) & 0xFF;
        frame.data[start_index++] = (this->gyro.x >> 0) & 0xFF;
        length--;
        is_skip = false;
    }
    if ((0x14 == sub_address) || ((false == is_skip) && (0 < length))) {
        frame.data[start_index++] = (this->gyro.y >> 8) & 0xFF;
        frame.data[start_index++] = (this->gyro.y >> 0) & 0xFF;
        length--;
        is_skip = false;
    }
    if ((0x15 == sub_address) || ((false == is_skip) && (0 < length))) {
        frame.data[start_index++] = (this->gyro.z >> 8) & 0xFF;
        frame.data[start_index++] = (this->gyro.z >> 0) & 0xFF;
        length--;
        is_skip = false;
    }

    return start_index;
}
int NeckController::_response_pwm_servo(MessageFrame &frame, unsigned int sub_address, unsigned int length, int start_index)
{
    bool is_skip = true;
    /*
        | Num    | Device type            | --  | Note          |
        | ------ | ---------------------- | --- | ------------- |
        | 0x7000 | PWM Motor(I2C) - ALL   | --  | for Broadcast |
        | 0x7001 | PWM Motor(I2C) - No.01 | --  |               |
        | 0x7002 | PWM Motor(I2C) - No.02 | --  |               |
        | 0x7003 | PWM Motor(I2C) - No.03 | --  |               |
        | 0x7004 | PWM Motor(I2C) - No.04 | --  |               |
        | 0x7005 | PWM Motor(I2C) - No.05 | --  |               |
    */
    if (0x00 == sub_address) {
        is_skip = false;
    }
    for (int i = 0; i < this->pwm_servo_count; i++) {
        if (((i + 1) == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->pwm_servo[i] >> 8) & 0xFF;
            frame.data[start_index++] = (this->pwm_servo[i] >> 0) & 0xFF;
            length--;
            is_skip = false;
        }
    }

    return start_index;
}
