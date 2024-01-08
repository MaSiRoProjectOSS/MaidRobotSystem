/**
 * @file modbus_impl.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "modbus_impl.hpp"

ModbusImpl::ModbusImpl(HardwareSerial *serial) : ModbusLibArduino(serial)
{
}

ModbusImpl::~ModbusImpl(void)
{
}

ModbusImpl::MessageFrame ModbusImpl::reception(ModbusImpl::MessageFrame frame)
{
    switch (frame.function) {
        case 0x07: // read_exception_status (serial line only)
            log_i("  read_exception_status");
            break;
        case 0x08: // diagnostics (serial line only)
            log_i("  diagnostics");
            break;
        case 0x0b: // get_comm_event_counter (serial line only)
            log_i("  get_comm_event_counter");
            break;
        case 0x0c: // get_comm_event_log (serial line only)
            log_i("  get_comm_event_log");
            break;
        case 0x11: // report_server_id (serial line only)
            log_i("  report_server_id");
            break;
        case 0x01: // read_coils
            log_i("  read_coils");
            break;
        case 0x02: // read_discrete_inputs
            log_i("  read_discrete_inputs");
            break;
        case 0x03: // read_holding_registers
            log_i("  read_holding_registers");
            break;
        case 0x04: // read_input_registers
            log_i("  read_input_registers");
            break;
        case 0x05: // write_single_coil
            log_i("  write_single_coil");
            break;
        case 0x06: // write_single_register
            log_i("  write_single_register");
            break;
        case 0x0f: // write_multiple_coils
            log_i("  write_multiple_coils");
            break;
        case 0x10: // write_multiple_registers
            log_i("  write_multiple_registers");
            break;
        case 0x14: // read_file_record
            log_i("  read_file_record");
            break;
        case 0x15: // write_file_record
            log_i("  write_file_record");
            break;
        case 0x16: // mask_write_register
            log_i("  mask_write_register");
            break;
        case 0x17: // read/write_multiple_registers
            log_i("  read/write_multiple_registers");
            break;
        case 0x18: // read_fifo_queue
            log_i("  read_fifo_queue");
            break;
        case 0x2b: // encapsulated_interface_transport
            log_i("  encapsulated_interface_transport");
            break;
        case 0x0d: // can_open_general reference request and response
            log_i("  can_open_general");
            break;
        case 0x0e: // read_device_identification
            log_i("  read_device_identification");
            break;

        default:
            log_i("  unknown function");
            break;
    }
    return frame;
}
