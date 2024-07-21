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

    MessageFrame _call_exception(MessageFrame frame);
    MessageFrame _call_read_discrete_inputs(MessageFrame frame);
    MessageFrame _call_read_coils(MessageFrame frame);
    MessageFrame _call_write_single_coil(MessageFrame frame);
    MessageFrame _call_write_multiple_coils(MessageFrame frame);
    MessageFrame _call_read_input_registers(MessageFrame frame);
    MessageFrame _call_read_holding_registers(MessageFrame frame);
    MessageFrame _call_write_single_register(MessageFrame frame);
    MessageFrame _call_write_multiple_registers(MessageFrame frame);
    MessageFrame _call_readwrite_multiple_registers(MessageFrame frame);
    MessageFrame _call_mask_write_register(MessageFrame frame);
    MessageFrame _call_read_fifo_queue(MessageFrame frame);

    MessageFrame _call_read_file_record(MessageFrame frame);
    MessageFrame _call_write_file_record(MessageFrame frame);
    MessageFrame _call_read_exception_status(MessageFrame frame);
    MessageFrame _call_diagnostics(MessageFrame frame);

    MessageFrame _call_get_comm_event_counter(MessageFrame frame);
    MessageFrame _call_get_comm_event_log(MessageFrame frame);
    MessageFrame _call_report_server_id(MessageFrame frame);
    MessageFrame _call_encapsulated_interface_transport(MessageFrame frame);
    MessageFrame _call_unknown(MessageFrame frame);
};

#endif
