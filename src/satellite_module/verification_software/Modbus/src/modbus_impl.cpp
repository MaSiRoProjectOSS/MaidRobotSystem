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

static void print_watermark()
{
    char *stack_name        = pcTaskGetName(NULL);
    UBaseType_t stack_cushy = uxTaskGetStackHighWaterMark(NULL);
    int stack_time          = (int)stack_cushy / 1024;
    int stack_mod           = (int)stack_cushy - (stack_time * 1024);
    log_i("STACK : %s [%d(= %d * 1024 + %d) / %d] ", stack_name, (int)stack_cushy, stack_time, stack_mod, ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE);
}
static void print_message_frame(MessageFrame frame)
{
    static char msg_buffer[512];
    sprintf(msg_buffer,
            "Address[%d] Func[%d] Len[%d] CRC[%04X] Data[%02X %02X %02X %02X %02X %02X %02X %02X]",
            frame.address,
            frame.function,
            frame.data_length,
            frame.footer,
            frame.data[0],
            frame.data[1],
            frame.data[2],
            frame.data[3],
            frame.data[4],
            frame.data[5],
            frame.data[6],
            frame.data[7]);
    log_i("%s", msg_buffer);
}

ModbusImpl::ModbusImpl(HardwareSerial *serial) : ModbusLibArduino(serial)
{
}

ModbusImpl::~ModbusImpl(void)
{
}
bool ModbusImpl::_call_exception(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_w("  EXCEPTION[%d]", (int)frame.error_code);
#endif
    return true;
}
bool ModbusImpl::_call_read_discrete_inputs(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("read_discrete_inputs");
#endif
    return true;
}
bool ModbusImpl::_call_read_coils(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("read_coils");
#endif
    return true;
}
bool ModbusImpl::_call_write_single_coil(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("write_single_coil");
#endif
    return true;
}
bool ModbusImpl::_call_write_multiple_coils(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("write_multiple_coils");
#endif
    return true;
}
bool ModbusImpl::_call_read_input_registers(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("read_input_registers");
#endif
    return true;
}
bool ModbusImpl::_call_read_holding_registers(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("read_holding_registers");
#endif
    return true;
}
bool ModbusImpl::_call_write_single_register(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("write_single_register");
#endif
    return true;
}
bool ModbusImpl::_call_write_multiple_registers(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("write_multiple_registers");
#endif
    return true;
}
bool ModbusImpl::_call_readwrite_multiple_registers(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("read/write_multiple_registers");
#endif
    return true;
}
bool ModbusImpl::_call_mask_write_register(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("mask_write_register");
#endif
    return true;
}
bool ModbusImpl::_call_read_fifo_queue(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("read_fifo_queue");
#endif
    return true;
}

bool ModbusImpl::_call_read_file_record(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("read_file_record");
#endif
    return true;
}
bool ModbusImpl::_call_write_file_record(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("write_file_record");
#endif
    return true;
}
bool ModbusImpl::_call_read_exception_status(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("read_exception_status");
#endif
    return true;
}
bool ModbusImpl::_call_diagnostics(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("diagnostics");
#endif
    return true;
}

bool ModbusImpl::_call_get_comm_event_counter(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("get_comm_event_counter");
#endif
    return true;
}
bool ModbusImpl::_call_get_comm_event_log(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("get_comm_event_log");
#endif
    return true;
}
bool ModbusImpl::_call_report_server_id(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("report_server_id");
#endif
    return true;
}
bool ModbusImpl::_call_encapsulated_interface_transport(MessageFrame &frame)
{
    bool result = true;
#if DEBUG_TRACE
    log_i("encapsulated_interface_transport");
#endif
    switch (frame.data[0]) {
        case 0x0d: // can_open_general reference request and response
#if DEBUG_TRACE
            log_i("can_open_general");
#endif
            break;
        case 0x0e: // read_device_identification
#if DEBUG_TRACE
            log_i("read_device_identification");
#endif
            break;

        default:
#if DEBUG_TRACE
            log_i("unknown function");
#endif
            frame.happened_error(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
            break;
    }
    return result;
}
void ModbusImpl::_call_unknown(MessageFrame &frame)
{
#if DEBUG_TRACE
    log_i("unknown function");
#endif
    frame.happened_error(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
}

bool ModbusImpl::_reception(MessageFrame &frame)
{
    bool result = true;
    if (0x80 <= frame.function) {
        result = this->_call_exception(frame);
    } else {
        switch (frame.function) {
            ///////////////////////////////////
            // Data Access
            // - Bit access
            //   - Physical Discrete Inputs
            ///////////////////////////////////
            case MODBUS_FUNCTION::FUNCTION_READ_DISCRETE_INPUTS: // read_discrete_inputs
                result = this->_call_read_discrete_inputs(frame);
                break;
            ///////////////////////////////////
            // Data Access
            // - Bit access
            //   - Internal Bits or Physical Coils
            ///////////////////////////////////
            case MODBUS_FUNCTION::FUNCTION_READ_COILS: // read_coils
                result = this->_call_read_coils(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_WRITE_SINGLE_COIL: // write_single_coil
                result = this->_call_write_single_coil(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_WRITE_MULTIPLE_COILS: // write_multiple_coils
                result = this->_call_write_multiple_coils(frame);
                break;
            ///////////////////////////////////
            // Data Access
            // - 16-bit access
            //   - Physical Discrete Inputs
            ///////////////////////////////////
            case MODBUS_FUNCTION::FUNCTION_READ_INPUT_REGISTERS: // read_input_registers
                result = this->_call_read_input_registers(frame);
                break;
            ///////////////////////////////////
            // Data Access
            // - 16-bit access
            //   - Internal Registers or Physical Output Registers
            ///////////////////////////////////
            case MODBUS_FUNCTION::FUNCTION_READ_HOLDING_REGISTERS: // read_holding_registers
                result = this->_call_read_holding_registers(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_WRITE_SINGLE_REGISTER: // write_single_register
                result = this->_call_write_single_register(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_WRITE_MULTIPLE_REGISTERS: // write_multiple_registers
                result = this->_call_write_multiple_registers(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_READWRITE_MULTIPLE_REGISTERS: // read/write_multiple_registers
                result = this->_call_readwrite_multiple_registers(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_MASK_WRITE_REGISTER: // mask_write_register
                result = this->_call_mask_write_register(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_READ_FIFO_QUEUE: // read_fifo_queue
                result = this->_call_read_fifo_queue(frame);
                break;
            ///////////////////////////////////
            // Data Access
            // - 16-bit access
            //   - File Record Access
            ///////////////////////////////////
            case MODBUS_FUNCTION::FUNCTION_READ_FILE_RECORD: // read_file_record
                result = this->_call_read_file_record(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_WRITE_FILE_RECORD: // write_file_record
                result = this->_call_write_file_record(frame);
                break;
            ///////////////////////////////////
            // Diagnostics
            ///////////////////////////////////
            case MODBUS_FUNCTION::FUNCTION_READ_EXCEPTION_STATUS: // read_exception_status (serial line only)
                result = this->_call_read_exception_status(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_DIAGNOSTICS: // diagnostics (serial line only)
                result = this->_call_diagnostics(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_GET_COMM_EVENT_COUNTER: // get_comm_event_counter (serial line only)
                result = this->_call_get_comm_event_counter(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_GET_COMM_EVENT_LOG: // get_comm_event_log (serial line only)
                result = this->_call_get_comm_event_log(frame);
                break;
            case MODBUS_FUNCTION::FUNCTION_REPORT_SERVER_ID: // report_server_id (serial line only)
                result = this->_call_report_server_id(frame);
                break;
            ///////////////////////////////////
            // Other
            ///////////////////////////////////
            case MODBUS_FUNCTION::FUNCTION_ENCAPSULATED_INTERFACE_TRANSPORT: // can_open_general reference request and response
                result = this->_call_encapsulated_interface_transport(frame);
                break;
            default:
                break;
        }
    }
    if (false == result) {
        this->_call_unknown(frame);
        result = false;
    }
    //if (true == this->is_range_slave_address())
    {
        print_message_frame(frame);
        print_watermark();
    }

    return result;
}
