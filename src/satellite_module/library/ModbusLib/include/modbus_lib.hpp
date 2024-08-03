/**
 * @file modbus_lib.hpp
 * @brief Modbus library
 * @version 0.23.12
 * @date 2024-01-06
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef MODBUS_LIB_HPP
#define MODBUS_LIB_HPP

#include "modbus_message_frame.hpp"

/**
 * @class ModbusLib
 * @brief Modbus library
 *
 * This class provides the base functionality for Modbus communication.
 */
class ModbusLib {
public:
    /**
     * @brief Constructor for ModbusLib
     */
    ModbusLib()
    {
    }
    /**
     * @brief Destructor for ModbusLib
     */
    ~ModbusLib()
    {
    }

protected:
    virtual bool _call_exception(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_read_discrete_inputs(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_read_coils(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_write_single_coil(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_write_multiple_coils(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_read_input_registers(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_read_holding_registers(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_write_single_register(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_write_multiple_registers(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_readwrite_multiple_registers(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_mask_write_register(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_read_fifo_queue(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }

    virtual bool _call_read_file_record(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_write_file_record(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_read_exception_status(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_diagnostics(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }

    virtual bool _call_get_comm_event_counter(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_get_comm_event_log(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_report_server_id(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual bool _call_encapsulated_interface_transport(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
        return false;
    }
    virtual void _call_unknown(MessageFrame &frame)
    {
        frame.happened_error(this->_type, MessageFrame::EXCEPTION_CODE::CODE_SERVER_DEVICE_FAILURE);
    }

    /**
     * @brief Receive a message frame
     *
     * This function is called when a MessageFrame is received. It should be overridden by subclasses to provide specific reception behavior.
     *
     * @param frame The received MessageFrame
     * @return The processed MessageFrame
     */
    virtual bool _reception(MessageFrame &frame)
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
                case MessageFrame::FUNCTION_READ_DISCRETE_INPUTS: // read_discrete_inputs
                    result = this->_call_read_discrete_inputs(frame);
                    break;
                ///////////////////////////////////
                // Data Access
                // - Bit access
                //   - Internal Bits or Physical Coils
                ///////////////////////////////////
                case MessageFrame::FUNCTION_READ_COILS: // read_coils
                    result = this->_call_read_coils(frame);
                    break;
                case MessageFrame::FUNCTION_WRITE_SINGLE_COIL: // write_single_coil
                    result = this->_call_write_single_coil(frame);
                    break;
                case MessageFrame::FUNCTION_WRITE_MULTIPLE_COILS: // write_multiple_coils
                    result = this->_call_write_multiple_coils(frame);
                    break;
                ///////////////////////////////////
                // Data Access
                // - 16-bit access
                //   - Physical Discrete Inputs
                ///////////////////////////////////
                case MessageFrame::FUNCTION_READ_INPUT_REGISTERS: // read_input_registers
                    result = this->_call_read_input_registers(frame);
                    break;
                ///////////////////////////////////
                // Data Access
                // - 16-bit access
                //   - Internal Registers or Physical Output Registers
                ///////////////////////////////////
                case MessageFrame::FUNCTION_READ_HOLDING_REGISTERS: // read_holding_registers
                    result = this->_call_read_holding_registers(frame);
                    break;
                case MessageFrame::FUNCTION_WRITE_SINGLE_REGISTER: // write_single_register
                    result = this->_call_write_single_register(frame);
                    break;
                case MessageFrame::FUNCTION_WRITE_MULTIPLE_REGISTERS: // write_multiple_registers
                    result = this->_call_write_multiple_registers(frame);
                    break;
                case MessageFrame::FUNCTION_READWRITE_MULTIPLE_REGISTERS: // read/write_multiple_registers
                    result = this->_call_readwrite_multiple_registers(frame);
                    break;
                case MessageFrame::FUNCTION_MASK_WRITE_REGISTER: // mask_write_register
                    result = this->_call_mask_write_register(frame);
                    break;
                case MessageFrame::FUNCTION_READ_FIFO_QUEUE: // read_fifo_queue
                    result = this->_call_read_fifo_queue(frame);
                    break;
                ///////////////////////////////////
                // Data Access
                // - 16-bit access
                //   - File Record Access
                ///////////////////////////////////
                case MessageFrame::FUNCTION_READ_FILE_RECORD: // read_file_record
                    result = this->_call_read_file_record(frame);
                    break;
                case MessageFrame::FUNCTION_WRITE_FILE_RECORD: // write_file_record
                    result = this->_call_write_file_record(frame);
                    break;
                ///////////////////////////////////
                // Diagnostics
                ///////////////////////////////////
                case MessageFrame::FUNCTION_READ_EXCEPTION_STATUS: // read_exception_status (serial line only)
                    result = this->_call_read_exception_status(frame);
                    break;
                case MessageFrame::FUNCTION_DIAGNOSTICS: // diagnostics (serial line only)
                    result = this->_call_diagnostics(frame);
                    break;
                case MessageFrame::FUNCTION_GET_COMM_EVENT_COUNTER: // get_comm_event_counter (serial line only)
                    result = this->_call_get_comm_event_counter(frame);
                    break;
                case MessageFrame::FUNCTION_GET_COMM_EVENT_LOG: // get_comm_event_log (serial line only)
                    result = this->_call_get_comm_event_log(frame);
                    break;
                case MessageFrame::FUNCTION_REPORT_SERVER_ID: // report_server_id (serial line only)
                    result = this->_call_report_server_id(frame);
                    break;
                ///////////////////////////////////
                // Other
                ///////////////////////////////////
                case MessageFrame::FUNCTION_ENCAPSULATED_INTERFACE_TRANSPORT: // can_open_general reference request and response
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
        return result;
    }

    /**
     * @brief Initialize the Modbus library
     *
     * This function initializes the Modbus library. It should be overridden by subclasses to provide specific initialization behavior.
     *
     * @return True if the initialization was successful, false otherwise
     */
    virtual bool _init()
    {
        return true;
    }

public:
    /**
     * @brief Initialize the Modbus library
     *
     * This function initializes the Modbus library with the provided address and type.
     *
     * @param address The address of the Modbus device
     * @param type The type of Modbus protocol to use
     * @return True if the initialization was successful, false otherwise
     */
    bool init(int address, MessageFrame::MODBUS_TYPE type)
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
            return _init();
        }
    }

    /**
     * @brief Get the address of the Modbus device
     *
     * This function returns the address of the Modbus device.
     *
     * @return The address of the Modbus device
     */
    int get_address(void)
    {
        return this->_address;
    }
    /**
     * @brief Get the type of Modbus protocol to use
     *
     * This function returns the type of Modbus protocol to use.
     *
     * @return The type of Modbus protocol to use
     */
    MessageFrame::MODBUS_TYPE get_type(void)
    {
        return this->_type;
    }

protected:
    /**
     * @brief Check if the address is a broadcast address
     *
     * This function checks if the provided address is a broadcast address.
     *
     * @param address The address to check
     * @return True if the address is a broadcast address, false otherwise
     */
    bool is_range_slave_address()
    {
        bool result = false;
        if (this->SLAVE_ADDRESS_MIN <= this->_address) {
            if (this->_address <= this->SLAVE_ADDRESS_MAX) {
                result = true;
            }
        }
        return result;
    }

protected:
    int _address;                    ///< The address of the Modbus device
    MessageFrame::MODBUS_TYPE _type; ///< The type of Modbus protocol to use

protected:
    const int BROADCAST_ADDRESS = 0;   ///< The broadcast address for Modbus communication
    const int SLAVE_ADDRESS_MIN = 1;   ///< The minimum slave address for Modbus communication
    const int SLAVE_ADDRESS_MAX = 247; ///< The maximum slave address for Modbus communication
};

#endif
