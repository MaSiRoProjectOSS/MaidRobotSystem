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
    enum MODBUS_FUNCTION
    {
        FUNCTION_READ_COILS                       = 0x01,
        FUNCTION_READ_DISCRETE_INPUTS             = 0x02,
        FUNCTION_READ_HOLDING_REGISTERS           = 0x03,
        FUNCTION_READ_INPUT_REGISTERS             = 0x04,
        FUNCTION_WRITE_SINGLE_COIL                = 0x05,
        FUNCTION_WRITE_SINGLE_REGISTER            = 0x06,
        FUNCTION_READ_EXCEPTION_STATUS            = 0x07,
        FUNCTION_DIAGNOSTICS                      = 0x08,
        FUNCTION_GET_COMM_EVENT_COUNTER           = 0x0b,
        FUNCTION_GET_COMM_EVENT_LOG               = 0x0c,
        FUNCTION_WRITE_MULTIPLE_COILS             = 0x0F,
        FUNCTION_WRITE_MULTIPLE_REGISTERS         = 0x10,
        FUNCTION_REPORT_SERVER_ID                 = 0x11,
        FUNCTION_READ_FILE_RECORD                 = 0x14,
        FUNCTION_WRITE_FILE_RECORD                = 0x15,
        FUNCTION_MASK_WRITE_REGISTER              = 0x16,
        FUNCTION_READWRITE_MULTIPLE_REGISTERS     = 0x17,
        FUNCTION_READ_FIFO_QUEUE                  = 0x18,
        FUNCTION_ENCAPSULATED_INTERFACE_TRANSPORT = 0x2b,
    };

public:
    /**
     * @brief Constructor for ModbusLib
     */
    ModbusLib();
    /**
     * @brief Destructor for ModbusLib
     */
    ~ModbusLib();

protected:
    virtual bool _call_exception(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_read_discrete_inputs(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_read_coils(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_write_single_coil(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_write_multiple_coils(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_read_input_registers(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_read_holding_registers(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_write_single_register(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_write_multiple_registers(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_readwrite_multiple_registers(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_mask_write_register(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_read_fifo_queue(MessageFrame &frame)
    {
        return false;
    }

    virtual bool _call_read_file_record(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_write_file_record(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_read_exception_status(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_diagnostics(MessageFrame &frame)
    {
        return false;
    }

    virtual bool _call_get_comm_event_counter(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_get_comm_event_log(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_report_server_id(MessageFrame &frame)
    {
        return false;
    }
    virtual bool _call_encapsulated_interface_transport(MessageFrame &frame)
    {
        return false;
    }
    virtual void _call_unknown(MessageFrame &frame)
    {
        frame.happened_error(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
    }

    /**
     * @brief Receive a message frame
     *
     * This function is called when a MessageFrame is received. It should be overridden by subclasses to provide specific reception behavior.
     *
     * @param frame The received MessageFrame
     * @return The processed MessageFrame
     */
    virtual bool _reception(MessageFrame &frame) = 0;

    /**
     * @brief Initialize the Modbus library
     *
     * This function initializes the Modbus library. It should be overridden by subclasses to provide specific initialization behavior.
     *
     * @return True if the initialization was successful, false otherwise
     */
    virtual bool _init();

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
    bool init(int address, MessageFrame::MODBUS_TYPE type);

    /**
     * @brief Get the address of the Modbus device
     *
     * This function returns the address of the Modbus device.
     *
     * @return The address of the Modbus device
     */
    int get_address(void);
    /**
     * @brief Get the type of Modbus protocol to use
     *
     * This function returns the type of Modbus protocol to use.
     *
     * @return The type of Modbus protocol to use
     */
    MessageFrame::MODBUS_TYPE get_type(void);

protected:
    /**
     * @brief Check if the address is a broadcast address
     *
     * This function checks if the provided address is a broadcast address.
     *
     * @param address The address to check
     * @return True if the address is a broadcast address, false otherwise
     */
    bool is_range_slave_address();

protected:
    int _address;                    ///< The address of the Modbus device
    MessageFrame::MODBUS_TYPE _type; ///< The type of Modbus protocol to use

protected:
    const int BROADCAST_ADDRESS = 0;   ///< The broadcast address for Modbus communication
    const int SLAVE_ADDRESS_MIN = 1;   ///< The minimum slave address for Modbus communication
    const int SLAVE_ADDRESS_MAX = 247; ///< The maximum slave address for Modbus communication
};

#endif
