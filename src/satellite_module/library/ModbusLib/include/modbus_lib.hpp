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
    ModbusLib();
    /**
     * @brief Destructor for ModbusLib
     */
    ~ModbusLib();

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
    bool init(int address, MessageFrame::MODBUS_TYPE type = MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU_EX);

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
     * @brief Receive a message frame
     *
     * This function is called when a MessageFrame is received. It should be overridden by subclasses to provide specific reception behavior.
     *
     * @param frame The received MessageFrame
     * @return The processed MessageFrame
     */
    virtual MessageFrame reception(MessageFrame frame) = 0;

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
