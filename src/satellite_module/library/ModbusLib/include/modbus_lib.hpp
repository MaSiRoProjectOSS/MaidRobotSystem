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

/**
 * @class ModbusLib
 * @brief Modbus library
 *
 * This class provides the base functionality for Modbus communication.
 */
class ModbusLib {
public:
    /**
     * @enum MODBUS_TYPE
     * @brief Modbus protocol types
     *
     * This enum represents the different types of Modbus protocols.
     */
    enum MODBUS_TYPE
    {
        MODBUS_TYPE_NONE,   ///< No Modbus protocol
        MODBUS_TYPE_ASCII,  ///< ASCII Modbus protocol
        MODBUS_TYPE_RTU,    ///< RTU Modbus protocol
        MODBUS_TYPE_RTU_EX, ///< RTU Modbus protocol / The first line of the data frame contains the number of data.
        MODBUS_TYPE_TCP,    ///< TCP Modbus protocol
    };

protected:
    /**
     * @struct MessageFrame
     * @brief Message frame
     *
     * This struct represents a Modbus message frame.
     */
    class MessageFrame {
    public:
        //int start;
        unsigned int address   = 0;     ///< The address of the Modbus device
        unsigned int function  = 0;     ///< The function code of the message frame
        unsigned int data[255] = { 0 }; ///< The data of the message frame
        unsigned int crc_lrc   = 0;     ///< The CRC or LRC of the message frame
        //int end;
        int data_length         = 0;     ///< The length of the data
        bool valid              = false; ///< Flag indicating whether the message frame is valid
        unsigned int error_code = 0;     ///< The error code of the message frame
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
    bool init(int address, MODBUS_TYPE type = MODBUS_TYPE_RTU);

public:
    /**
     * @brief Get the address of the Modbus device
     *
     * This function returns the address of the Modbus device.
     *
     * @return The address of the Modbus device
     */
    int get_address(void);

    /**
     * @brief Make a message frame
     *
     * This function creates a MessageFrame using the provided function code and data.
     *
     * @param address The address of the Modbus device
     * @param function The function code to use in the message frame
     * @param data The data to include in the message frame
     * @param len The length of the data
     * @return The created MessageFrame
     */
    MessageFrame make_frame(unsigned int address, unsigned int function, unsigned int *data, int len);

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
    /**
     * @brief Receive a Modbus message
     *
     * This function is called when a Modbus message is received. It processes the received MessageFrame.
     *
     * @param frame The received MessageFrame
     */
    void receive(MessageFrame &frame);
    /**
     * @brief Receive an error message
     *
     * This function is called when an error occurs during Modbus communication.
     *
     * @param frame The MessageFrame containing the error message
     */
    void receive_error(MessageFrame &frame);

    /**
     * @brief Calculate the CRC for a message frame
     *
     * This function calculates the CRC (Cyclic Redundancy Check) for a given MessageFrame.
     *
     * @param frame The MessageFrame to calculate the CRC for
     * @param first_generate Flag indicating whether this is the first CRC generation for the frame
     */
    void calc_crc(MessageFrame &frame, bool first_generate = false);
    /**
     * @brief Calculate the LRC for a message frame
     *
     * This function calculates the LRC (Longitudinal Redundancy Check) for a given MessageFrame.
     *
     * @param frame The MessageFrame to calculate the LRC for
     * @param first_generate Flag indicating whether this is the first LRC generation for the frame
     */
    void calc_lrc(MessageFrame &frame, bool first_generate = false);

    /**
     * @brief Calculate the CCITT checksum for a data array
     *
     * This function calculates the CCITT (International Telegraph and Telephone Consultative Committee) checksum for a given data array.
     *
     * @param data The data array to calculate the checksum for
     * @param len The length of the data array
     * @param seed The seed value for the checksum calculation
     * @return The calculated checksum
     */
    unsigned int ccitt(unsigned int *data, int len, int seed);

protected:
    int _address;      ///< The address of the Modbus device
    MODBUS_TYPE _type; ///< The type of Modbus protocol to use

private:
    const int BROADCAST_ADDRESS = 0;   ///< The broadcast address for Modbus communication
    const int SLAVE_ADDRESS_MIN = 1;   ///< The minimum slave address for Modbus communication
    const int SLAVE_ADDRESS_MAX = 247; ///< The maximum slave address for Modbus communication
};

#endif
