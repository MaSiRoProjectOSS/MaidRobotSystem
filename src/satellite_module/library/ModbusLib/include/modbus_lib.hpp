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
 * @struct MessageFrame
 * @brief Message frame
 *
 * This struct represents a Modbus message frame.
 */
class MessageFrame {
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
    /**
     * @enum EXCEPTION_CODE
     * @brief Exception codes
     *
     * This enum represents the different exception codes for Modbus communication.
     */
    enum EXCEPTION_CODE
    {
        CODE_NONE                  = 0x0,  ///< No error
        CODE_ILLEGAL_FUNCTION      = 0x01, ///< The server can't process the received function code.
        CODE_ILLEGAL_DATA_ADDRESS  = 0x02, ///< The server rejects queries with invalid data addresses.
        CODE_ILLEGAL_DATA_VALUE    = 0x03, ///< The server rejects queries with invalid data values.
        CODE_SERVER_DEVICE_FAILURE = 0x04, ///< The server encountered a fatal error while trying to execute a task.
        CODE_ACKNOWLEDGE           = 0x05, ///< The server is processing a programming command which takes a long time, and sends a response to prevent client timeout.
        CODE_SERVER_DEVICE_BUSY    = 0x06, ///< The server is busy processing a long-lasting programming command. The client should resend the message when the server is available.
        CODE_MEMORY_PARITY_ERROR   = 0x08, ///< The server detected a parity error in the memory.
        CODE_GATEWAY_PATH_UNAVAILABLE                = 0x0A, ///< The gateway couldn't allocate an internal communication path.
        CODE_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B, ///< In the context of gateways, no response from the target device usually indicates its absence from the network.

        ///////////////////////////////////////////////////////////////
        // This error code has been added. It is not standard.
        ///////////////////////////////////////////////////////////////
        CODE_COMMUNICATION_ERROR = 0x0F, ///< Communication error
    };

public:
    /**
     * @brief Constructor for MessageFrame
     *
     * This function constructs a MessageFrame using the provided type.
     *
     * @param type The type of Modbus protocol to use
     */
    MessageFrame(MODBUS_TYPE type);
    /**
     * @brief Destructor for MessageFrame
     */
    ~MessageFrame(void);

public:
    //int start;
    unsigned int address   = 0;     ///< The address of the Modbus device
    unsigned int function  = 0;     ///< The function code of the message frame
    unsigned int data[255] = { 0 }; ///< The data of the message frame
    unsigned int footer    = 0;     ///< The CRC or LRC of the message frame
    //int end;
    int data_length           = 0;                         ///< The length of the data
    bool valid                = false;                     ///< Flag indicating whether the message frame is valid
    EXCEPTION_CODE error_code = EXCEPTION_CODE::CODE_NONE; ///< The error code of the message frame
public:
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
    void make_frame(unsigned int address, unsigned int function, unsigned int *data, int len);
    /**
     * @brief Calculate the footer for a message frame
     *
     * This function calculates the footer (CRC or LRC) for a given MessageFrame.
     *
     * @param first_generate Flag indicating whether this is the first footer generation for the frame
     */
    void calc_footer(bool first_generate = false);
    /**
     * @brief Set the error code for a message frame
     *
     * This function sets the error code for a given MessageFrame.
     *
     * @param error_code The error code to set
     */
    void happened_error(EXCEPTION_CODE error_code);

private:
    /**
     * @brief Calculate the CRC for a message frame
     *
     * This function calculates the CRC (Cyclic Redundancy Check) for a given MessageFrame.
     *
     * @param first_generate Flag indicating whether this is the first CRC generation for the frame
     */
    void _calc_crc(bool first_generate = false);
    /**
     * @brief Calculate the LRC for a message frame
     *
     * This function calculates the LRC (Longitudinal Redundancy Check) for a given MessageFrame.
     *
     * @param first_generate Flag indicating whether this is the first LRC generation for the frame
     */
    void _calc_lrc(bool first_generate = false);

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
    unsigned int _ccitt(unsigned int *data, int len, int seed);

private:
    MODBUS_TYPE _type; ///< The type of Modbus protocol to use
};
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
