/**
 * @file modbus_message_frame.hpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-11
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef MODBUS_MESSAGE_FRAME_HPP
#define MODBUS_MESSAGE_FRAME_HPP

/**
 * @struct MessageFrame
 * @brief Message frame
 *
 * This struct represents a Modbus message frame.
 */
class MessageFrame {
public:
    enum MODBUS_FUNCTION
    {
        FUNCTION_UNKNOWN                          = 0x00,
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
    MessageFrame();
    /**
     * @brief Destructor for MessageFrame
     */
    ~MessageFrame(void);

public:
    //int start;
    unsigned int address   = 0;                                               ///< The address of the Modbus device
    unsigned int function  = (unsigned int)MODBUS_FUNCTION::FUNCTION_UNKNOWN; ///< The function code of the message frame
    unsigned int data[255] = { 0 };                                           ///< The data of the message frame
    unsigned int footer    = 0;                                               ///< The CRC or LRC of the message frame
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
    void make_frame(MessageFrame::MODBUS_TYPE type, unsigned int address, MODBUS_FUNCTION function, unsigned int *data, int len);

    /**
     * @brief Calculate the footer for a message frame
     *
     * This function calculates the footer (CRC or LRC) for a given MessageFrame.
     *
     * @param first_generate Flag indicating whether this is the first footer generation for the frame
     */
    void calc_footer(MessageFrame::MODBUS_TYPE type, bool first_generate = false);
    /**
     * @brief Set the error code for a message frame
     *
     * This function sets the error code for a given MessageFrame.
     *
     * @param error_code The error code to set
     */
    void happened_error(MessageFrame::MODBUS_TYPE type, EXCEPTION_CODE error_code);

private:
    /**
     * @brief Calculate the CRC for a message frame
     *
     * This function calculates the CRC (Cyclic Redundancy Check) for a given MessageFrame.
     *
     * @param first_generate Flag indicating whether this is the first CRC generation for the frame
     */
    void _calc_crc(MessageFrame::MODBUS_TYPE type, bool first_generate = false);
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
    static unsigned int _ccitt(unsigned int *data, int len, int seed);
};
#endif
