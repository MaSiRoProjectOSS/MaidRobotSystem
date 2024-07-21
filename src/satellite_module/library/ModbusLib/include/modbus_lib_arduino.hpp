/**
 * @file modbus_lib_arduino.hpp
 * @brief Modbus library for Arduino
 * @version 0.23.12
 * @date 2024-01-06
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef MODBUS_LIB_ARDUINO_HPP
#define MODBUS_LIB_ARDUINO_HPP

#include "modbus_lib.hpp"

#include <Arduino.h>

/**
 * @class ModbusLibArduino
 * @brief Modbus library for Arduino
 *
 * This class extends the ModbusLib class to provide functionality specific to Arduino.
 */
class ModbusLibArduino : public ModbusLib {
public:
    /**
     * @brief Constructor for ModbusLibArduino
     *
     * @param serial Pointer to a HardwareSerial object
     */
    ModbusLibArduino(HardwareSerial *serial, int timeout_times = 500) : ModbusLib()
    {
        this->_serial        = serial;
        this->_timeout_times = timeout_times;
    }
    /**
     * @brief Destructor for ModbusLibArduino
     */
    ~ModbusLibArduino()
    {
    }

    bool _reception(MessageFrame &frame)
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
        return result;
    }

public:
    /**
     * @brief Initialize the Modbus library
     *
     * @param address The address of the device
     * @param type The type of Modbus protocol to use (default is MODBUS_TYPE_RTU)
     * @param baud The baud rate for serial communication (default is 115200)
     * @return true if initialization was successful, false otherwise
     */
    bool begin( //
            int address,
            MessageFrame::MODBUS_TYPE type = MessageFrame::MODBUS_TYPE_RTU,
            unsigned long baud             = 115200)
    {
        this->_serial->setRxBufferSize(BUFFERSIZE_RX);
        this->_serial->setTxBufferSize(BUFFERSIZE_TX);
        this->_sleep_us = 1 + ((1000 * 1000) / (baud / 8));

        bool result = this->init(address, type);
        if (true == result) {
            if (true == this->is_range_slave_address()) {
                this->_serial->onReceiveError([this](hardwareSerial_error_t error) { this->on_receive_error(error); });
                switch (type) {
                    case MessageFrame::MODBUS_TYPE::MODBUS_TYPE_ASCII:
                        this->_serial->onReceive([this]() { this->on_receive_ascii(); });
                        break;
                    case MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU:
                    case MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU_EX:
                        this->_serial->onReceive([this]() { this->on_receive_rtu(); });
                        break;
                    case MessageFrame::MODBUS_TYPE::MODBUS_TYPE_TCP:
                        this->_serial->onReceive([this]() { this->on_receive_tcp(); });
                        break;
                    default:
                        break;
                }
            }

            this->_serial->begin(baud);
        }
        return result;
    }

    bool send(unsigned int address, MODBUS_FUNCTION function, unsigned int *data, int len)
    {
        return this->send(address, (unsigned int)function, data, len);
    }

    /**
     * @brief Send a Modbus message
     *
     * This function sends a Modbus message over the serial port. The message is constructed
     * using the provided function code and data.
     *
     * @param address The address of the Modbus device
     * @param function The Modbus function code to use in the message
     * @param data Pointer to an array of data to include in the message. The data should be in big-endian format.
     * @param len The number of elements in the data array
     * @return true if the message was sent successfully, false otherwise
     */
    bool send(unsigned int address, unsigned int function, unsigned int *data, int len)
    {
        bool result = false;
        if (0 == this->_address) {
            MessageFrame frame(this->_type);
            frame.make_frame(address, function, data, len);
            switch (this->_type) {
                case MessageFrame::MODBUS_TYPE::MODBUS_TYPE_ASCII:
                    this->_send_ascii(frame);
                    if (this->BROADCAST_ADDRESS != frame.address) {
                        for (int i = 0; i < this->_timeout_times; i++) {
                            result = this->on_receive_ascii();
                            if (true == result) {
                                break;
                            }
                            delayMicroseconds(this->_sleep_us);
                        }
                    } else {
                        // No response is returned in case of broadcast
                        result = true;
                    }
                    break;
                case MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU:
                case MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU_EX:
                    delayMicroseconds((uint32_t)(this->_sleep_us * 3.6));
                    this->_send_rtu(frame);
                    delayMicroseconds((uint32_t)(this->_sleep_us * 3.6));
                    if (this->BROADCAST_ADDRESS != frame.address) {
                        for (int i = 0; i < this->_timeout_times; i++) {
                            result = this->on_receive_rtu();
                            if (true == result) {
                                break;
                            }
                            delayMicroseconds(this->_sleep_us);
                        }
                    } else {
                        // No response is returned in case of broadcast
                        result = true;
                    }
                    break;
                case MessageFrame::MODBUS_TYPE::MODBUS_TYPE_TCP:
                    break;
                default:
                    break;
            }
        }
        return result;
    }

protected:
    void on_receive_error(hardwareSerial_error_t error)
    {
    }
    bool on_receive_ascii()
    {
        bool result = false;
        while (0 < this->_serial->available()) {
            MessageFrame frame(this->_type);
            int step    = 0;
            int buf     = 0;
            int timeout = 4;
            bool flag   = true;
            char c1     = 0x00;
            char c2     = 0x00;
            if (':' == this->_serial->read()) {
                while (true == flag) {
                    if (1 < this->_serial->available()) {
                        c1  = this->_serial->read();
                        c2  = this->_serial->read();
                        buf = this->_char_to_int(c1) << 4 | this->_char_to_int(c2);
                        switch (step) {
                            case 0:
                                frame.address = buf;
                                step++;
                                break;
                            case 1:
                                frame.function = buf;
                                step++;
                                break;
                            case 2:
                                if ((c1 == '\r') && (c2 == '\n')) {
                                    if (1 <= frame.data_length) {
                                        frame.footer      = (frame.data[frame.data_length - 1]);
                                        frame.data_length = frame.data_length - 1;
                                    } else {
                                        timeout = 0;
                                    }
                                    flag = false;
                                } else if (255 <= frame.data_length) {
                                    timeout = 0;
                                    flag    = false;
                                } else {
                                    frame.data[frame.data_length] = buf;
                                    frame.data_length++;
                                }
                                break;

                            default:
                                timeout = 0;
                                flag    = false;
                                break;
                        }
                    } else {
                        timeout--;
                        if (0 > timeout) {
                            flag = false;
                            break;
                        }
                        delayMicroseconds(this->_sleep_us);
                    }
                }
            }
            result = true;
            if (true == this->is_range_slave_address()) {
                if ((this->BROADCAST_ADDRESS == frame.address) || (this->_address == frame.address)) {
                    frame.calc_footer();
                    if (true == frame.valid) {
                        result = this->_reception(frame);
                        if (this->BROADCAST_ADDRESS != frame.address) {
                            frame.calc_footer(true);
                            this->_send_ascii(frame);
                        } else {
                            // do nothing
                            //   No response is returned in case of broadcast
                        }
                    } else {
                        frame.happened_error(MessageFrame::EXCEPTION_CODE::CODE_COMMUNICATION_ERROR);
                        this->_send_ascii(frame);
                    }
                } else {
                    // do nothing
                }
            } else if (0 == this->_address) {
                frame.calc_footer();
                result = this->_reception(frame);
            }
        }
        return result;
    }
    bool on_receive_rtu()
    {
        bool result = false;
        while (0 < this->_serial->available()) {
            MessageFrame frame(this->_type);
            int step         = 0;
            int buf          = 0;
            int timeout      = 4;
            bool flag        = true;
            bool last_char   = false;
            int count_length = 0;
            buf              = this->_serial->read();
            frame.address    = buf;
            step++;
            while (true == flag) {
                if (0 < this->_serial->available()) {
                    buf = this->_serial->read();
                    switch (step) {
                        case 1:
                            frame.function = buf;
                            step++;
                            break;
                        case 2:
                            count_length = 0;
                            if (MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU_EX == this->_type) {
                                frame.data_length = buf;
                            } else {
                                frame.data[count_length] = buf;
                                count_length++;
                            }
                            step++;
                            break;
                        case 3:
                            if (255 <= count_length) {
                                timeout = 0;
                                flag    = false;
                            } else {
                                frame.data[count_length] = buf;
                                count_length++;
                            }
                            break;

                        default:
                            timeout = 0;
                            flag    = false;
                            break;
                    }
                } else {
                    timeout--;
                    delayMicroseconds(this->_sleep_us);
                }
                // check last message
                if (MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU_EX == this->_type) {
                    if ((0 >= timeout) || (count_length >= (frame.data_length + 2))) {
                        last_char = true;
                    }
                } else {
                    if (0 >= timeout) {
                        last_char         = true;
                        frame.data_length = count_length - 2;
                    }
                }
                if (true == last_char) {
                    if (2 <= count_length) {
                        frame.footer = (frame.data[count_length - 2] << 8) | (frame.data[count_length - 1]);
                        timeout      = 4;
                    } else {
                        timeout = 0;
                    }
                    flag = false;
                }
                if (0 >= timeout) {
                    flag = false;
                }
            }
            result = true;
            if (true == this->is_range_slave_address()) {
                if ((this->BROADCAST_ADDRESS == frame.address) || (this->_address == frame.address)) {
                    frame.calc_footer();
                    if (true == frame.valid) {
                        result = this->_reception(frame);
                        if (this->BROADCAST_ADDRESS != frame.address) {
                            frame.calc_footer(true);
                            this->_send_rtu(frame);
                        } else {
                            // do nothing
                            //   No response is returned in case of broadcast
                        }
                    } else {
                        frame.happened_error(MessageFrame::CODE_COMMUNICATION_ERROR);
                        this->_send_rtu(frame);
                    }
                } else {
                    // do nothing
                }
            } else if (0 == this->_address) {
                frame.calc_footer();
                result = this->_reception(frame);
            }
        }
        return result;
    }
    bool on_receive_tcp()
    {
        bool result = false;
        return result;
    }

    /**
     * @brief Convert a character to an integer
     *
     * This function converts a character to its corresponding integer value.
     * It is used for converting ASCII characters to their numerical equivalents.
     *
     * @param c The character to convert
     * @return The integer value of the character
     */
    int _char_to_int(char c)
    {
        if ('0' <= c && c <= '9') {
            return c - '0';
        } else if ('A' <= c && c <= 'F') {
            return c - 'A' + 10;
        } else if ('a' <= c && c <= 'f') {
            return c - 'a' + 10;
        } else {
            return 0;
        }
    }
    /**
     * @brief Send a Modbus message in RTU format
     *
     * This function sends a Modbus message over the serial port in RTU (Remote Terminal Unit) format.
     * The message is constructed using the provided MessageFrame.
     *
     * @param frame The MessageFrame to send
     */
    void _send_rtu(MessageFrame frame)
    {
        // address
        this->_serial->write(frame.address);
        // function
        this->_serial->write(frame.function);
        if (MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU_EX == this->_type) {
            // length
            this->_serial->write(frame.data_length);
        }

        // data
        for (int i = 0; i < frame.data_length; i++) {
            this->_serial->write(frame.data[i]);
        }
        // lrc
        this->_serial->write((unsigned int)((frame.footer >> 8) & 0xFFu));
        this->_serial->write((unsigned int)((frame.footer) & 0xFFu));
    }
    /**
     * @brief Send a Modbus message in ASCII format
     *
     * This function sends a Modbus message over the serial port in ASCII (American Standard Code for Information Interchange) format.
     * The message is constructed using the provided MessageFrame.
     *
     * @param frame The MessageFrame to send
     */
    void _send_ascii(MessageFrame frame)
    {
        char buffer[10] = { 0 };
        String str      = "";
        // data
        for (int i = 0; i < frame.data_length; i++) {
            sprintf(buffer, "%02X", frame.data[i]);
            str += buffer;
        }
    }

private:
    /**
         * @brief Pointer to a HardwareSerial object
         *
         * This is used for serial communication with the Modbus device.
         */
    HardwareSerial *_serial;
    /**
         * @brief Sleep time in microseconds
         *
         * This is used to control the delay between sending and receiving data on the serial port.
         */
    uint32_t _sleep_us = 1;

    int _timeout_times = 500;

private:
    const int BUFFERSIZE_RX = (256 * 2);
    const int BUFFERSIZE_TX = (256 * 2);
};

#endif
