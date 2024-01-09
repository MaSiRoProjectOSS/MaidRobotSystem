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

public:
    /**
     * @brief Initialize the Modbus library
     *
     * @param address The address of the device
     * @param type The type of Modbus protocol to use (default is MODBUS_TYPE_RTU)
     * @param baud The baud rate for serial communication (default is 115200)
     * @return true if initialization was successful, false otherwise
     */
    bool begin(int address, MessageFrame::MODBUS_TYPE type = MessageFrame::MODBUS_TYPE_RTU, unsigned long baud = 115200)
    {
        this->_serial->setRxBufferSize(256 * 2);
        this->_serial->setTxBufferSize(256 * 2);
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
                log_v("received: slave");
                if ((this->BROADCAST_ADDRESS == frame.address) || (this->_address == frame.address)) {
                    frame.calc_footer();
                    if (true == frame.valid) {
                        this->reception(frame);
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
                        log_w("receive error");
                    }
                } else {
                    // do nothing
                    log_d("not target address[my:%02X][req:%02X]", this->_address, frame.address);
                }
            } else if (0 == this->_address) {
                log_v("received: master");
                frame.calc_footer();
                this->reception(frame);
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
                                log_d("overflow");
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
                        log_i("timeout");
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
                log_v("received: slave");
                if ((this->BROADCAST_ADDRESS == frame.address) || (this->_address == frame.address)) {
                    frame.calc_footer();
                    if (true == frame.valid) {
                        this->reception(frame);
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
                        log_w("receive error");
                    }
                } else {
                    // do nothing
                    log_d("not target address[my:%02X][req:%02X]", this->_address, frame.address);
                }
            } else if (0 == this->_address) {
                log_v("received: master");
                frame.calc_footer();
                this->reception(frame);
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
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_VERBOSE
        char buffer[10] = { 0 };
        String str      = "";
#endif
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
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_VERBOSE
            sprintf(buffer, "%02X", frame.data[i]);
            str += buffer;
#endif
        }
        // lrc
        this->_serial->write((unsigned int)((frame.footer >> 8) & 0xFFu));
        this->_serial->write((unsigned int)((frame.footer) & 0xFFu));

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_VERBOSE
        log_v("%s:%02X%02X%02X%s%04X\r\n", frame.valid ? "T" : "F", frame.address, frame.function, frame.data_length, str.c_str(), frame.footer & 0xFFFF);
#endif
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
        this->_serial->printf(":%02X%02X%s%02X\r\n", frame.address, frame.function, str.c_str(), frame.footer & 0xFF);
        log_v("%s:%02X%02X%s%02X\r\n", frame.valid ? "T" : "F", frame.address, frame.function, str.c_str(), frame.footer & 0xFF);
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
};

#endif
