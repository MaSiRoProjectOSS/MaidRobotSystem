/**
 * @file zlac8015d_modbus.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-05
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ZLAC8015D_MODBUS_HPP
#define ZLAC8015D_MODBUS_HPP

#include "modbus_lib_arduino.hpp"

#include <Arduino.h>
#include <vector>

#ifndef ZLAC_MODBUS_TYPE
#define ZLAC_MODBUS_TYPE MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU
#endif
#ifndef MODBUS_TARGET_ADDRESS
#define MODBUS_TARGET_ADDRESS 0x01
#endif

class ZLAC8015DModbus : public ModbusLibArduino {
public:
    typedef enum zlac_terminal_function
    {
        ZLAC_TERMINAL_FUNCTION_UNDEFINED,
        ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE,
        ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE,
        ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL,
        ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL,
        ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL,

    } ZLAC_TERMINAL_FUNCTION;

    typedef enum modbus_target_motor
    {
        TARGET_MOTOR_INVALID = 0,
        TARGET_MOTOR_LEFT,
        TARGET_MOTOR_RIGHT,
        TARGET_MOTOR_ALL
    } MODBUS_TARGET_MOTOR;

    typedef enum modbus_driver_mode
    {
        NOT_INITIALIZED,
        POSITION_RELATIVE,
        POSITION_ABSOLUTE,
        VELOCITY,
        TORQUE,
        UNDEFINED
    } MODBUS_DRIVER_MODE;

    typedef enum zlac_control_word
    {
        CONTROL_WORD_UNDEFINED         = 0x00,
        CONTROL_WORD_EMERGENCY_STOP    = 0x05,
        CONTROL_WORD_CLEAR_FAULT       = 0x06,
        CONTROL_WORD_STOP              = 0x07,
        CONTROL_WORD_ENABLE            = 0x08,
        CONTROL_WORD_SYNCHRONOUS_START = 0x10, //(Position mode)
        CONTROL_WORD_START_LEFT        = 0x11,
        CONTROL_WORD_START_RIGHT       = 0x12,
    } ZLAC_CONTROL_WORD;
    typedef enum rs585_baud_rate
    {
        RS485_BAUD_RATE_128000 = 1,
        RS485_BAUD_RATE_115200 = 2,
        RS485_BAUD_RATE_57600  = 3,
        RS485_BAUD_RATE_38400  = 4,
        RS485_BAUD_RATE_19200  = 5,
        RS485_BAUD_RATE_9600   = 6,
        RS485_BAUD_RATE_INVALID
    } RS485_BAUD_RATE;
    typedef enum can_baud_rate
    {
        CAN_BAUD_RATE_1000K = 0,
        CAN_BAUD_RATE_500K  = 1,
        CAN_BAUD_RATE_250K  = 2,
        CAN_BAUD_RATE_125K  = 3,
        CAN_BAUD_RATE_100K  = 4,
        CAN_BAUD_RATE_50K   = 5,
        CAN_BAUD_RATE_25K   = 6,
        CAN_BAUD_RATE_INVALID,
    } CAN_BAUD_RATE;
    typedef enum zlac_stop_control
    {
        ZLAC_STOP_CONTROL_UNDEFINED                  = 0,
        ZLAC_STOP_CONTROL_STOP                       = 5,
        ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION    = 6,
        ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION = 7,
    } ZLAC_STOP_CONTROL;
    typedef enum zlac_status_word
    {
        ZLAC_STATUS_WORD_UNDEFINED,
        ZLAC_STATUS_WORD_SHAFT_RELEASE,
        ZLAC_STATUS_WORD_SHAFT_LOCK,
        ZLAC_STATUS_WORD_EMERGENCY_STOP,
        ZLAC_STATUS_WORD_ALARM
    } ZLAC_STATUS_WORD;
    typedef enum terminal_function
    {
        TERMINAL_FUNCTION_NONE,
        TERMINAL_FUNCTION_EMERGENCY_STOP,
        TERMINAL_FUNCTION_NC,
    } TERMINAL_FUNCTION;

    class zlac_error {
    public:
        zlac_error()
        {
            this->clear();
        }
        void check(int value)
        {
            this->clear();
            this->err_value = value;
            if (0x00 < (value & 0x01)) {
                this->over_voltage = true;
            }
            if (0x00 < (value & 0x02)) {
                this->under_voltage = true;
            }
            if (0x00 < (value & 0x04)) {
                this->over_current = true;
            }
            if (0x00 < (value & 0x08)) {
                this->over_load = true;
            }
            if (0x00 < (value & 0x10)) {
                this->current_out_of_tolerance = true;
            }
            if (0x00 < (value & 0x20)) {
                this->encoder_out_of_tolerance = true;
            }
            if (0x00 < (value & 0x40)) {
                this->velocity_out_of_tolerance = true;
            }
            if (0x00 < (value & 0x80)) {
                this->reference_voltage_error = true;
            }
            if (0x00 < (value & 0x100)) {
                this->eeprom_error = true;
            }
            if (0x00 < (value & 0x200)) {
                this->hall_error = true;
            }
            if (0x00 < (value & 0x400)) {
                this->motor_temperature_over_temperature = true;
            }
            if (0x00 < value) {
                this->no_error = false;
            }
        }
        void clear()
        {
            this->no_error                           = true;
            this->over_voltage                       = false;
            this->under_voltage                      = false;
            this->over_current                       = false;
            this->over_load                          = false;
            this->current_out_of_tolerance           = false;
            this->encoder_out_of_tolerance           = false;
            this->velocity_out_of_tolerance          = false;
            this->reference_voltage_error            = false;
            this->eeprom_error                       = false;
            this->hall_error                         = false;
            this->motor_temperature_over_temperature = false;
            this->err_value                          = 0;
        }
        bool no_error                           = true;
        bool over_voltage                       = false;
        bool under_voltage                      = false;
        bool over_current                       = false;
        bool over_load                          = false;
        bool current_out_of_tolerance           = false;
        bool encoder_out_of_tolerance           = false;
        bool velocity_out_of_tolerance          = false;
        bool reference_voltage_error            = false;
        bool eeprom_error                       = false;
        bool hall_error                         = false;
        bool motor_temperature_over_temperature = false;
        int err_value                           = 0;
    };

public:
    ZLAC8015DModbus()
    {
    }
    ~ZLAC8015DModbus()
    {
    }
    void set_address(unsigned int address)
    {
        this->_address = address;
    }
    bool _reception(MessageFrame &frame) override
    {
        return true;
    }

private:
    MODBUS_DRIVER_MODE _mode = MODBUS_DRIVER_MODE::NOT_INITIALIZED;
    MessageFrame _frame;
    unsigned int _address                        = MODBUS_TARGET_ADDRESS;
    const MessageFrame::MODBUS_TYPE _modbus_type = ZLAC_MODBUS_TYPE;

private:
    MessageFrame _modbus_send_read(unsigned long index, int size)
    {
        std::vector<unsigned int> arr = { //
                                          (unsigned int)((index >> 8) & 0xFF),
                                          (unsigned int)(index & 0xFF),
                                          (unsigned int)((size >> 8) & 0xFF),
                                          (unsigned int)(size & 0xFF)
        };
        this->_frame.make_frame( //
                this->_modbus_type,
                this->_address,
                MessageFrame::FUNCTION_READ_HOLDING_REGISTERS,
                arr.data(),
                arr.size());
        log_v("      ADR[0x%02X] Fun[0x%02X] Len[%d] CRC[0x%04X] Reg[0x%02X%02X] Data[%02X %02X %02X %02X %02X %02X %02X %02X]",
              this->_frame.address,
              this->_frame.function,
              this->_frame.data_length - 2,
              this->_frame.footer,
              this->_frame.data[0],
              this->_frame.data[1],
              this->_frame.data[2],
              this->_frame.data[3],
              this->_frame.data[4],
              this->_frame.data[5],
              this->_frame.data[6],
              this->_frame.data[7],
              this->_frame.data[8],
              this->_frame.data[9]);
        return this->send_frame(this->_frame);
    }

    MessageFrame _modbus_writer_single(unsigned long index, int data)
    {
        std::vector<unsigned int> arr = { //
                                          (unsigned int)((index >> 8) & 0xFF),
                                          (unsigned int)(index & 0xFF),
                                          (unsigned int)((data >> 8) & 0xFF),
                                          (unsigned int)(data & 0xFF)
        };
        this->_frame.make_frame( //
                this->_modbus_type,
                this->_address,
                MessageFrame::FUNCTION_WRITE_SINGLE_REGISTER,
                arr.data(),
                arr.size());
        log_v("  ADR[0x%02X] Fun[0x%02X] Len[%d] CRC[0x%04X] Reg[0x%02X%02X] Data[%02X %02X %02X %02X %02X %02X %02X %02X]",
              this->_frame.address,
              this->_frame.function,
              this->_frame.data_length - 2,
              this->_frame.footer,
              this->_frame.data[0],
              this->_frame.data[1],
              this->_frame.data[2],
              this->_frame.data[3],
              this->_frame.data[4],
              this->_frame.data[5],
              this->_frame.data[6],
              this->_frame.data[7],
              this->_frame.data[8],
              this->_frame.data[9]);
        return this->send_frame(this->_frame);
    }

    MessageFrame _modbus_writer_multiple(unsigned long start, std::vector<int> data)
    {
        std::vector<unsigned int> arr = { //
                                          (unsigned int)((start >> 8) & 0xFF),
                                          (unsigned int)(start & 0xFF),
                                          (unsigned int)((data.size() >> 8) & 0xFF),
                                          (unsigned int)(data.size() & 0xFF)
        };
        arr.push_back((unsigned int)(data.size() * 2));
        for (int i = 0; i < data.size(); ++i) {
            arr.push_back((unsigned int)((data[i] >> 8) & 0xFF));
            arr.push_back((unsigned int)(data[i] & 0xFF));
        }

        this->_frame.make_frame( //
                this->_modbus_type,
                this->_address,
                MessageFrame::FUNCTION_WRITE_MULTIPLE_REGISTERS,
                arr.data(),
                arr.size());
        log_v("ADR[0x%02X] Fun[0x%02X] Len[%d] CRC[0x%04X] Reg[0x%02X%02X] Size[0x%02X%02X / %d] Data[%02X %02X %02X %02X %02X %02X %02X %02X]",
              this->_frame.address,
              this->_frame.function,
              this->_frame.data_length,
              this->_frame.footer,
              this->_frame.data[0],
              this->_frame.data[1],
              this->_frame.data[2],
              this->_frame.data[3],
              this->_frame.data[4],
              this->_frame.data[5],
              this->_frame.data[6],
              this->_frame.data[7],
              this->_frame.data[8],
              this->_frame.data[9],
              this->_frame.data[10],
              this->_frame.data[11],
              this->_frame.data[12]);
        return this->send_frame(this->_frame);
    }

public:
    ////////////////
    // Common constant for Left and Right motors
    ////////////////
    /**
     * @brief Get the communication offline time object (0x2000h)
     */
    bool get_communication_offline_time(int *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2000u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the communication offline time object (0x2000h)
     */
    bool set_communication_offline_time(int value_ms, bool check = false)
    {
        bool result = false;
        if (0 <= value_ms && value_ms <= 32767) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2000u, value_ms);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value_ms) {
                    result = false;
                } else if (true == check) {
                    result = this->get_communication_offline_time(&buf);
                    if (true == result) {
                        if (buf != value_ms) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the rs485 node id object (0x2001h)
     */
    bool get_rs485_node_id(int *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2001u, 1);
        if (0x80 <= _frame.function) {
            *value = -1;
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the rs485 node id object (0x2001h)
     */
    bool set_rs485_node_id(int id, bool check = false)
    {
        bool result = false;
        if (0 <= id && id <= 127) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2001u, id);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != id) {
                    result = false;
                } else if (true == check) {
                    result = this->get_rs485_node_id(&buf);
                    if (true == result) {
                        if (buf != id) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the rs485 baud rate object (0x2002h)
     */
    bool get_rs485_baud_rate(RS485_BAUD_RATE *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2002u, 1);
        if (0x80 <= _frame.function) {
            *value = RS485_BAUD_RATE::RS485_BAUD_RATE_INVALID;
            result = false;
        } else {
            unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            switch (buffer) {
                case RS485_BAUD_RATE::RS485_BAUD_RATE_128000:
                    *value = RS485_BAUD_RATE::RS485_BAUD_RATE_128000;
                    break;
                case RS485_BAUD_RATE::RS485_BAUD_RATE_115200:
                    *value = RS485_BAUD_RATE::RS485_BAUD_RATE_115200;
                    break;
                case RS485_BAUD_RATE::RS485_BAUD_RATE_57600:
                    *value = RS485_BAUD_RATE::RS485_BAUD_RATE_57600;
                    break;
                case RS485_BAUD_RATE::RS485_BAUD_RATE_38400:
                    *value = RS485_BAUD_RATE::RS485_BAUD_RATE_38400;
                    break;
                case RS485_BAUD_RATE::RS485_BAUD_RATE_19200:
                    *value = RS485_BAUD_RATE::RS485_BAUD_RATE_19200;
                    break;
                case RS485_BAUD_RATE::RS485_BAUD_RATE_9600:
                    *value = RS485_BAUD_RATE::RS485_BAUD_RATE_9600;
                    break;
                default:
                    *value = RS485_BAUD_RATE::RS485_BAUD_RATE_INVALID;
                    result = false;
            }
        }
        return result;
    }
    /**
     * @brief Set the rs485 baud rate object (0x2002h)
     */
    bool set_rs485_baud_rate(RS485_BAUD_RATE baud, bool check = false)
    {
        bool result = true;
        int input   = 2;
        switch (baud) {
            case RS485_BAUD_RATE::RS485_BAUD_RATE_128000:
            case RS485_BAUD_RATE::RS485_BAUD_RATE_115200:
            case RS485_BAUD_RATE::RS485_BAUD_RATE_57600:
            case RS485_BAUD_RATE::RS485_BAUD_RATE_38400:
            case RS485_BAUD_RATE::RS485_BAUD_RATE_19200:
            case RS485_BAUD_RATE::RS485_BAUD_RATE_9600:
                input = (int)baud;
                break;
            default:
                result = false;
                log_w("not support baud rate");
                break;
        }

        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2002u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                RS485_BAUD_RATE buf = RS485_BAUD_RATE::RS485_BAUD_RATE_INVALID;
                unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                switch (buffer) {
                    case RS485_BAUD_RATE::RS485_BAUD_RATE_128000:
                        buf = RS485_BAUD_RATE::RS485_BAUD_RATE_128000;
                        break;
                    case RS485_BAUD_RATE::RS485_BAUD_RATE_115200:
                        buf = RS485_BAUD_RATE::RS485_BAUD_RATE_115200;
                        break;
                    case RS485_BAUD_RATE::RS485_BAUD_RATE_57600:
                        buf = RS485_BAUD_RATE::RS485_BAUD_RATE_57600;
                        break;
                    case RS485_BAUD_RATE::RS485_BAUD_RATE_38400:
                        buf = RS485_BAUD_RATE::RS485_BAUD_RATE_38400;
                        break;
                    case RS485_BAUD_RATE::RS485_BAUD_RATE_19200:
                        buf = RS485_BAUD_RATE::RS485_BAUD_RATE_19200;
                        break;
                    case RS485_BAUD_RATE::RS485_BAUD_RATE_9600:
                        buf = RS485_BAUD_RATE::RS485_BAUD_RATE_9600;
                        break;
                    default:
                        buf = RS485_BAUD_RATE::RS485_BAUD_RATE_INVALID;
                }
                if (buf != baud) {
                    result = false;
                } else if (true == check) {
                    result = this->get_rs485_baud_rate(&buf);
                    if (true == result) {
                        if (buf != baud) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the input signal status object (0x2003h)
     */
    bool get_input_signal_status(bool *x0, bool *x1)
    {
        bool result         = true;
        *x0                 = false;
        *x1                 = false;
        MessageFrame _frame = this->_modbus_send_read(0x2003u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int value = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0 < (value & 0x01)) {
                *x0 = true;
            }
            if (0 < (value & 0x02)) {
                *x1 = true;
            }
        }
        return result;
    }
    /**
     * @brief Get the out signal status object (0x2004h)
     */
    bool get_out_signal_status(bool *x0, bool *x1)
    {
        bool result         = true;
        *x0                 = false;
        *x1                 = false;
        MessageFrame _frame = this->_modbus_send_read(0x2004u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int value = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0 < (value & 0x01)) {
                *x0 = true;
            }
            if (0 < (value & 0x01)) {
                *x1 = true;
            }
        }
        return result;
    }
    /**
     * @brief Get the clear feedback position object (0x2005h)
     */
    bool get_clear_feedback_position(MODBUS_TARGET_MOTOR *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2005u, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_INVALID;
        } else {
            int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            switch (buf) {
                case 1:
                    *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_LEFT;
                    break;
                case 2:
                    *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_RIGHT;
                    break;
                case 3:
                    *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_ALL;
                    break;
                case 0:
                default:
                    *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_INVALID;
                    break;
            }
        }
        return result;
    }
    /**
     * @brief Set the clear feedback position object (0x2005h)
     */
    bool set_clear_feedback_position(MODBUS_TARGET_MOTOR target, bool check = false)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x2005u, (int)target);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            MODBUS_TARGET_MOTOR buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_INVALID;
            unsigned int buffer     = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            switch (buffer) {
                case 1:
                    buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_LEFT;
                    break;
                case 2:
                    buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_RIGHT;
                    break;
                case 3:
                    buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_ALL;
                    break;
                case 0:
                default:
                    buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_INVALID;
                    break;
            }
            if (buf != target) {
                result = false;
            } else if (true == check) {
                result = this->get_clear_feedback_position(&buf);
                if (true == result) {
                    if (buf != target) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
#if 0
    /**
     * @brief Get the reset the zero point in absolute position control object (0x2006h)
     */
    bool get_reset_the_zero_point_in_absolute_position_control(MODBUS_TARGET_MOTOR *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2006u, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_INVALID;
        } else {
            int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            switch (buf) {
                case 1:
                    *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_LEFT;
                    break;
                case 2:
                    *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_RIGHT;
                    break;
                case 3:
                    *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_ALL;
                    break;
                case 0:
                default:
                    *value = MODBUS_TARGET_MOTOR::TARGET_MOTOR_INVALID;
                    break;
            }
        }
        return result;
    }
#endif
    /**
     * @brief Reset the zero point in absolute position control object (0x2006h)
     */
    bool reset_the_zero_point_in_absolute_position_control(MODBUS_TARGET_MOTOR target, bool check = false)
    {
        bool result = false;
        switch (this->_mode) {
            case MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
                result = true;
                break;
            case MODBUS_DRIVER_MODE::POSITION_RELATIVE:
            case MODBUS_DRIVER_MODE::VELOCITY:
            case MODBUS_DRIVER_MODE::TORQUE:
            default:
                log_w("not support mode");
                break;
        }
        if (true == result) {
            unsigned int input  = (unsigned int)target;
            MessageFrame _frame = this->_modbus_writer_single(0x2006u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                MODBUS_TARGET_MOTOR buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_INVALID;
                unsigned int buffer     = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                switch (buffer) {
                    case 1:
                        buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_LEFT;
                        break;
                    case 2:
                        buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_RIGHT;
                        break;
                    case 3:
                        buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_ALL;
                        break;
                    case 0:
                    default:
                        buf = MODBUS_TARGET_MOTOR::TARGET_MOTOR_INVALID;
                        break;
                }
                if (buf != target) {
                    result = false;
                }
#if 0
                 else if (true == check) {
                    result = this->get_reset_the_zero_point_in_absolute_position_control(&buf);
                    if (true == result) {
                        if (buf != target) {
                            result = false;
                        }
                    }
                }
#endif
            }
        }
        return result;
    }
    /**
     * @brief Get the lock shaft state after power on object (0x2007h)
     */
    bool get_lock_shaft_state_after_power_on(bool *flag)
    {
        bool result         = true;
        *flag               = true;
        MessageFrame _frame = this->_modbus_send_read(0x2007u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0x00 == buf) {
                *flag = false;
            }
        }
        return result;
    }
    /**
     * @brief Set the lock shaft state after power on object (0x2007h)
     */
    bool set_lock_shaft_state_after_power_on(bool flag, bool check = false)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x2007u, flag ? 1 : 0);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            bool buf   = (0x00 == buffer) ? false : true;
            if (buf != flag) {
                result = false;
            } else if (true == check) {
                result = this->get_lock_shaft_state_after_power_on(&buf);
                if (true == result) {
                    if (buf != flag) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the maximum motor speed object (0x2008h)
     */
    bool get_maximum_motor_speed(int *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2008u, 1);
        if (0x80 <= _frame.function) {
            *value = -1;
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the maximum motor speed object (0x2008h)
     */
    bool set_maximum_motor_speed(int r_min, bool check = false)
    {
        bool result = false;
        if (1 <= r_min && r_min <= 10000) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2008u, r_min);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != r_min) {
                    result = false;
                } else if (true == check) {
                    result = this->get_maximum_motor_speed(&buf);
                    if (true == result) {
                        if (buf != r_min) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
#if 0
    bool get_register_parameter_settings(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_read(0x2009u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0x01 == buf) {
                *value = true;
            }
        }
        return result;
    }
    bool set_register_parameter_settings(bool restore_factory_settings, bool check = false)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x2009u, restore_factory_settings ? 1 : 0);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            bool buf            = (0x01 == buffer) ? true : false;
            if (buf != restore_factory_settings) {
                result = false;
            } else if (true == check) {
                result = this->get_register_parameter_settings(&buf);
                if (true == result) {
                    if (buf != restore_factory_settings) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
#else
    bool restore_factory_settings()
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x2009u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            bool buf            = (0x01 == buffer) ? true : false;
            if (buf != true) {
                result = false;
            }
        }
        return result;
    }

#endif
    /**
     * @brief Get the can node info object (0x200Ah -0x200Bh)
     */
    bool get_can_node_info(int *id, CAN_BAUD_RATE *baud)
    {
        bool result         = false;
        MessageFrame _frame = this->_modbus_send_read(0x200Au, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *id                = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            unsigned int value = (unsigned int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
            switch (value) {
                case 0:
                    *baud = CAN_BAUD_RATE::CAN_BAUD_RATE_1000K;
                    break;
                case 1:
                    *baud = CAN_BAUD_RATE::CAN_BAUD_RATE_500K;
                    break;
                case 2:
                    *baud = CAN_BAUD_RATE::CAN_BAUD_RATE_250K;
                    break;
                case 3:
                    *baud = CAN_BAUD_RATE::CAN_BAUD_RATE_125K;
                    break;
                case 4:
                    *baud = CAN_BAUD_RATE::CAN_BAUD_RATE_100K;
                    break;
                case 5:
                    *baud = CAN_BAUD_RATE::CAN_BAUD_RATE_50K;
                    break;
                case 6:
                    *baud = CAN_BAUD_RATE::CAN_BAUD_RATE_25K;
                    break;
                default:
                    *baud = CAN_BAUD_RATE::CAN_BAUD_RATE_INVALID;
                    break;
            }
            result = true;
        }
        return result;
    }
    /**
     * @brief Set the can node info object (0x200Ah -0x200Bh)
     */
    bool set_can_node_info(int id, CAN_BAUD_RATE baud, bool check = false)
    {
        bool result = true;
        if (1 <= id && id <= 127) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        int input_baud = 0;
        switch (baud) {
            case CAN_BAUD_RATE::CAN_BAUD_RATE_1000K:
                input_baud = 0;
                break;
            case CAN_BAUD_RATE::CAN_BAUD_RATE_500K:
                input_baud = 1;
                break;
            case CAN_BAUD_RATE::CAN_BAUD_RATE_250K:
                input_baud = 2;
                break;
            case CAN_BAUD_RATE::CAN_BAUD_RATE_125K:
                input_baud = 3;
                break;
            case CAN_BAUD_RATE::CAN_BAUD_RATE_100K:
                input_baud = 4;
                break;
            case CAN_BAUD_RATE::CAN_BAUD_RATE_50K:
                input_baud = 5;
                break;
            case CAN_BAUD_RATE::CAN_BAUD_RATE_25K:
                input_baud = 6;
                break;
            case CAN_BAUD_RATE::CAN_BAUD_RATE_INVALID:
                result = false;
                log_w("not support baud rate");
                break;
        }

        if (true == result) {
            std::vector<int> data = { id, input_baud };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x200Au, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_id             = 0;
                CAN_BAUD_RATE buf_baud = CAN_BAUD_RATE::CAN_BAUD_RATE_INVALID;
#else
                int buf_id          = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                unsigned int buffer = (unsigned int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
                switch (buffer) {
                    case 0:
                        buf_baud = CAN_BAUD_RATE::CAN_BAUD_RATE_1000K;
                        break;
                    case 1:
                        buf_baud = CAN_BAUD_RATE::CAN_BAUD_RATE_500K;
                        break;
                    case 2:
                        buf_baud = CAN_BAUD_RATE::CAN_BAUD_RATE_250K;
                        break;
                    case 3:
                        buf_baud = CAN_BAUD_RATE::CAN_BAUD_RATE_125K;
                        break;
                    case 4:
                        buf_baud = CAN_BAUD_RATE::CAN_BAUD_RATE_100K;
                        break;
                    case 5:
                        buf_baud = CAN_BAUD_RATE::CAN_BAUD_RATE_50K;
                        break;
                    case 6:
                        buf_baud = CAN_BAUD_RATE::CAN_BAUD_RATE_25K;
                        break;
                    default:
                        buf_baud = CAN_BAUD_RATE::CAN_BAUD_RATE_INVALID;
                        break;
                }
                if (buf_id != id) {
                    result = false;
                } else if (buf_baud != baud) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_can_node_info(&buf_id, &buf_baud);
                    if (true == result) {
                        if (buf_id != id) {
                            result = false;
                        }
                        if (buf_baud != baud) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the control mode object (0x200Dh)
     */
    bool get_control_mode(MODBUS_DRIVER_MODE *mode)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x200Du, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *mode  = MODBUS_DRIVER_MODE::NOT_INITIALIZED;
        } else {
            int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            switch (buf) {
                case 1:
                    *mode = MODBUS_DRIVER_MODE::POSITION_RELATIVE;
                    break;
                case 2:
                    *mode = MODBUS_DRIVER_MODE::POSITION_ABSOLUTE;
                    break;
                case 3:
                    *mode = MODBUS_DRIVER_MODE::VELOCITY;
                    break;
                case 4:
                    *mode = MODBUS_DRIVER_MODE::TORQUE;
                    break;
                case 0:
                    *mode = MODBUS_DRIVER_MODE::UNDEFINED;
                    break;
                default:
                    *mode = MODBUS_DRIVER_MODE::NOT_INITIALIZED;
                    break;
            }
            this->_mode = *mode;
        }
        return result;
    }
    /**
     * @brief Set the control mode object (0x200Dh)
     */
    bool set_control_mode(MODBUS_DRIVER_MODE mode, bool check = false)
    {
        bool result = true;
        int input   = 2;
        switch (mode) {
            case MODBUS_DRIVER_MODE::POSITION_RELATIVE:
                input = 1;
                break;
            case MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
                input = 2;
                break;
            case MODBUS_DRIVER_MODE::VELOCITY:
                input = 3;
                break;
            case MODBUS_DRIVER_MODE::TORQUE:
                input = 4;
                break;
            case MODBUS_DRIVER_MODE::UNDEFINED:
                input = 0;
                break;
            default:
                result = false;
                log_w("not support mode");
                break;
        }

        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x200Du, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                MODBUS_DRIVER_MODE buf = MODBUS_DRIVER_MODE::NOT_INITIALIZED;
                unsigned int buffer    = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                switch (buffer) {
                    case 1:
                        buf = MODBUS_DRIVER_MODE::POSITION_RELATIVE;
                        break;
                    case 2:
                        buf = MODBUS_DRIVER_MODE::POSITION_ABSOLUTE;
                        break;
                    case 3:
                        buf = MODBUS_DRIVER_MODE::VELOCITY;
                        break;
                    case 4:
                        buf = MODBUS_DRIVER_MODE::TORQUE;
                        break;
                    case 0:
                        buf = MODBUS_DRIVER_MODE::UNDEFINED;
                        break;
                    default:
                        buf = MODBUS_DRIVER_MODE::NOT_INITIALIZED;
                        break;
                }
                if (buf != mode) {
                    result = false;
                } else if (true == check) {
                    result = this->get_control_mode(&buf);
                    if (true == result) {
                        if (buf != mode) {
                            result = false;
                        }
                    }
                } else {
                    this->_mode = mode;
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the control word object (0x200Eh)
     */
    bool get_control_word(ZLAC_CONTROL_WORD *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x200Eu, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED;
        } else {
            int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            switch (buf) {
                case 0x05:
                    *value = ZLAC_CONTROL_WORD::CONTROL_WORD_EMERGENCY_STOP;
                    break;
                case 0x06:
                    *value = ZLAC_CONTROL_WORD::CONTROL_WORD_CLEAR_FAULT;
                    break;
                case 0x07:
                    *value = ZLAC_CONTROL_WORD::CONTROL_WORD_STOP;
                    break;
                case 0x08:
                    *value = ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE;
                    break;
                case 0x10:
                    *value = ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START;
                    break;
                case 0x11:
                    *value = ZLAC_CONTROL_WORD::CONTROL_WORD_START_LEFT;
                    break;
                case 0x12:
                    *value = ZLAC_CONTROL_WORD::CONTROL_WORD_START_RIGHT;
                    break;
                case 0x00:
                default:
                    *value = ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED;
                    break;
            }
        }
        return result;
    }
    /**
     * @brief clear fault (0x200Eh)
     */
    bool clear_fault()
    {
        return this->set_control_word(ZLAC_CONTROL_WORD::CONTROL_WORD_CLEAR_FAULT);
    }
    /**
     * @brief emergency stop (0x200Eh)
     */
    bool control_word_emergency_stop()
    {
        return this->set_control_word(ZLAC_CONTROL_WORD::CONTROL_WORD_EMERGENCY_STOP);
    }
    /**
     * @brief control enable (0x200Eh)
     */
    bool control_word_enable()
    {
        return this->set_control_word(ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE);
    }
    /**
     * @brief synchronous start (0x200Eh)
     */
    bool control_word_synchronous_start()
    {
        return this->set_control_word(ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START);
    }
    /**
     * @brief start left (0x200Eh)
     */
    bool control_word_start_left()
    {
        return this->set_control_word(ZLAC_CONTROL_WORD::CONTROL_WORD_START_LEFT);
    }
    /**
     * @brief start right (0x200Eh)
     */
    bool control_word_start_right()
    {
        return this->set_control_word(ZLAC_CONTROL_WORD::CONTROL_WORD_START_RIGHT);
    }
    /**
     * @brief control stop (0x200Eh)
     */
    bool control_word_stop()
    {
        return this->set_control_word(ZLAC_CONTROL_WORD::CONTROL_WORD_STOP);
    }
    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool control_word_none()
    {
        return this->set_control_word(ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED);
    }
    /**
     * @brief Set the control word object (0x200Eh)
     */
    bool set_control_word(ZLAC_CONTROL_WORD word, bool check = false)
    {
        bool result = true;
        int input   = 2;
        switch (word) {
            case ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED:
                input = 0x00;
                break;
            case ZLAC_CONTROL_WORD::CONTROL_WORD_EMERGENCY_STOP:
                input = 0x05;
                break;
            case ZLAC_CONTROL_WORD::CONTROL_WORD_CLEAR_FAULT:
                input = 0x06;
                break;
            case ZLAC_CONTROL_WORD::CONTROL_WORD_STOP:
                input = 0x07;
                break;
            case ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE:
                input = 0x08;
                break;
            case ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START:
                switch (this->_mode) {
                    case MODBUS_DRIVER_MODE::POSITION_RELATIVE:
                    case MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
                        input = 0x10; //(Position mode)
                        break;
                    case MODBUS_DRIVER_MODE::VELOCITY:
                    case MODBUS_DRIVER_MODE::TORQUE:
                    default:
                        result = false;
                        log_w("not support mode");
                        break;
                }
                break;
            case ZLAC_CONTROL_WORD::CONTROL_WORD_START_LEFT:
                input = 0x11;
                break;
            case ZLAC_CONTROL_WORD::CONTROL_WORD_START_RIGHT:
                input = 0x12;
                break;
            default:
                result = false;
                log_w("not support mode");
                break;
        }

        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x200Eu, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                ZLAC_CONTROL_WORD buf = ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED;
                unsigned int buffer   = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                switch (buffer) {
                    case 0x05:
                        buf = ZLAC_CONTROL_WORD::CONTROL_WORD_EMERGENCY_STOP;
                        break;
                    case 0x06:
                        buf = ZLAC_CONTROL_WORD::CONTROL_WORD_CLEAR_FAULT;
                        break;
                    case 0x07:
                        buf = ZLAC_CONTROL_WORD::CONTROL_WORD_STOP;
                        break;
                    case 0x08:
                        buf = ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE;
                        break;
                    case 0x10:
                        buf = ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START;
                        break;
                    case 0x11:
                        buf = ZLAC_CONTROL_WORD::CONTROL_WORD_START_LEFT;
                        break;
                    case 0x12:
                        buf = ZLAC_CONTROL_WORD::CONTROL_WORD_START_RIGHT;
                        break;
                    case 0x00:
                    default:
                        buf = ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED;
                        break;
                }
                if (buf != word) {
                    result = false;
                } else if (true == check) {
                    result = this->get_control_word(&buf);
                    if (true == result) {
                        if (buf != word) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the synchronous control status object (0x200Fh)
     */
    bool get_synchronous_control_status(bool *synchronous)
    {
        bool result         = true;
        *synchronous        = false;
        MessageFrame _frame = this->_modbus_send_read(0x200Fu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0x01 == buf) {
                *synchronous = true;
            }
        }
        return result;
    }
    /**
     * @brief Set the synchronous control status object (0x200Fh)
     */
    bool set_synchronous_control_status(bool synchronous, bool check = false)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x200Fu, synchronous ? 1 : 0);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            bool buf            = (0x01 == buffer) ? true : false;
            if (buf != synchronous) {
                result = false;
            } else if (true == check) {
                result = this->get_synchronous_control_status(&buf);
                if (true == result) {
                    if (buf != synchronous) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }

    /**
     * @brief store rw register to eperm (0x2010h)
     */
    bool store_rw_register_to_eperm()
    {
        bool result         = false;
        MessageFrame _frame = this->_modbus_send_read(0x2010u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0x01 == buf) {
                result = true;
            }
        }
        return result;
    }
    /**
     * @brief Get the quick stop control object (0x2011h)
     */
    bool get_quick_stop_control(ZLAC_STOP_CONTROL *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2011u, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
        } else {
            int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            switch (buf) {
                case 0x05:
                    *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP;
                    break;
                case 0x06:
                    *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION;
                    break;
                case 0x07:
                    *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION;
                    break;
                case 0x00:
                default:
                    *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
                    break;
            }
        }
        return result;
    }
    /**
     * @brief Set the quick stop control object (0x2011h)
     */
    bool set_quick_stop_control(ZLAC_STOP_CONTROL ctrl, bool check = false)
    {
        bool result = true;
        int input   = 2;
        switch (ctrl) {
            case ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP:
                input = 5;
                break;
            case ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION:
                input = 6;
                break;
            case ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION:
                input = 7;
                break;
            default:
                result = false;
                log_w("not support ZLAC_STOP_CONTROL");
                break;
        }

        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2011u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                ZLAC_STOP_CONTROL buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
                unsigned int buffer   = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                switch (buffer) {
                    case 0x05:
                        buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP;
                        break;
                    case 0x06:
                        buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION;
                        break;
                    case 0x07:
                        buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION;
                        break;
                    case 0x00:
                    default:
                        buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
                        break;
                }
                if (buf != ctrl) {
                    result = false;
                } else if (true == check) {
                    result = this->get_quick_stop_control(&buf);
                    if (true == result) {
                        if (buf != ctrl) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the close operation control object (0x2012h)
     */
    bool get_close_operation_control(bool *stop_normally)
    {
        bool result         = true;
        *stop_normally      = false;
        MessageFrame _frame = this->_modbus_send_read(0x2012u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0x01 == buf) {
                *stop_normally = true;
            }
        }
        return result;
    }
    /**
     * @brief Set the close operation control object (0x2012h)
     */
    bool set_close_operation_control(bool stop_normally, bool check = false)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x2012u, stop_normally ? 1 : 0);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            bool buf            = (0x01 == buffer) ? true : false;
            if (buf != stop_normally) {
                result = false;
            } else if (true == check) {
                bool buf = false;
                result   = this->get_close_operation_control(&buf);
                if (true == result) {
                    if (buf != stop_normally) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the disable control object (0x2013h)
     */
    bool get_disable_control(bool *stop)
    {
        bool result         = true;
        *stop               = false;
        MessageFrame _frame = this->_modbus_send_read(0x2013u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0x01 == buf) {
                *stop = true;
            }
        }
        return result;
    }
    /**
     * @brief Set the disable control object (0x2013h)
     */
    bool set_disable_control(bool stop, bool check = false)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x2013u, stop ? 1 : 0);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            bool buf            = (0x01 == buffer) ? true : false;
            if (buf != stop) {
                result = false;
            } else if (true == check) {
                result = this->get_disable_control(&buf);
                if (true == result) {
                    if (buf != stop) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the halt control object (0x2014h)
     */
    bool get_halt_control(ZLAC_STOP_CONTROL *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2014u, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
        } else {
            int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            switch (buf) {
                case 0x05:
                    *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP;
                    break;
                case 0x06:
                    *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION;
                    break;
                case 0x07:
                    *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION;
                    break;
                case 0x00:
                default:
                    *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
                    break;
            }
        }
        return result;
    }
    /**
     * @brief Set the halt control object (0x2014h)
     */
    bool set_halt_control(ZLAC_STOP_CONTROL ctrl, bool check = false)
    {
        bool result = true;
        int input   = 2;
        switch (ctrl) {
            case ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP:
                input = 5;
                break;
            case ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION:
                input = 6;
                break;
            case ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION:
                input = 7;
                break;
            default:
                result = false;
                log_w("not support ZLAC_STOP_CONTROL");
                break;
        }

        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2014u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                ZLAC_STOP_CONTROL buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
                unsigned int buffer   = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                switch (buffer) {
                    case 0x05:
                        buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP;
                        break;
                    case 0x06:
                        buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION;
                        break;
                    case 0x07:
                        buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION;
                        break;
                    case 0x00:
                    default:
                        buf = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
                        break;
                }
                if (buf != ctrl) {
                    result = false;
                } else if (true == check) {
                    result = this->get_halt_control(&buf);
                    if (true == result) {
                        if (buf != ctrl) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the input effective level object (0x2016h)
     */
    bool get_input_effective_low_level(bool *x0, bool *x1)
    {
        bool result         = true;
        *x0                 = false;
        *x1                 = false;
        MessageFrame _frame = this->_modbus_send_read(0x2016u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0 < (0x01 & buf)) {
                *x0 = true;
            }
            if (0 < (0x02 & buf)) {
                *x1 = true;
            }
        }
        return result;
    }
    /**
     * @brief Set the input effective level object (0x2016h)
     */
    bool set_input_effective_low_level(bool x0, bool x1, bool check = false)
    {
        bool result         = true;
        int input           = (x0 ? 0x01 : 0x00) | (x1 ? 0x02 : 0x00);
        MessageFrame _frame = this->_modbus_writer_single(0x2016u, input);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            bool buf_x0         = (0 < (0x01 & buffer)) ? true : false;
            bool buf_x1         = (0 < (0x02 & buffer)) ? true : false;
            if (buf_x0 != x0) {
                result = false;
            } else if (buf_x1 != x1) {
                result = false;
            } else if (true == check) {
                result = this->get_input_effective_low_level(&buf_x0, &buf_x1);
                if (true == result) {
                    if (buf_x0 != x0) {
                        result = false;
                    }
                    if (buf_x1 != x1) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the input terminal terminal function selection object (0x2017h -0x2018h)
     */
    bool get_input_terminal_terminal_function_selection(TERMINAL_FUNCTION *x0, TERMINAL_FUNCTION *x1)
    {
        bool result         = true;
        *x0                 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_NC;
        *x1                 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_NC;
        MessageFrame _frame = this->_modbus_send_read(0x2017u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int value0 = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            unsigned int value1 = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            if (0 == value0) {
                *x0 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE;
            } else if (9 == value0) {
                *x0 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP;
            }
            if (0 == value1) {
                *x1 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE;
            } else if (9 == value1) {
                *x1 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP;
            }
        }
        return result;
    }
    /**
     * @brief Set the input terminal terminal function selection object (0x2017h -0x2018h)
     */
    bool set_input_terminal_terminal_function_selection(TERMINAL_FUNCTION x0, TERMINAL_FUNCTION x1, bool check = false)
    {
        bool result = true;
        std::vector<int> data;
        switch (x0) {
            case TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE:
                data.push_back(0);
                break;
            case TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP:
                data.push_back(9);
                break;
            default:
                result = false;
                log_w("Not support function: x0");
        }
        switch (x1) {
            case TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE:
                data.push_back(0);
                break;
            case TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP:
                data.push_back(9);
                break;
            default:
                log_w("Not support function: x1");
                result = false;
                break;
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_multiple(0x2017u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                TERMINAL_FUNCTION buf_x0 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_NC;
                TERMINAL_FUNCTION buf_x1 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_NC;
#if 0
                unsigned int value0      = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                unsigned int value1      = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                if (0 == value0) {
                    buf_x0 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE;
                } else if (9 == value0) {
                    buf_x0 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP;
                }
                if (0 == value1) {
                    buf_x1 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE;
                } else if (9 == value1) {
                    buf_x1 = TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP;
                }
                if (buf_x0 != x0) {
                    result = false;
                } else if (buf_x1 != x1) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_input_terminal_terminal_function_selection(&buf_x0, &buf_x1);
                    if (true == result) {
                        if (buf_x0 != x0) {
                            result = false;
                        }
                        if (buf_x1 != x1) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the output effective low level object (0x2019h -0x2020h)
     */
    bool get_output_effective_low_level(bool *y0, bool *y1, bool *b0, bool *b1)
    {
        bool result         = true;
        *y0                 = false;
        *y1                 = false;
        *b0                 = false;
        *b1                 = false;
        MessageFrame _frame = this->_modbus_send_read(0x2019u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int value = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0 < (0x01 & value)) {
                *y0 = true;
            }
            if (0 < (0x02 & value)) {
                *y1 = true;
            }
            if (0 < (0x04 & value)) {
                *b0 = true;
            }
            if (0 < (0x08 & value)) {
                *b1 = true;
            }
        }
        return result;
    }
    /**
     * @brief Set the output effective low level object (0x2019h -0x2020h)
     */
    bool set_output_effective_low_level(bool y0, bool y1, bool b0, bool b1, bool check = false)
    {
        bool result = true;
        int input   = (y0 ? 0x01 : 0x00) | //
                    (y1 ? 0x02 : 0x00) |   //
                    (b0 ? 0x04 : 0x00) |   //
                    (b1 ? 0x08 : 0x00);
        MessageFrame _frame = this->_modbus_writer_single(0x2019u, input);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            bool buf_y0         = false;
            bool buf_y1         = false;
            bool buf_b0         = false;
            bool buf_b1         = false;
            unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0 < (0x01 & buffer)) {
                buf_y0 = true;
            }
            if (0 < (0x02 & buffer)) {
                buf_y1 = true;
            }
            if (0 < (0x04 & buffer)) {
                buf_b0 = true;
            }
            if (0 < (0x08 & buffer)) {
                buf_b1 = true;
            }
            if (buf_y0 != y0) {
                result = false;
            } else if (buf_y1 != y1) {
                result = false;
            } else if (buf_b0 != b0) {
                result = false;
            } else if (buf_b1 != b1) {
                result = false;
            } else if (true == check) {
                result = this->get_output_effective_low_level(&buf_y0, &buf_y1, &buf_b0, &buf_b1);
                if (true == result) {
                    if (buf_y0 != y0) {
                        result = false;
                    }
                    if (buf_y1 != y1) {
                        result = false;
                    }
                    if (buf_b0 != b0) {
                        result = false;
                    }
                    if (buf_b1 != b1) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the output terminal function selection object (0x201Ah - 0x201Dh)
     */
    bool get_output_terminal_function_selection( //
            ZLAC_TERMINAL_FUNCTION *b0,
            ZLAC_TERMINAL_FUNCTION *b1,
            ZLAC_TERMINAL_FUNCTION *y0,
            ZLAC_TERMINAL_FUNCTION *y1)
    {
        bool result         = true;
        *b0                 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
        *b1                 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
        *y0                 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
        *y1                 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
        MessageFrame _frame = this->_modbus_send_read(0x201Au, 4);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int value0 = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            unsigned int value1 = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            unsigned int value2 = (_frame.data[4] << 8) | (_frame.data[5] & 0xFF);
            unsigned int value3 = (_frame.data[6] << 8) | (_frame.data[7] & 0xFF);
            if (0 == value0) {
                *b0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE;
            } else if (1 == value0) {
                *b0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE;
            }
            if (0 == value1) {
                *b1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE;
            } else if (1 == value1) {
                *b1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE;
            }
            if (0 == value2) {
                *y0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
            } else if (1 == value2) {
                *y0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL;
            } else if (2 == value2) {
                *y0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL;
            } else if (3 == value2) {
                *y0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL;
            }
            if (0 == value3) {
                *y1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
            } else if (1 == value3) {
                *y1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL;
            } else if (2 == value3) {
                *y1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL;
            } else if (3 == value3) {
                *y1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL;
            }
        }
        return result;
    }
    /**
     * @brief Set the output terminal function selection object (0x201Ah - 0x201Dh)
     */
    bool set_output_terminal_function_selection( //
            ZLAC_TERMINAL_FUNCTION b0,
            ZLAC_TERMINAL_FUNCTION b1,
            ZLAC_TERMINAL_FUNCTION y0,
            ZLAC_TERMINAL_FUNCTION y1,
            bool check = false)
    {
        bool result = true;
        std::vector<int> data;
        switch (b0) {
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE:
                data.push_back(0);
                break;
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE:
                data.push_back(1);
                break;
            default:
                result = false;
                break;
        }
        switch (b1) {
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE:
                data.push_back(0);
                break;
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE:
                data.push_back(1);
                break;
            default:
                result = false;
                break;
        }
        switch (y0) {
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED:
                data.push_back(0);
                break;
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL:
                data.push_back(1);
                break;
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL:
                data.push_back(2);
                break;
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL:
                data.push_back(3);
                break;
            default:
                result = false;
                break;
        }
        switch (y1) {
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED:
                data.push_back(0);
                break;
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL:
                data.push_back(1);
                break;
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL:
                data.push_back(2);
                break;
            case ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL:
                data.push_back(3);
                break;
            default:
                result = false;
                break;
        }

        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_multiple(0x201Au, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                ZLAC_TERMINAL_FUNCTION buf_y0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
                ZLAC_TERMINAL_FUNCTION buf_y1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
                ZLAC_TERMINAL_FUNCTION buf_b0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
                ZLAC_TERMINAL_FUNCTION buf_b1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
#if 0
                unsigned int value0 = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                unsigned int value1 = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                unsigned int value2 = (_frame.data[4] << 8) | (_frame.data[5] & 0xFF);
                unsigned int value3 = (_frame.data[6] << 8) | (_frame.data[7] & 0xFF);
                if (0 == value0) {
                    buf_b0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE;
                } else if (1 == value0) {
                    buf_b0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE;
                }
                if (0 == value1) {
                    buf_b1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE;
                } else if (1 == value1) {
                    buf_b1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE;
                }
                if (0 == value2) {
                    buf_y0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
                } else if (1 == value2) {
                    buf_y0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL;
                } else if (2 == value2) {
                    buf_y0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL;
                } else if (3 == value2) {
                    buf_y0 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL;
                }
                if (0 == value3) {
                    buf_y1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED;
                } else if (1 == value3) {
                    buf_y1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL;
                } else if (2 == value3) {
                    buf_y1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL;
                } else if (3 == value3) {
                    buf_y1 = ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL;
                }
                if (buf_y0 != y0) {
                    result = false;
                } else if (buf_y1 != y1) {
                    result = false;
                } else if (buf_b0 != b0) {
                    result = false;
                } else if (buf_b1 != b1) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_output_terminal_function_selection(&buf_b0, &buf_b1, &buf_y0, &buf_y1);
                    if (true == result) {
                        if (buf_b0 != b0) {
                            result = false;
                        }
                        if (buf_b1 != b1) {
                            result = false;
                        }
                        if (buf_y0 != y0) {
                            result = false;
                        }
                        if (buf_y1 != y1) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the driver temperature protection threshold object (0x201Eh)
     */
    bool get_driver_temperature_protection_threshold(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_read(0x201Eu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
        }
        return result;
    }
    /**
     * @brief Set the driver temperature protection threshold object (0x201Eh)
     */
    bool set_driver_temperature_protection_threshold(double value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 120.0) {
            result = true;
        } else {
            log_w("Out of range [%f]", value);
        }
        if (true == result) {
            int input           = value * 10;
            MessageFrame _frame = this->_modbus_writer_single(0x201Eu, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                double buf = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
                if (0.1 < std::abs(buf - value)) {
                    result = false;
                } else if (true == check) {
                    result = this->get_driver_temperature_protection_threshold(&buf);
                    if (true == result) {
                        if (0.1 < std::abs(buf - value)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the alarm pwm processing method object (0x201Fh)
     */
    bool get_alarm_pwm_processing_method(bool *open)
    {
        bool result         = true;
        *open               = false;
        MessageFrame _frame = this->_modbus_send_read(0x201Fu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (unsigned int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            if (0x01 == buf) {
                *open = true;
            }
        }
        return result;
    }
    /**
     * @brief Set the alarm pwm processing method object (0x201Fh)
     */
    bool set_alarm_pwm_processing_method(bool open, bool check = false)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x201Fu, open ? 1 : 0);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buffer = (unsigned int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            bool buf            = (0x01 == buffer) ? true : false;
            if (buf != open) {
                result = false;
            } else if (true == check) {
                result = this->get_alarm_pwm_processing_method(&buf);
                if (true == result) {
                    if (buf != open) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the overload processing method object (0x2020h)
     */
    bool get_overload_processing_method(bool *open)
    {
        bool result         = true;
        *open               = false;
        MessageFrame _frame = this->_modbus_send_read(0x2020u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (unsigned int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            if (0x01 == buf) {
                *open = true;
            }
        }
        return result;
    }
    /**
     * @brief Set the overload processing method object (0x2020h)
     */
    bool set_overload_processing_method(bool open, bool check = false)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x2020u, open ? 1 : 0);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buffer = (unsigned int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            bool buf            = (0x01 == buffer) ? true : false;
            if (buf != open) {
                result = false;
            } else if (true == check) {
                result = this->get_overload_processing_method(&buf);
                if (true == result) {
                    if (buf != open) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the io emergency stop processing mode object (0x2021h)
     */
    bool get_io_emergency_stop_processing_mode(bool *lock_shaft)
    {
        bool result         = true;
        *lock_shaft         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2021u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (unsigned int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            if (0x01 == buf) {
                *lock_shaft = false;
            }
        }
        return result;
    }
    /**
     * @brief Set the io emergency stop processing mode object (0x2021h)
     */
    bool set_io_emergency_stop_processing_mode(bool lock_shaft, bool check = false)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_writer_single(0x2021u, lock_shaft ? 0 : 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buffer = (unsigned int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            bool buf            = (0x01 == buffer) ? false : true;
            if (buf != lock_shaft) {
                result = false;
            } else if (true == check) {
                result = set_overload_processing_method(&buf);
                if (true == result) {
                    if (buf != lock_shaft) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }

    ////////////////
    // Left motor parameter
    ////////////////
    /**
     * @brief Get the encoder line left object (0x2030h)
     */
    bool get_encoder_line_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2030u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the encoder line left object (0x2030h)
     */
    bool set_encoder_line_left(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 4096) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2030u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_encoder_line_left(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the hall offset angle left object (0x2031h)
     */
    bool get_hall_offset_angle_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2031u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if ((_frame.data[0] & 0x80u) > 0) {
                *value |= 0xFFFF0000u;
            }
        }
        return result;
    }
    /**
     * @brief Set the hall offset angle left object (0x2031h)
     */
    bool set_hall_offset_angle_left(int value, bool check = false)
    {
        bool result = false;
        if (-360 <= value && value <= 360) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2031u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                if ((_frame.data[0] & 0x80u) > 0) {
                    buf |= 0xFFFF0000u;
                }
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_hall_offset_angle_left(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the overload factor left object (0x2032h)
     */
    bool get_overload_factor_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2032u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the overload factor left object (0x2032h)
     */
    bool set_overload_factor_left(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 300) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2032u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_overload_factor_left(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the current left object (0x2033h)
     */
    bool get_current_left(double *rated, double *maximum)
    {
        bool result         = true;
        *rated              = 0.0;
        *maximum            = 0.0;
        MessageFrame _frame = this->_modbus_send_read(0x2033u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *rated   = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 0.1;
            *maximum = (double)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF)) * 0.1;
        }
        return result;
    }
    /**
     * @brief Set the rated current left object (0x2033h)
     */
    bool set_rated_current_left(double value, bool check = false)
    {
        bool result = false;
        int input   = value * 10;
        if (0 <= input && input <= 150) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2033u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                double buf_rated   = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 0.1;
                double buf_maximum = 0;
                if (0.1 < std::abs(buf_rated - value)) {
                    result = false;
                } else if (true == check) {
                    result = this->get_current_left(&buf_rated, &buf_maximum);
                    if (true == result) {
                        if (0.1 < std::abs(buf_rated - value)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Set the maximum current left object (0x2034h)
     */
    bool set_maximum_current_left(double value, bool check = false)
    {
        bool result = false;
        int input   = value * 10;
        if (0 <= input && input <= 300) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2034u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                double buf_rated   = 0;
                double buf_maximum = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 0.1;
                if (0.1 < std::abs(buf_maximum - value)) {
                    result = false;
                } else if (true == check) {
                    result = this->get_current_left(&buf_rated, &buf_maximum);
                    if (true == result) {
                        if (0.1 < std::abs(buf_maximum - value)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the overload protection time left object (0x2035h)
     */
    bool get_overload_protection_time_left(int *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_read(0x2035u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 10;
        }
        return result;
    }
    /**
     * @brief Set the overload protection time left object (0x2035h)
     */
    bool set_overload_protection_time_left(int value, bool check = false)
    {
        bool result = true;
        int input   = value / 10;
        if (0 <= input && input <= 6553) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2035u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != input) {
                    result = false;
                } else if (true == check) {
                    result = this->get_overload_protection_time_left(&buf);
                    if (true == result) {
                        if (buf != (input * 10)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the position following error threshold left object (0x2036h)
     */
    bool get_position_following_error_threshold_left(int *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_read(0x2036u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 10;
        }
        return result;
    }
    /**
     * @brief Set the position following error threshold left object (0x2036h)
     */
    bool set_position_following_error_threshold_left(int value, bool check = false)
    {
        bool result = true;
        int input   = value / 10;
        if (1 <= input && input <= 6553) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2036u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != input) {
                    result = false;
                } else if (true == check) {
                    result = this->get_position_following_error_threshold_left(&buf);
                    if (true == result) {
                        if (buf != (input * 10)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the velocity smoothing factor left object (0x2037h)
     */
    bool get_velocity_smoothing_factor_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2037u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the velocity smoothing factor left object (0x2037h)
     */
    bool set_velocity_smoothing_factor_left(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 30000) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2037u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_velocity_smoothing_factor_left(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the current loop left object (0x2038h)
     */
    bool get_current_loop_left(int *kp, int *ki)
    {
        bool result         = true;
        *kp                 = 0;
        *ki                 = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2038u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *ki = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the current loop left object (0x2038h - 0x2039h)
     */
    bool set_current_loop_left(int kp, int ki, bool check = false)
    {
        bool result = true;
        if (0 <= kp && kp <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= ki && ki <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { kp, ki };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x2038u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_kp = 0;
                int buf_ki = 0;
#else
                int buf_kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                int buf_ki = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
                if (buf_kp != kp) {
                    result = false;
                } else if (buf_ki != ki) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_current_loop_left(&buf_kp, &buf_ki);
                    if (true == result) {
                        if (buf_kp != kp) {
                            result = false;
                        }
                        if (buf_ki != ki) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the feedforward output smoothing factor left object (0x203Ah)
     */
    bool get_feedforward_output_smoothing_factor_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x203Au, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the feedforward output smoothing factor left object (0x203Ah)
     */
    bool set_feedforward_output_smoothing_factor_left(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 30000) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x203Au, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_feedforward_output_smoothing_factor_left(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the torque output smoothing factor left object (0x203Bh)
     */
    bool get_torque_output_smoothing_factor_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x203Bu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the torque output smoothing factor left object (0x203Bh)
     */
    bool set_torque_output_smoothing_factor_left(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 30000) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x203Bu, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_torque_output_smoothing_factor_left(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the velocity loop left object (0x203Ch -0x203Eh)
     */
    bool get_velocity_loop_left(int *kp, int *ki, int *kf)
    {
        bool result         = true;
        *kp                 = 0;
        *ki                 = 0;
        *kf                 = 0;
        MessageFrame _frame = this->_modbus_send_read(0x203Cu, 3);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *ki = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
            *kf = (int)((_frame.data[4] << 8) | (_frame.data[5] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the velocity loop left object (0x203Ch -0x203Eh)
     */
    bool set_velocity_loop_left(int kp, int ki, int kf, bool check = false)
    {
        bool result = true;
        if (0 <= kp && kp <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= ki && ki <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= kf && kf <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { kp, ki, kf };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x203Cu, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_kp = 0;
                int buf_ki = 0;
                int buf_kf = 0;
#else
                int buf_kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                int buf_ki = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
                int buf_kf = (int)((_frame.data[4] << 8) | (_frame.data[5] & 0xFF));
                if (buf_kp != kp) {
                    result = false;
                } else if (buf_ki != ki) {
                    result = false;
                } else if (buf_kf != kf) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_velocity_loop_left(&buf_kp, &buf_ki, &buf_kf);
                    if (true == result) {
                        if (buf_kp != kp) {
                            result = false;
                        }
                        if (buf_ki != ki) {
                            result = false;
                        }
                        if (buf_kf != kf) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the position loop left object (0x203Fh - 0x2040h)
     */
    bool get_position_loop_left(int *kp, int *kf)
    {
        bool result         = true;
        *kp                 = 0;
        *kf                 = 0;
        MessageFrame _frame = this->_modbus_send_read(0x203Fu, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *kf = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the position loop left object (0x203Fh - 0x2040h)
     */
    bool set_position_loop_left(int kp, int kf, bool check = false)
    {
        bool result = true;
        if (0 <= kp && kp <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= kf && kf <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { kp, kf };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x203Fu, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_kp = 0;
                int buf_kf = 0;
#else
                int buf_kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                int buf_kf = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
                if (buf_kp != kp) {
                    result = false;
                } else if (buf_kf != kf) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_position_loop_left(&buf_kp, &buf_kf);
                    if (true == result) {
                        if (buf_kp != kp) {
                            result = false;
                        }
                        if (buf_kf != kf) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the initial velocity left object (VELOCITY:0x2043h / POSITION:0x2044h)
     */
    bool get_initial_velocity_left(int *value)
    {
        bool result = true;
        *value      = 0;
        int address = 0x00;
        switch (this->_mode) {
            case MODBUS_DRIVER_MODE::VELOCITY:
                address = 0x2043u;
                break;
            case MODBUS_DRIVER_MODE::POSITION_RELATIVE:
            case MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
                address = 0x2044u;
                break;
            default:
                break;
        }
        if (0 != address) {
            MessageFrame _frame = this->_modbus_send_read(address, 1);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            }
        }
        return result;
    }
    /**
     * @brief Set the initial velocity left object (VELOCITY:0x2043h / POSITION:0x2044h)
     */
    bool set_initial_velocity_left(int value, bool check = false)
    {
        bool result = false;
        int address = 0x00;
        if (1 <= value && value <= 250) {
            result = true;
        } else {
            log_w("Out of range");
        }
        switch (this->_mode) {
            case MODBUS_DRIVER_MODE::VELOCITY:
                address = 0x2043u;
                break;
            case MODBUS_DRIVER_MODE::POSITION_RELATIVE:
            case MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
                address = 0x2044u;
                break;
            default:
                result = false;
                break;
        }
        if (0 != address) {
            if (true == result) {
                MessageFrame _frame = this->_modbus_writer_single(address, value);
                if (0x80 <= _frame.function) {
                    result = false;
                } else {
                    int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                    if (buf != value) {
                        result = false;
                    } else if (true == check) {
                        result = this->get_initial_velocity_left(&buf);
                        if (true == result) {
                            if (buf != value) {
                                result = false;
                            }
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the motor poles left object (0x2045h)
     */
    bool get_motor_poles_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2045u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the motor poles left object (0x2045h)
     */
    bool set_motor_poles_left(int value, bool check = false)
    {
        bool result = false;
        if (4 <= value && value <= 64) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2045u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_motor_poles_left(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the over temperature threshold left object (0x2046h)
     */
    bool get_over_temperature_threshold_left(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_read(0x2046u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
        }
        return result;
    }
    /**
     * @brief Set the over temperature threshold left object (0x2046h)
     */
    bool set_over_temperature_threshold_left(double value, bool check = false)
    {
        bool result = false;
        int input   = value * 10;
        if (0 <= input && input <= 1200) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2046u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                double buf = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
                if (0.1 < std::abs(buf - value)) {
                    result = false;
                } else if (true == check) {
                    result = this->get_over_temperature_threshold_left(&buf);
                    if (true == result) {
                        if (0.1 < std::abs(buf - value)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the velocity observer coefficient left object (0x2047h - 0x204Ah)
     */
    bool get_velocity_observer_coefficient_left(int *index1, int *index2, int *index3, int *index4)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2047u, 4);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *index1 = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *index2 = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
            *index3 = (int)((_frame.data[4] << 8) | (_frame.data[5] & 0xFF));
            *index4 = (int)((_frame.data[6] << 8) | (_frame.data[7] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the velocity observer coefficient left object (0x2047h - 0x204Ah)
     */
    bool set_velocity_observer_coefficient_left(int index1, int index2, int index3, int index4, bool check = false)
    {
        bool result = true;
        if (0 <= index1 && index1 <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= index2 && index2 <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= index3 && index3 <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= index4 && index4 <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { index1, index2, index3, index4 };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x2047u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_index1 = 0;
                int buf_index2 = 0;
                int buf_index3 = 0;
                int buf_index4 = 0;
#else
                int buf_index1 = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                int buf_index2 = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
                int buf_index3 = (int)((_frame.data[4] << 8) | (_frame.data[5] & 0xFF));
                int buf_index4 = (int)((_frame.data[6] << 8) | (_frame.data[7] & 0xFF));
                if (buf_index1 != index1) {
                    result = false;
                } else if (buf_index2 != index2) {
                    result = false;
                } else if (buf_index3 != index3) {
                    result = false;
                } else if (buf_index4 != index4) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_velocity_observer_coefficient_left(&buf_index1, &buf_index2, &buf_index3, &buf_index4);
                    if (true == result) {
                        if (buf_index1 != index1) {
                            result = false;
                        }
                        if (buf_index2 != index2) {
                            result = false;
                        }
                        if (buf_index3 != index3) {
                            result = false;
                        }
                        if (buf_index4 != index4) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }

    ////////////////
    // Right motor parameter
    ////////////////
    /**
     * @brief Get the encoder line right object (0x2060h)
     */
    bool get_encoder_line_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2060u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the encoder line right object (0x2060h)
     */
    bool set_encoder_line_right(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 4096) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2060u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_encoder_line_right(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the hall offset angle right object (0x2061h)
     */
    bool get_hall_offset_angle_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2061u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if ((_frame.data[0] & 0x80u) > 0) {
                *value |= 0xFFFF0000u;
            }
        }
        return result;
    }
    /**
     * @brief Set the hall offset angle right object (0x2061h)
     */
    bool set_hall_offset_angle_right(int value, bool check = false)
    {
        bool result = false;
        if (-360 <= value && value <= 360) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2061u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                if ((_frame.data[0] & 0x80u) > 0) {
                    buf |= 0xFFFF0000u;
                }
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_hall_offset_angle_right(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the overload factor right object (0x2062h)
     */
    bool get_overload_factor_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2062u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the overload factor right object (0x2062h)
     */
    bool set_overload_factor_right(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 300) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2062u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_overload_factor_right(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the current right object (0x2063h - 0x2064h)
     */
    bool get_current_right(double *rated, double *maximum)
    {
        bool result         = true;
        *rated              = 0.0;
        *maximum            = 0.0;
        MessageFrame _frame = this->_modbus_send_read(0x2063u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *rated   = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 0.1;
            *maximum = (double)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF)) * 0.1;
        }
        return result;
    }
    /**
     * @brief Set the rated current right object (0x2063h - 0x2064h)
     */
    bool set_rated_current_right(double value, bool check = false)
    {
        bool result = false;
        int input   = value * 10;
        if (0 <= input && input <= 150) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2063u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                double buf_rated   = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 0.1;
                double buf_maximum = 0;
                if (0.1 < std::abs(buf_rated - value)) {
                    result = false;
                } else if (true == check) {
                    result = this->get_current_right(&buf_rated, &buf_maximum);
                    if (true == result) {
                        if (0.1 < std::abs(buf_rated - value)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Set the maximum current right object (0x2064)
     */
    bool set_maximum_current_right(double value, bool check = false)
    {
        bool result = false;
        int input   = value * 10;
        if (0 <= input && input <= 300) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2064u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                double buf_rated   = 0;
                double buf_maximum = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 0.1;
                if (0.1 < std::abs(buf_maximum - value)) {
                    result = false;
                } else if (true == check) {
                    result = this->get_current_right(&buf_rated, &buf_maximum);
                    if (true == result) {
                        if (0.1 < std::abs(buf_maximum - value)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the overload protection time right object (0x2065h)
     */
    bool get_overload_protection_time_right(int *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_read(0x2065u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 10;
        }
        return result;
    }
    /**
     * @brief Set the overload protection time right object (0x2065h)
     */
    bool set_overload_protection_time_right(int value, bool check = false)
    {
        bool result = true;
        int input   = value / 10;
        if (0 <= input && input <= 6553) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2065u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != input) {
                    result = false;
                } else if (true == check) {
                    result = this->get_overload_protection_time_right(&buf);
                    if (true == result) {
                        if (buf != (input * 10)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the position following error threshold right object (0x2066h)
     */
    bool get_position_following_error_threshold_right(int *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_read(0x2066u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) * 10;
        }
        return result;
    }
    /**
     * @brief Set the position following error threshold right object (0x2066h)
     */
    bool set_position_following_error_threshold_right(int value, bool check = false)
    {
        bool result = true;
        int input   = value / 10;
        if (1 <= input && input <= 6553) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2066u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != input) {
                    result = false;
                } else if (true == check) {
                    result = this->get_position_following_error_threshold_right(&buf);
                    if (true == result) {
                        if (buf != (input * 10)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the velocity smoothing factor right object (0x2067h)
     */
    bool get_velocity_smoothing_factor_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2067u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the velocity smoothing factor right object (0x2067h)
     */
    bool set_velocity_smoothing_factor_right(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 30000) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2067u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_velocity_smoothing_factor_right(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the current loop right object (0x2068h - 0x2069h)
     */
    bool get_current_loop_right(int *kp, int *ki)
    {
        bool result         = true;
        *kp                 = 0;
        *ki                 = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2068u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *ki = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the current loop right object (0x2068h - 0x2069h)
     */
    bool set_current_loop_right(int kp, int ki, bool check = false)
    {
        bool result = true;
        if (0 <= kp && kp <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= ki && ki <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { kp, ki };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x2068u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_kp = 0;
                int buf_ki = 0;
#else
                int buf_kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                int buf_ki = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
                if (buf_kp != kp) {
                    result = false;
                } else if (buf_ki != ki) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_current_loop_right(&buf_kp, &buf_ki);
                    if (true == result) {
                        if (buf_kp != kp) {
                            result = false;
                        }
                        if (buf_ki != ki) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the feedforward output smoothing factor right object (0x206Ah)
     */
    bool get_feedforward_output_smoothing_factor_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x206Au, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the feedforward output smoothing factor right object (0x206Ah)
     */
    bool set_feedforward_output_smoothing_factor_right(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 30000) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x206Au, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_feedforward_output_smoothing_factor_right(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the torque output smoothing factor right object (0x206Bh)
     */
    bool get_torque_output_smoothing_factor_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x206Bu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the torque output smoothing factor right object (0x206Bh)
     */
    bool set_torque_output_smoothing_factor_right(int value, bool check = false)
    {
        bool result = false;
        if (0 <= value && value <= 30000) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x206Bu, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_torque_output_smoothing_factor_right(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the velocity loop right object (0x206Ch - 0x206Eh)
     */
    bool get_velocity_loop_right(int *kp, int *ki, int *kf)
    {
        bool result         = true;
        *kp                 = 0;
        *ki                 = 0;
        *kf                 = 0;
        MessageFrame _frame = this->_modbus_send_read(0x206Cu, 3);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *ki = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
            *kf = (int)((_frame.data[4] << 8) | (_frame.data[5] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the velocity loop right object (0x206Ch - 0x206Eh)
     */
    bool set_velocity_loop_right(int kp, int ki, int kf, bool check = false)
    {
        bool result = true;
        if (0 <= kp && kp <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= ki && ki <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= kf && kf <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { kp, ki, kf };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x206Cu, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_kp = 0;
                int buf_ki = 0;
                int buf_kf = 0;
#else
                int buf_kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                int buf_ki = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
                int buf_kf = (int)((_frame.data[4] << 8) | (_frame.data[5] & 0xFF));
                if (buf_kp != kp) {
                    result = false;
                } else if (buf_ki != ki) {
                    result = false;
                } else if (buf_kf != kf) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_velocity_loop_right(&buf_kp, &buf_ki, &buf_kf);
                    if (true == result) {
                        if (buf_kp != kp) {
                            result = false;
                        }
                        if (buf_ki != ki) {
                            result = false;
                        }
                        if (buf_kf != kf) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the position loop right object (0x206Fh)
     */
    bool get_position_loop_right(int *kp, int *kf)
    {
        bool result         = true;
        *kp                 = 0;
        *kf                 = 0;
        MessageFrame _frame = this->_modbus_send_read(0x206Fu, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *kf = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the position loop right object (0x206Fh)
     */
    bool set_position_loop_right(int kp, int kf, bool check = false)
    {
        bool result = true;
        if (0 <= kp && kp <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= kf && kf <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { kp, kf };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x206Fu, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_kp = 0;
                int buf_kf = 0;
#else
                int buf_kp = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                int buf_kf = (int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
                if (buf_kp != kp) {
                    result = false;
                } else if (buf_kf != kf) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_position_loop_right(&buf_kp, &buf_kf);
                    if (true == result) {
                        if (buf_kp != kp) {
                            result = false;
                        }
                        if (buf_kf != kf) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the initial velocity right object (VELOCITY:0x2043h / POSITION:0x2044)
     */
    bool get_initial_velocity_right(int *value)
    {
        bool result = true;
        *value      = 0;
        int address = 0x00;
        switch (this->_mode) {
            case MODBUS_DRIVER_MODE::VELOCITY:
                address = 0x2043u;
                break;
            case MODBUS_DRIVER_MODE::POSITION_RELATIVE:
            case MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
                address = 0x2044u;
                break;
            default:
                break;
        }
        if (0 != address) {
            MessageFrame _frame = this->_modbus_send_read(address, 1);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            }
        }
        return result;
    }
    /**
     * @brief Set the initial velocity right object (VELOCITY:0x2043h / POSITION:0x2044)
     */
    bool set_initial_velocity_right(int value, bool check = false)
    {
        bool result = false;
        int address = 0x00;
        if (1 <= value && value <= 250) {
            result = true;
        } else {
            log_w("Out of range");
        }
        switch (this->_mode) {
            case MODBUS_DRIVER_MODE::VELOCITY:
                address = 0x2043u;
                break;
            case MODBUS_DRIVER_MODE::POSITION_RELATIVE:
            case MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
                address = 0x2044u;
                break;
            default:
                result = false;
                break;
        }
        if (0 != address) {
            if (true == result) {
                MessageFrame _frame = this->_modbus_writer_single(address, value);
                if (0x80 <= _frame.function) {
                    result = false;
                } else {
                    int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                    if (buf != value) {
                        result = false;
                    } else if (true == check) {
                        result = this->get_initial_velocity_right(&buf);
                        if (true == result) {
                            if (buf != value) {
                                result = false;
                            }
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the motor poles right object (0x2075h)
     */
    bool get_motor_poles_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x2075u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Set the motor poles right object (0x2075h)
     */
    bool set_motor_poles_right(int value, bool check = false)
    {
        bool result = false;
        if (4 <= value && value <= 64) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2075u, value);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
                if (buf != value) {
                    result = false;
                } else if (true == check) {
                    result = this->get_motor_poles_right(&buf);
                    if (true == result) {
                        if (buf != value) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the over temperature threshold right object (0x2076h)
     */
    bool get_over_temperature_threshold_right(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_read(0x2076u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
        }
        return result;
    }
    /**
     * @brief Set the over temperature threshold right object (0x2076h)
     */
    bool set_over_temperature_threshold_right(double value, bool check = false)
    {
        bool result = false;
        int input   = value * 10;
        if (0 <= input && input <= 1200) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            MessageFrame _frame = this->_modbus_writer_single(0x2076u, input);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                double buf = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
                if (0.1 < std::abs(buf - value)) {
                    result = false;
                } else if (true == check) {
                    result = this->get_over_temperature_threshold_right(&buf);
                    if (true == result) {
                        if (0.1 < std::abs(buf - value)) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the velocity observer coefficient right object (0x2077h - 0x207Ah)
     */
    bool get_velocity_observer_coefficient_right(int *index1, int *index2, int *index3, int *index4)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2077u, 4);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *index1 = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *index2 = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            *index3 = (_frame.data[4] << 8) | (_frame.data[5] & 0xFF);
            *index4 = (_frame.data[6] << 8) | (_frame.data[7] & 0xFF);
        }
        return result;
    }
    /**
     * @brief Set the velocity observer coefficient right object (0x2077h - 0x207Ah)
     */
    bool set_velocity_observer_coefficient_right(int index1, int index2, int index3, int index4, bool check = false)
    {
        bool result = true;
        if (0 <= index1 && index1 <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= index2 && index2 <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= index3 && index3 <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (0 <= index4 && index4 <= 30000) {
            // do nothing
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { index1, index2, index3, index4 };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x2077u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_index1 = 0;
                int buf_index2 = 0;
                int buf_index3 = 0;
                int buf_index4 = 0;
#else
                int buf_index1 = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                int buf_index2 = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                int buf_index3 = (_frame.data[4] << 8) | (_frame.data[5] & 0xFF);
                int buf_index4 = (_frame.data[6] << 8) | (_frame.data[7] & 0xFF);
                if (buf_index1 != index1) {
                    result = false;
                } else if (buf_index2 != index2) {
                    result = false;
                } else if (buf_index3 != index3) {
                    result = false;
                } else if (buf_index4 != index4) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_velocity_observer_coefficient_right(&buf_index1, &buf_index2, &buf_index3, &buf_index4);
                    if (true == result) {
                        if (buf_index1 != index1) {
                            result = false;
                        }
                        if (buf_index2 != index2) {
                            result = false;
                        }
                        if (buf_index3 != index3) {
                            result = false;
                        }
                        if (buf_index4 != index4) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }

    ////////////////
    // Control parameter
    ////////////////
    /**
     * @brief Get the s-shape acceleration time object (0x2080h - 0x2081h)
     */
    bool get_s_shape_acceleration_time(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2080u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    /**
     * @brief Set the s-shape acceleration time object (0x2080h - 0x2081h)
     */
    bool set_s_shape_acceleration_time(int left, int right, bool check = false)
    {
        bool result = true;
        std::vector<int> data;
        if ((0 <= left) && (left <= 32767)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if ((0 <= right) && (right <= 32767)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { left, right };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x2080u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_left  = 0;
                int buf_right = 0;
#else
                int buf_left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                int buf_right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                if (buf_left != left) {
                    result = false;
                } else if (buf_right != right) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_s_shape_acceleration_time(&buf_left, &buf_right);
                    if (true == result) {
                        if (buf_left != left) {
                            result = false;
                        }
                        if (buf_right != right) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the s-shape deceleration time object (0x2082h - 0x2083h)
     */
    bool get_s_shape_deceleration_time(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2082u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    /**
     * @brief Set the s-shape deceleration time object (0x2082h - 0x2083h)
     */
    bool set_s_shape_deceleration_time(int left, int right, bool check = false)
    {
        bool result = true;
        std::vector<int> data;
        if ((0 <= left) && (left <= 32767)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if ((0 <= right) && (right <= 32767)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { left, right };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x2082u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_left  = 0;
                int buf_right = 0;
#else
                int buf_left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                int buf_right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                if (buf_left != left) {
                    result = false;
                } else if (buf_right != right) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_s_shape_deceleration_time(&buf_left, &buf_right);
                    if (true == result) {
                        if (buf_left != left) {
                            result = false;
                        }
                        if (buf_right != right) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the deceleration time of quick stop object (0x2084h - 0x2085h)
     */
    bool get_deceleration_time_of_quick_stop(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2084u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            return true;
        }
        return result;
    }
    /**
     * @brief Set the deceleration time of quick stop object (0x2084h - 0x2085h)
     */
    bool set_deceleration_time_of_quick_stop(int left, int right, bool check = false)
    {
        bool result = true;
        std::vector<int> data;
        if ((0 <= left) && (left <= 32767)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if ((0 <= right) && (right <= 32767)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { left, right };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x2084u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_left  = 0;
                int buf_right = 0;
#else
                int buf_left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                int buf_right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                if (buf_left != left) {
                    result = false;
                } else if (buf_right != right) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_deceleration_time_of_quick_stop(&buf_left, &buf_right);
                    if (true == result) {
                        if (buf_left != left) {
                            result = false;
                        }
                        if (buf_right != right) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the torque slope object (0x2086h - 0x2087h)
     */
    bool get_torque_slope(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2086u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            return true;
        }
        return result;
    }
    /**
     * @brief Set the torque slope object (0x2086h - 0x2087h)
     */
    bool set_torque_slope(int left, int right, bool check = false)
    {
        bool result           = true;
        std::vector<int> data = { left, right };
        MessageFrame _frame   = this->_modbus_writer_multiple(0x2086u, data);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
#if 1
            int buf_left  = 0;
            int buf_right = 0;
#else
            int buf_left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            int buf_right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            if (buf_left != left) {
                result = false;
            } else if (buf_right != right) {
                result = false;
            } else
#endif
            if (true == check) {
                result = this->get_torque_slope(&buf_left, &buf_right);
                if (true == result) {
                    if (buf_left != left) {
                        result = false;
                    }
                    if (buf_right != right) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the target velocity object (0x2088h - 0x2089h)
     */
    bool get_target_velocity(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2088u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = ((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *right = ((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
            if ((_frame.data[0] & 0x80u) > 0) {
                *left |= 0xFFFF0000u;
            }
            if ((_frame.data[2] & 0x80u) > 0) {
                *right |= 0xFFFF0000u;
            }
        }
        return result;
    }
    /**
     * @brief Set the target velocity object (0x2088h - 0x2089h)
     */
    bool set_target_velocity(int left, int right, bool check = false)
    {
        bool result = true;
        std::vector<int> data;
        if ((-3000 <= left) && (left <= 3000)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if ((-3000 <= right) && (right <= 3000)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { left, right };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x2088u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_left  = 0;
                int buf_right = 0;
#else
                int buf_left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                int buf_right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                if (buf_left != left) {
                    result = false;
                } else if (buf_right != right) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_target_velocity(&buf_left, &buf_right);
                    if (true == result) {
                        if (buf_left != left) {
                            result = false;
                        }
                        if (buf_right != right) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the target position object (0x208Ah - 0x208Dh)
     */
    bool get_target_position(long *left, long *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x208Au, 4);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 24) | (_frame.data[1] << 16) | (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            *right = (_frame.data[4] << 24) | (_frame.data[5] << 16) | (_frame.data[6] << 8) | (_frame.data[7] & 0xFF);
        }
        return result;
    }
    /**
     * @brief Set the target position object (0x208Ah - 0x208Dh)
     */
    bool set_target_position(long left, long right, bool check = false)
    {
        bool result = true;
        std::vector<int> data;
        if ((-0x7FFFFFFF <= left) && (left <= 0x7FFFFFFF)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if ((-0x7FFFFFFF <= right) && (right <= 0x7FFFFFFF)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { //
                                      (left >> 16) & 0xFFFF,
                                      left & 0xFFFF,
                                      (right >> 16) & 0xFFFF,
                                      right & 0xFFFF
            };
            MessageFrame _frame = this->_modbus_writer_multiple(0x208Au, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                long buf_left  = 0;
                long buf_right = 0;
#else
                long buf_left  = (_frame.data[0] << 24) | (_frame.data[1] << 16) | (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                long buf_right = (_frame.data[4] << 24) | (_frame.data[5] << 16) | (_frame.data[6] << 8) | (_frame.data[7] & 0xFF);
                if (buf_left != left) {
                    result = false;
                } else if (buf_right != right) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_target_position(&buf_left, &buf_right);
                    if (true == result) {
                        if (buf_left != left) {
                            result = false;
                        }
                        if (buf_right != right) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the max speed object (0x208Eh - 0x208Fh)
     */
    bool get_max_speed(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x208Eu, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    /**
     * @brief Set the max speed object (0x208Eh - 0x208Fh)
     */
    bool set_max_speed(int left, int right, bool check = false)
    {
        bool result = true;
        std::vector<int> data;
        if ((1 <= left) && (left <= 1000)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if ((1 <= right) && (right <= 1000)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { left, right };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x208Eu, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_left  = 0;
                int buf_right = 0;
#else
                int buf_left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                int buf_right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                if (buf_left != left) {
                    result = false;
                } else if (buf_right != right) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_max_speed(&buf_left, &buf_right);
                    if (true == result) {
                        if (buf_left != left) {
                            result = false;
                        }
                        if (buf_right != right) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    /**
     * @brief Get the target torque object (0x2090h - 0x2091h)
     */
    bool get_target_torque(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x2090u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = ((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *right = ((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
            if ((_frame.data[0] & 0x80u) > 0) {
                *left |= 0xFFFF0000u;
            }
            if ((_frame.data[2] & 0x80u) > 0) {
                *right |= 0xFFFF0000u;
            }
        }
        return result;
    }
    /**
     * @brief Set the target torque object (0x2090h - 0x2091h)
     */
    bool set_target_torque(int left, int right, bool check = false)
    {
        bool result = true;
        std::vector<int> data;
        if ((-30000 <= left) && (left <= 30000)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if ((-30000 <= right) && (right <= 30000)) {
            // do noting
        } else {
            result = false;
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<int> data = { left, right };
            MessageFrame _frame   = this->_modbus_writer_multiple(0x2090u, data);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
#if 1
                int buf_left  = 0;
                int buf_right = 0;
#else
                int buf_left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
                int buf_right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
                if (buf_left != left) {
                    result = false;
                } else if (buf_right != right) {
                    result = false;
                } else
#endif
                if (true == check) {
                    result = this->get_target_torque(&buf_left, &buf_right);
                    if (true == result) {
                        if (buf_left != left) {
                            result = false;
                        }
                        if (buf_right != right) {
                            result = false;
                        }
                    }
                }
            }
        }
        return result;
    }
    ////////////////
    // Read only parameter
    ////////////////
    /**
     * @brief Get the software version (0x20A0h)
     */
    bool get_software_version(int *version)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x20A0u, 1);
        if (0x80 <= _frame.function) {
            *version = -1;
            result   = false;
        } else {
            *version = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    /**
     * @brief Get the bus voltage object (0x20A1h)
     */
    bool get_bus_voltage(double *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x20A1u, 1);
        if (0x80 <= _frame.function) {
            *value = NAN;
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 100.0;
        }
        return result;
    }
    /**
     * @brief Get the status word object (0x20A2h)
     */
    bool get_status_word(bool *left_shaft_lock,
                         bool *left_emergency_stop,
                         bool *left_alarm,
                         bool *left_is_run,
                         bool *right_shaft_lock,
                         bool *right_emergency_stop,
                         bool *right_alarm,
                         bool *right_is_run)
    {
        bool result           = true;
        *left_shaft_lock      = false;
        *left_emergency_stop  = false;
        *left_alarm           = false;
        *left_is_run          = false;
        *right_shaft_lock     = false;
        *right_emergency_stop = false;
        *right_alarm          = false;
        *right_is_run         = false;

        int result_left     = 0;
        int result_right    = 0;
        MessageFrame _frame = this->_modbus_send_read(0x20A2u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            result_left  = _frame.data[0];
            result_right = _frame.data[1];
            // is_run
            if (0 < (result_left & 0x01)) {
                *left_is_run = true;
            }
            if (0 < (result_right & 0x01)) {
                *right_is_run = true;
            }
            // shaft_lock
            if (0 < (result_left & 0x40)) {
                *left_shaft_lock = true;
            }
            if (0 < (result_right & 0x40)) {
                *right_shaft_lock = true;
            }
            // emergency_stop
            if (0 < (result_left & 0x80)) {
                *left_emergency_stop = true;
            }
            if (0 < (result_right & 0x80)) {
                *right_emergency_stop = true;
            }
            // emergency_stop
            if (0 < (result_left & 0xC0)) {
                *left_alarm = true;
            }
            if (0 < (result_right & 0xC0)) {
                *right_alarm = true;
            }
        }
        return result;
    }
    /**
     * @brief Get the hall input state object (0x20A3h)
     */
    bool get_hall_input_state(bool *left_hall_err, bool *right_hall_err)
    {
        bool result         = true;
        *left_hall_err      = false;
        *right_hall_err     = false;
        MessageFrame _frame = this->_modbus_send_read(0x20A3u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            int result_left  = _frame.data[0];
            int result_right = _frame.data[1];
            if ((0 == result_left) || (7 == result_left)) {
                *left_hall_err = true;
            }
            if ((0 == result_right) || (7 == result_right)) {
                *left_hall_err = true;
            }
        }
        return result;
    }
    /**
     * @brief Get the motor temperature object (0x20A4h)
     */
    bool get_motor_temperature(int *left, int *right)
    {
        bool result         = true;
        *left               = 0;
        *right              = 0;
        MessageFrame _frame = this->_modbus_send_read(0x20A4u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = _frame.data[0];
            *right = _frame.data[1];
            if ((_frame.data[0] & 0x80u) > 0) {
                *left |= 0xFFFF0000u;
            }
            if ((_frame.data[2] & 0x80u) > 0) {
                *right |= 0xFFFF0000u;
            }
        }
        return result;
    }
    /**
     * @brief Get the error code object (0x20A5h -0x20A6h)
     */
    bool get_error_code(zlac_error *left, zlac_error *right)
    {
        bool result = true;
        left->clear();
        right->clear();
        MessageFrame _frame = this->_modbus_send_read(0x20A5u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            left->check((int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)));
            left->check((int)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF)));
        }
        return result;
    }
    /**
     * @brief Get the actual motor position object (0x20A7h -0x20AAh)
     */
    bool get_actual_motor_position(long *left, long *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x20A7u, 4);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left = (_frame.data[0] << 24)   //
                    | (_frame.data[1] << 16) //
                    | (_frame.data[2] << 8)  //
                    | (_frame.data[3] & 0xFF);
            *right = (_frame.data[4] << 24)   //
                     | (_frame.data[5] << 16) //
                     | (_frame.data[6] << 8)  //
                     | (_frame.data[7] & 0xFF);
        }
        return result;
    }
    /**
     * @brief Get the actual velocity object (0x20ABh - 0x20ACh)
     */
    bool get_actual_velocity(double *left, double *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x20ABu, 2);
        if (0x80 <= _frame.function) {
            *left  = NAN;
            *right = NAN;
            result = false;
        } else {
            unsigned int result_left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            unsigned int result_right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);

            if (0xF0000 <= result_left) {
                *left = (double)(0x7FFF & result_left) / -10.0;
            } else {
                *left = (double)result_left / 10.0;
            }
            if (0xF0000 <= result_right) {
                *right = (double)(0x7FFF & result_right) / -10.0;
            } else {
                *right = (double)result_right / 10.0;
            }
        }
        return result;
    }

    /**
     * @brief Get the actual torque object (0x20AD - 0x20AEh)
     */
    bool get_actual_torque(double *left, double *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_read(0x20ADu, 2);
        if (0x80 <= _frame.function) {
            *left  = NAN;
            *right = NAN;
            result = false;
        } else {
            int buffer_l = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            int buffer_r = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            if ((_frame.data[0] & 0x80u) > 0) {
                buffer_l |= 0xFFFF0000u;
            }
            if ((_frame.data[2] & 0x80u) > 0) {
                buffer_r |= 0xFFFF0000u;
            }
            *left  = (double)(buffer_l) / 10.0;
            *right = (double)(buffer_r) / 10.0;
        }
        return result;
    }
    /**
     * @brief Get the software connected status object (0x20AFh)
    */
    bool get_software_connected_status(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_read(0x20AFu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
#if 1
            *value = true;
#else
            int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            if (1 == buf) {
                *value = true;
            }
#endif
        }
        return result;
    }
    /**
     * @brief Get the driver temperature object (0x20B0h)
     */
    bool get_driver_temperature(double *value)
    {
        double result       = true;
        MessageFrame _frame = this->_modbus_send_read(0x20B0u, 1);
        if (0x80 <= _frame.function) {
            *value = NAN;
            result = false;
        } else {
            int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if ((_frame.data[0] & 0x80u) > 0) {
                buffer |= 0xFFFF0000u;
            }
            *value = (double)buffer / 10.0;
        }
        return result;
    }
};

#endif
