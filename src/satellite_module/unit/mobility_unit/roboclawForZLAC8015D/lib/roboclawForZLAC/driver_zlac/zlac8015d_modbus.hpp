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
#include "zlac_driver.hpp"

#ifndef ZLAC_MODBUS_TYPE
#define ZLAC_MODBUS_TYPE MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU
#endif
#ifndef MODBUS_TARGET_ADDRESS
#define MODBUS_TARGET_ADDRESS 0x01
#endif

class ZLAC8015DCtrl : public ModbusLibArduino {
public:
    ZLAC8015DCtrl()
    {
    }
    ~ZLAC8015DCtrl()
    {
    }
    bool _reception(MessageFrame &frame) override
    {
        log_v("Address[%d] Func[%d] Len[%d] CRC[%04X] Data[%02X %02X %02X %02X %02X %02X %02X %02X]",
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
        return true;
    }
    void set_address(unsigned int address)
    {
        this->_address = address;
    }
    typedef enum zlac_terminal_function
    {
        ZLAC_TERMINAL_FUNCTION_UNDEFINED,
        ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE,
        ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE,
        ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL,
        ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL,
        ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL,

    } ZLAC_TERMINAL_FUNCTION;

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
    class zlac_error {
    public:
        zlac_error()
        {
            this->clear();
        }
        void check(int value)
        {
            this->clear();
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
        }
        void clear()
        {
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
        }
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
    };
    ZLAC::DRIVER_MODE _mode = ZLAC::DRIVER_MODE::NOT_INITIALIZED;

private:
    MessageFrame _frame;
    unsigned int _address                        = MODBUS_TARGET_ADDRESS;
    const MessageFrame::MODBUS_TYPE _modbus_type = ZLAC_MODBUS_TYPE;

private:
    MessageFrame _modbus_send_0x03(unsigned long index, int size)
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
        log_v("ADR[0x%0X] Fun[0x%0X] Len[%d] CRC[%04X] Data[%02X %02X %02X %02X %02X %02X %02X %02X]",
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
              this->_frame.data[7]);
        return this->send_frame(this->_frame);
    }
#if 0
    MessageFrame _modbus_send_0x03(unsigned int *data, int len)
    {
        this->_frame.make_frame( //
                this->_modbus_type,
                this->_address,
                MessageFrame::FUNCTION_READ_HOLDING_REGISTERS,
                data,
                len);
        return this->send_frame(this->_frame);
    }
#endif
    MessageFrame _modbus_send_0x06(unsigned int *data, int len)
    {
        this->_frame.make_frame( //
                this->_modbus_type,
                this->_address,
                MessageFrame::FUNCTION_WRITE_SINGLE_REGISTER,
                data,
                len);
        log_v("ADR[0x%0X] Fun[0x%0X] Len[%d] CRC[%04X] Data[%02X %02X %02X %02X %02X %02X %02X %02X]",
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
              this->_frame.data[7]);
        return this->send_frame(this->_frame);
    }
    MessageFrame _modbus_send_0x10(unsigned int *data, int len)
    {
        this->_frame.make_frame( //
                this->_modbus_type,
                this->_address,
                MessageFrame::FUNCTION_WRITE_MULTIPLE_REGISTERS,
                data,
                len);
        log_v("ADR[0x%0X] Fun[0x%0X] Len[%d] CRC[%04X] Data[%02X %02X %02X %02X %02X %02X %02X %02X]",
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
              this->_frame.data[7]);
        return this->send_frame(this->_frame);
    }

public:
    ////////////////
    // Common constant for Left and Right motors
    ////////////////
    bool get_communication_offline_time(int *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2000u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_communication_offline_time(int value_ms, bool check = false)
    {
        bool result = false;
        if (0 <= value_ms && value_ms <= 32767) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<unsigned int> arr = { //
                                              0x20u,
                                              0x00u,
                                              value_ms >> 8 & 0xFFu,
                                              value_ms & 0xFFu
            };

            MessageFrame _frame = this->_modbus_send_0x06(arr.data(), arr.size());
            if (true == check) {
                int buf = 0;
                result  = get_communication_offline_time(&buf);
                if (true == result) {
                    if (buf != value_ms) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    bool get_rs485_node_id(int *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2001u, 1);
        if (0x80 <= _frame.function) {
            *value = -1;
            result = false;
        } else {
            *value = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_rs485_node_id(int id, bool check = false)
    {
        bool result = false;
        if (0 <= id && id <= 127) {
            result = true;
        } else {
            log_w("Out of range");
        }
        if (true == result) {
            std::vector<unsigned int> arr = { //
                                              0x20u,
                                              0x01u,
                                              id >> 8 & 0xFFu,
                                              id & 0xFFu
            };

            MessageFrame _frame = this->_modbus_send_0x06(arr.data(), arr.size());
            if (true == check) {
                int buf = 0;
                result  = get_rs485_node_id(&buf);
                if (true == result) {
                    if (buf != id) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    bool get_rs485_baud_rate(RS485_BAUD_RATE *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2002u, 1);
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
            std::vector<unsigned int> arr = { //
                                              0x20u,
                                              0x02u,
                                              input >> 8 & 0xFFu,
                                              input & 0xFFu
            };

            MessageFrame _frame = this->_modbus_send_0x06(arr.data(), arr.size());
            if (true == check) {
                RS485_BAUD_RATE buf = RS485_BAUD_RATE::RS485_BAUD_RATE_INVALID;
                result              = get_rs485_baud_rate(&buf);
                if (true == result) {
                    if (buf != baud) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    bool get_input_signal_status(int &x0, int &x1)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2003u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int value = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            x0                 = (value >> 0) & 0x01;
            x1                 = (value >> 1) & 0x01;
        }
        return result;
    }
    bool get_out_signal_status(int &x0, int &x1)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2004u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int value = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            x0                 = (value >> 0) & 0x01;
            x1                 = (value >> 1) & 0x01;
        }
        return result;
    }

    bool get_clear_feedback_position(ZLAC::target_motor *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2005u, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = ZLAC::target_motor::TARGET_MOTOR_INVALID;
        } else {
            int buf = _frame.data[0];
            switch (buf) {
                case 1:
                    *value = ZLAC::target_motor::TARGET_MOTOR_LEFT;
                    break;
                case 2:
                    *value = ZLAC::target_motor::TARGET_MOTOR_RIGHT;
                    break;
                case 3:
                    *value = ZLAC::target_motor::TARGET_MOTOR_ALL;
                    break;
                case 0:
                default:
                    *value = ZLAC::target_motor::TARGET_MOTOR_INVALID;
                    break;
            }
        }
        return result;
    }
    bool set_clear_feedback_position(ZLAC::target_motor target, bool check = false)
    {
        bool result = false;
        if (true == result) {
            unsigned int input            = (unsigned int)target;
            std::vector<unsigned int> arr = { //
                                              0x20u,
                                              0x05u,
                                              input >> 8 & 0xFFu,
                                              input & 0xFFu
            };

            MessageFrame _frame = this->_modbus_send_0x06(arr.data(), arr.size());
            if (true == check) {
                ZLAC::target_motor buf = ZLAC::target_motor::TARGET_MOTOR_INVALID;
                result                 = get_clear_feedback_position(&buf);
                if (true == result) {
                    if (buf != target) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }

    bool get_reset_the_zero_point_in_absolute_position_control(ZLAC::target_motor *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2006u, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = ZLAC::target_motor::TARGET_MOTOR_INVALID;
        } else {
            int buf = _frame.data[0];
            switch (buf) {
                case 1:
                    *value = ZLAC::target_motor::TARGET_MOTOR_LEFT;
                    break;
                case 2:
                    *value = ZLAC::target_motor::TARGET_MOTOR_RIGHT;
                    break;
                case 3:
                    *value = ZLAC::target_motor::TARGET_MOTOR_ALL;
                    break;
                case 0:
                default:
                    result = false;
                    *value = ZLAC::target_motor::TARGET_MOTOR_INVALID;
                    break;
            }
        }
        return result;
    }
    bool set_reset_the_zero_point_in_absolute_position_control(ZLAC::target_motor target, bool check = false)
    {
        bool result = false;
        if (true == result) {
            unsigned int input            = (unsigned int)target;
            std::vector<unsigned int> arr = { //
                                              0x20u,
                                              0x06u,
                                              input >> 8 & 0xFFu,
                                              input & 0xFFu
            };

            MessageFrame _frame = this->_modbus_send_0x06(arr.data(), arr.size());
            if (true == check) {
                ZLAC::target_motor buf = ZLAC::target_motor::TARGET_MOTOR_INVALID;
                result                 = get_reset_the_zero_point_in_absolute_position_control(&buf);
                if (true == result) {
                    if (buf != target) {
                        result = false;
                    }
                }
            }
        }
        return result;
    }
    bool get_shaft_state_after_power_on(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x2007u, 1);
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
    bool set_shaft_state_after_power_on(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_maximum_motor_speed(int *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2008u, 1);
        if (0x80 <= _frame.function) {
            *value = -1;
            result = false;
        } else {
            *value = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    bool set_maximum_motor_speed(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_register_parameter_settings(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x2009u, 1);
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
    bool set_register_parameter_settings(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_can_node_info(int *id, CAN_BAUD_RATE *baud)
    {
        bool result         = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x200Au, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *id                = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            unsigned int value = (int)(_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
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

    bool set_can_node_info(int id, CAN_BAUD_RATE baud, bool check = false)
    {
        // TODO
        return true;
    }

    bool get_control_mode(ZLAC::DRIVER_MODE *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x200Du, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = ZLAC::DRIVER_MODE::NOT_INITIALIZED;
        } else {
            int buf = _frame.data[0];
            switch (buf) {
                case 1:
                    *value = ZLAC::DRIVER_MODE::POSITION_RELATIVE;
                    break;
                case 2:
                    *value = ZLAC::DRIVER_MODE::POSITION_ABSOLUTE;
                    break;
                case 3:
                    *value = ZLAC::DRIVER_MODE::VELOCITY;
                    break;
                case 4:
                    *value = ZLAC::DRIVER_MODE::TORQUE;
                    break;
                case 0:
                default:
                    *value = ZLAC::DRIVER_MODE::NOT_INITIALIZED;
                    break;
            }
        }
        return result;
    }
    bool set_control_mode(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_control_word(ZLAC_CONTROL_WORD *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x200Eu, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED;
        } else {
            int buf = _frame.data[0];
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
    bool set_control_word(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_synchronous_onous_control_status(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x200Fu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int buf = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0x00 == buf) {
                *value = true;
            }
        }
        return result;
    }
    bool set_synchronous_onous_control_status(bool check = false)
    {
        // TODO
        return true;
    }
    bool store_rw_register_to_eperm()
    {
        bool result         = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x2010u, 1);
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
    bool get_quick_stop_control(ZLAC_STOP_CONTROL *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2011u, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
        } else {
            int buf = _frame.data[0];
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
    bool set_quick_stop_control(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_close_operation_control(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x2012u, 1);
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
    bool set_close_operation_control(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_disable_control(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x2013u, 1);
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
    bool set_disable_control(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_halt_control(ZLAC_STOP_CONTROL *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2014u, 1);
        if (0x80 <= _frame.function) {
            result = false;
            *value = ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED;
        } else {
            int buf = _frame.data[0];
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
    bool set_halt_control(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_input_effective_level(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x2016u, 1);
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
    bool set_input_effective_level(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_input_terminal_terminal_function_selection(int *x0, int *x1)
    {
        bool result         = true;
        *x0                 = false;
        *x1                 = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x201Fu, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            unsigned int value0 = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            unsigned int value1 = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            if (9 == value0) {
                *x0 = true;
            }
            if (9 == value1) {
                *x1 = true;
            }
        }
        return result;
    }
    bool set_input_terminal_terminal_function_selection(int x0, int x1, bool check = false)
    {
        // TODO
        return true;
    }

    bool get_output_effective_high_level(bool *y0, bool *y1, bool *b0, bool *b1)
    {
        bool result         = true;
        *y0                 = false;
        *y1                 = false;
        *b0                 = false;
        *b1                 = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x2F19u, 2);
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
    bool set_output_effective_level(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_output_terminal_close_brake( //
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
        MessageFrame _frame = this->_modbus_send_0x03(0x201Au, 4);
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
    bool set_output_terminal_terminal_function_selection(int b0, int b1, int y0, int y1, bool check = false)
    {
        // TODO
        return true;
    }

    bool get_driver_temperature_protection_threshold(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_0x03(0x201Eu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
        }
        return result;
    }
    bool set_driver_temperature_protection_threshold(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_alarm_pwm_processing_method(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x201Fu, 1);
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
    bool set_alarm_pwm_processing_method(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_overload_processing_method(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x2020u, 1);
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
    bool set_overload_processing_method(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_io_emergency_stop_processing_mode(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x2021u, 1);
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
    bool set_io_emergency_stop_processing_mode(bool check = false)
    {
        // TODO
        return true;
    }

    ////////////////
    // Left motor parameter
    ////////////////
    bool get_encoder_line_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2030u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_encoder_line_left(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_hall_offset_angle_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2031u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_hall_offset_angle_left(bool check = false)
    {
        // TODO
        return true;
    }
    int get_overload_factor_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2032u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_overload_factor_left(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_rated_current_left(double *rated, double *maximum)
    {
        bool result         = true;
        *rated              = 0.0;
        *maximum            = 0.0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2033u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *rated   = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 0.1;
            *maximum = (double)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF)) / 0.1;
        }
        return result;
    }
    bool set_rated_current_left(bool check = false)
    {
        // TODO
        return true;
    }
    bool set_maximum_current_left(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_overload_protection_time_left(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2035u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 100.0;
        }
        return result;
    }
    bool set_overload_protection_time_left(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_position_following_error_threshold_left(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2036u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
        }
        return result;
    }
    bool set_position_following_error_threshold_left(bool check = false)
    {
        // TODO
        return true;
    }
    int get_velocity_smoothing_factor_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2037u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_velocity_smoothing_factor_left(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_cl_left(int *kp, int *ki)
    {
        bool result         = true;
        *kp                 = 0;
        *ki                 = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2038u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *ki = (int)(_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    bool set_cl_left(int kp, int ki, bool check = false)
    {
        // TODO
        return true;
    }
    bool get_feedforward_output_smoothing_factor_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x203Au, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_feedforward_output_smoothing_factor_left(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_torque_output_smoothing_factor_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x203Bu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_torque_output_smoothing_factor_left(bool check = false)
    {
        // TODO
        return true;
    }
    int get_velocity_loop_left(int *kp, int *ki, int *kf)
    {
        bool result         = true;
        *kp                 = 0;
        *ki                 = 0;
        *kf                 = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x203Cu, 3);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *ki = (int)(_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            *kf = (int)(_frame.data[4] << 8) | (_frame.data[5] & 0xFF);
        }
        return result;
    }
    bool set_velocity_loop_left(int kp, int ki, int kf, bool check = false)
    {
        // TODO
        return true;
    }
    int get_position_loop_left(int *kp, int *kf)
    {
        bool result         = true;
        *kp                 = 0;
        *kf                 = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x203Fu, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *kf = (int)(_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    bool set_position_loop_kp_left(bool check = false)
    {
        // TODO
        return true;
    }

    bool get_initial_velocity_left(int *value)
    {
        bool result = true;
        *value      = 0;
        int address = 0x00;
        switch (this->_mode) {
            case ZLAC::DRIVER_MODE::VELOCITY:
                address = 0x2043u;
                break;
            case ZLAC::DRIVER_MODE::POSITION_RELATIVE:
            case ZLAC::DRIVER_MODE::POSITION_ABSOLUTE:
                address = 0x2044u;
                break;
            default:
                break;
        }
        if (0 != address) {
            MessageFrame _frame = this->_modbus_send_0x03(address, 1);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            }
        }
        return result;
    }
    bool set_initial_velocity_left(bool check = false)
    {
        // TODO
        return true;
    }

    bool get_motor_poles_left(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2045u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_motor_poles_left(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_over_temperature_threshold_left(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2046u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
        }
        return result;
    }
    bool set_over_temperature_threshold_left(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_velocity_observer_coefficient_left(int *index1, int *index2, int *index3, int *index4)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2047u, 4);
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
    bool set_velocity_observer_coefficient_left(bool check = false)
    {
        // TODO
        return true;
    }

    ////////////////
    // Right motor parameter
    ////////////////

    bool get_encoder_line_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2060u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_encoder_line_right(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_hall_offset_angle_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2061u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_hall_offset_angle_right(bool check = false)
    {
        // TODO
        return true;
    }
    int get_overload_factor_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2062u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_overload_factor_right(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_rated_current_right(double *rated, double *maximum)
    {
        bool result         = true;
        *rated              = 0.0;
        *maximum            = 0.0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2063u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *rated   = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 0.1;
            *maximum = (double)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF)) / 0.1;
        }
        return result;
    }
    bool set_rated_current_right(bool check = false)
    {
        // TODO
        return true;
    }
    bool set_maximum_current_right(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_overload_protection_time_right(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2065u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 100.0;
        }
        return result;
    }
    bool set_overload_protection_time_right(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_position_following_error_threshold_right(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2066u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
        }
        return result;
    }
    bool set_position_following_error_threshold_right(bool check = false)
    {
        // TODO
        return true;
    }
    int get_velocity_smoothing_factor_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2067u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_velocity_smoothing_factor_right(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_cl_right(int *kp, int *ki)
    {
        bool result         = true;
        *kp                 = 0;
        *ki                 = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2068u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *ki = (int)(_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    bool set_cl_right(int kp, int ki, bool check = false)
    {
        // TODO
        return true;
    }
    bool get_feedforward_output_smoothing_factor_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x206Au, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_feedforward_output_smoothing_factor_right(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_torque_output_smoothing_factor_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x206Bu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_torque_output_smoothing_factor_right(bool check = false)
    {
        // TODO
        return true;
    }
    int get_velocity_loop_right(int *kp, int *ki, int *kf)
    {
        bool result         = true;
        *kp                 = 0;
        *ki                 = 0;
        *kf                 = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x206Cu, 3);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *ki = (int)(_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            *kf = (int)(_frame.data[4] << 8) | (_frame.data[5] & 0xFF);
        }
        return result;
    }
    bool set_velocity_loop_right(int kp, int ki, int kf, bool check = false)
    {
        // TODO
        return true;
    }
    int get_position_loop_right(int *kp, int *kf)
    {
        bool result         = true;
        *kp                 = 0;
        *kf                 = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x206Fu, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *kp = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *kf = (int)(_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    bool set_position_loop_kp_right(bool check = false)
    {
        // TODO
        return true;
    }

    bool get_initial_velocity_right(int *value)
    {
        bool result = true;
        *value      = 0;
        int address = 0x00;
        switch (this->_mode) {
            case ZLAC::DRIVER_MODE::VELOCITY:
                address = 0x2043u;
                break;
            case ZLAC::DRIVER_MODE::POSITION_RELATIVE:
            case ZLAC::DRIVER_MODE::POSITION_ABSOLUTE:
                address = 0x2044u;
                break;
            default:
                break;
        }
        if (0 != address) {
            MessageFrame _frame = this->_modbus_send_0x03(address, 1);
            if (0x80 <= _frame.function) {
                result = false;
            } else {
                *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            }
        }
        return result;
    }
    bool set_initial_velocity_right(bool check = false)
    {
        // TODO
        return true;
    }

    bool get_motor_poles_right(int *value)
    {
        bool result         = true;
        *value              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2075u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
        }
        return result;
    }
    bool set_motor_poles_right(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_over_temperature_threshold_right(double *value)
    {
        bool result         = true;
        *value              = 0.0;
        MessageFrame _frame = this->_modbus_send_0x03(0x2076u, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 10.0;
        }
        return result;
    }
    bool set_over_temperature_threshold_right(bool check = false)
    {
        // TODO
        return true;
    }
    bool get_velocity_observer_coefficient_right(int *index1, int *index2, int *index3, int *index4)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2077u, 4);
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
    bool set_velocity_observer_coefficient_right(bool check = false)
    {
        // TODO
        return true;
    }
    ////////////////
    // Control parameter
    ////////////////
    bool get_s_shape_acceleration_time(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2080u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    bool set_s_shape_acceleration_time(ZLAC::TARGET_MOTOR target, bool check = false)
    {
        // TODO
        return true;
    }

    bool get_s_shape_deceleration_time(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2082u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    bool set_s_shape_deceleration_time(ZLAC::TARGET_MOTOR target, bool check = false)
    {
        // TODO
        return true;
    }

    bool get_deceleration_time_of_quick_stop(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2084u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            return true;
        }
        return result;
    }
    bool set_deceleration_time_of_quick_stop(ZLAC::TARGET_MOTOR target, bool check = false)
    {
        // TODO
        return true;
    }

    bool get_torque_slope(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2086u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            return true;
        }
        return result;
    }
    bool set_torque_slope(ZLAC::TARGET_MOTOR target, bool check = false)
    {
        // TODO
        return 0;
    }

    bool get_target_velocity(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2088u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    bool set_target_velocity(ZLAC::TARGET_MOTOR target, bool check = false)
    {
        // TODO
        return true;
    }

    bool get_target_position(long *left, long *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x208Au, 4);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 24) | (_frame.data[1] << 16) | (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
            *right = (_frame.data[4] << 24) | (_frame.data[5] << 16) | (_frame.data[6] << 8) | (_frame.data[7] & 0xFF);
        }
        return result;
    }
    bool set_target_position(ZLAC::TARGET_MOTOR target, bool check = false)
    {
        // TODO
        return true;
    }

    bool get_max_speed(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x208Eu, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    bool set_max_speed(ZLAC::TARGET_MOTOR target, bool check = false)
    {
        // TODO
        return true;
    }

    bool get_target_torque(int *left, int *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x2090u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            *left  = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            *right = (_frame.data[2] << 8) | (_frame.data[3] & 0xFF);
        }
        return result;
    }
    bool set_target_torque(ZLAC::TARGET_MOTOR target, bool check = false)
    {
        // TODO
        return true;
    }
    ////////////////
    // Read only parameter
    ////////////////
    bool get_software_version(int *version)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x20A0u, 1);
        if (0x80 <= _frame.function) {
            *version = -1;
            result   = false;
        } else {
            *version = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
        }
        return result;
    }
    bool get_bus_voltage(double *value)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x20A1u, 1);
        if (0x80 <= _frame.function) {
            *value = NAN;
            result = false;
        } else {
            *value = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF)) / 100.0;
        }
        return result;
    }
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
        MessageFrame _frame = this->_modbus_send_0x03(0x20A2u, 1);
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
    bool get_hall_input_state(bool *left_hall_err, bool *right_hall_err)
    {
        bool result         = true;
        *left_hall_err      = false;
        *right_hall_err     = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x20A3u, 1);
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
    bool get_motor_temperature(int *left, int *right)
    {
        bool result         = true;
        *left               = 0;
        *right              = 0;
        MessageFrame _frame = this->_modbus_send_0x03(0x20A4u, 1);
        if (0x80 <= _frame.function) {
            *left  = -1;
            *right = -1;
            result = false;
        } else {
            *left  = (int)_frame.data[0];
            *right = (int)_frame.data[1];
            if (0x80 <= *left) {
                *left = (0x7F & *left) * -1;
            }
            if (0x80 <= *right) {
                *right = (0x7F & *right) * -1;
            }
        }
        return result;
    }
    bool get_error_code(zlac_error *left, zlac_error *right)
    {
        bool result = true;
        left->clear();
        right->clear();
        MessageFrame _frame = this->_modbus_send_0x03(0x20A5u, 2);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            left->check((int)(_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            left->check((int)(_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
        }
        return result;
    }

    bool get_actual_motor_position(long *left, long *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x20A7u, 4);
        if (0x80 <= _frame.function) {
            *left  = -1;
            *right = -1;
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

    bool get_actual_velocity(double *left, double *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x20ABu, 2);
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

    bool get_actual_torque(double *left, double *right)
    {
        bool result         = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x20ADu, 2);
        if (0x80 <= _frame.function) {
            *left  = NAN;
            *right = NAN;
            result = false;
        } else {
            *left  = (double)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
            *right = (double)((_frame.data[2] << 8) | (_frame.data[3] & 0xFF));
            *left  = *left / 10.0;
            *right = *right / 10.0;
        }
        return result;
    }

    bool get_software_connected_status(bool *value)
    {
        bool result         = true;
        *value              = false;
        MessageFrame _frame = this->_modbus_send_0x03(0x20AFu, 1);
        if (0x80 <= _frame.function) {
            result = false;
        } else {
            int buf = (int)((_frame.data[0] << 8) | (_frame.data[1] & 0xFF));
#if 1
            *value = true;
#else
            if (1 == buf) {
                *value = true;
            }
#endif
        }
        return result;
    }
    bool get_driver_temperature(double *value)
    {
        double result       = true;
        MessageFrame _frame = this->_modbus_send_0x03(0x20B0u, 2);
        if (0x80 <= _frame.function) {
            *value = NAN;
            result = false;
        } else {
            unsigned int buffer = (_frame.data[0] << 8) | (_frame.data[1] & 0xFF);
            if (0xF0000 <= buffer) {
                *value = (double)(0x7FFF & buffer) / -10.0;
            } else {
                *value = (double)buffer / 10.0;
            }
        }
        return result;
    }
};

class ZLAC8015DModbus : public ZlacDriver, ZLAC8015DCtrl {
public:
    ZLAC8015DModbus()
    {
        this->info.left.interval  = SETTING_INTERVAL_LEFT;
        this->info.right.interval = SETTING_INTERVAL_RIGHT;
    }
    ~ZLAC8015DModbus()
    {
        if (nullptr != this->_serial_driver_1) {
            this->_serial_driver_1->end();
        }
        if (nullptr != this->_serial_driver_2) {
            this->_serial_driver_2->end();
        }
    }
#if 0
    bool _call_exception(MessageFrame &frame) override
    {
        bool result = true;
        log_i("_call_exception");
        frame = frame;

        return true;
    }
    bool _call_read_holding_registers(MessageFrame &frame) override
    {
        bool result = true;
        log_i("_call_read_holding_registers");
        frame = frame;

        return true;
    }
    bool _call_write_single_register(MessageFrame &frame) override
    {
        bool result = true;
        log_i("_call_write_single_register");
        frame = frame;

        return true;
    }
    bool _call_write_multiple_registers(MessageFrame &frame) override
    {
        bool result = true;
        log_i("_call_write_multiple_registers");
        frame = frame;

        return true;
    }
    bool _call_diagnostics(MessageFrame &frame) override
    {
        bool result = true;
        log_i("_call_diagnostics");
        frame = frame;

        return true;
    }
#endif
    bool setup(HardwareSerial *null_serial, HardwareSerial *serial, unsigned long baud = 57600)
    {
        bool result = false;
        try {
#if SETTING_MOTOR_ENABLE_LEFT
            if (nullptr != serial) {
                this->_serial_driver_1 = serial;
                this->_serial_driver_1->setTimeout(this->TIMEOUT_DRIVER_MS);
                this->_serial_driver_1->setRxBufferSize(this->RX_BUFFER_SIZE);
                this->_serial_driver_1->setTxBufferSize(this->TX_BUFFER_SIZE);
                this->_serial_driver_1->begin(baud, SERIAL_8N1, -1, -1, this->_flag_invert, this->TIMEOUT_DRIVER_MS);
                result = true;
            }
#endif
        } catch (...) {
        }
        return result;
    }

public:
    bool begin() override
    {
        bool result = true;
        log_v("%s", __func__);
        delay(100);
        this->_clear_receive(ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL);
        this->info.system.set(LOG_BEGIN, 0, 0);

        return result;
    }
    bool loop() override
    {
        bool result = true;
        log_v("%s", __func__);
        return result;
    }

public:
    bool cmd_setting_proportional_gain(ZLAC::TARGET_MOTOR target, int value) override
    {
        log_v("%s", __func__);
        char a1 = 0x00;
        switch (this->info.mode) {
            case DRIVER_MODE::POSITION_FROM_PULSE:
            case DRIVER_MODE::POSITION_FROM_DIGITAL:
            case DRIVER_MODE::POSITION_FROM_ANALOG:
                a1 = 0x1A;
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::SPEED_FROM_DIGITAL:
            case DRIVER_MODE::SPEED_FROM_ANALOG:
                a1 = 0x40;
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::TORQUE_FROM_DIGITAL:
            case DRIVER_MODE::TORQUE_FROM_ANALOG:
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::NOT_INITIALIZED:
            default:
                break;
        }
        bool result      = false;
        char buffer[100] = { 0 };
        if (0x00 != a1) {
            result = this->_send_target(__func__, target, a1, value >> 8, value & 0xFF, true);
        }
        if (true == result) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                if (0x40 != a1) {
                    this->info.left.position_proportional_gain = value;
                } else {
                    this->info.left.speed_proportional_gain = value;
                }
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                if (0x40 != a1) {
                    this->info.right.position_proportional_gain = value;
                } else {
                    this->info.right.speed_proportional_gain = value;
                }
            }
        }
        return result;
    }

    bool cmd_setting_integral_gain(ZLAC::TARGET_MOTOR target, int value)
    {
        log_v("%s", __func__);
        char a1 = 0x00;
        switch (this->info.mode) {
            case DRIVER_MODE::POSITION_FROM_PULSE:
            case DRIVER_MODE::POSITION_FROM_DIGITAL:
            case DRIVER_MODE::POSITION_FROM_ANALOG:
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::SPEED_FROM_DIGITAL:
            case DRIVER_MODE::SPEED_FROM_ANALOG:
                a1 = 0x41;
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::TORQUE_FROM_DIGITAL:
            case DRIVER_MODE::TORQUE_FROM_ANALOG:
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::NOT_INITIALIZED:
            default:
                break;
        }
        bool result      = false;
        char buffer[100] = { 0 };
        if (0x00 != a1) {
            result = this->_send_target(__func__, target, a1, value >> 8, value & 0xFF, true);
        }
        if (true == result) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->info.left.speed_integral_gain = value;
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->info.right.speed_integral_gain = value;
            }
        }
        return result;
    }

    bool cmd_setting_differential_gain(ZLAC::TARGET_MOTOR target, int value)
    {
        log_v("%s", __func__);
        bool result      = false;
        char buffer[100] = { 0 };
        char a1          = 0x00;
        switch (this->info.mode) {
            case DRIVER_MODE::POSITION_FROM_PULSE:
            case DRIVER_MODE::POSITION_FROM_DIGITAL:
            case DRIVER_MODE::POSITION_FROM_ANALOG:
                a1 = 0x1B;
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::SPEED_FROM_DIGITAL:
            case DRIVER_MODE::SPEED_FROM_ANALOG:
                a1 = 0x42;
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::TORQUE_FROM_DIGITAL:
            case DRIVER_MODE::TORQUE_FROM_ANALOG:
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::NOT_INITIALIZED:
            default:
                break;
        }
        if (0x00 != a1) {
            result = this->_send_target(__func__, target, a1, value >> 8, value & 0xFF, true);
        }
        if (true == result) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                if (0x42 != a1) {
                    this->info.left.position_differential_gain = value;
                } else {
                    this->info.left.speed_differential_gain = value;
                }
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                if (0x42 != a1) {
                    this->info.right.position_differential_gain = value;
                } else {
                    this->info.right.speed_differential_gain = value;
                }
            }
        }
        return result;
    }

    bool cmd_setting_feed_forward_gain(ZLAC::TARGET_MOTOR target, int value)
    {
        log_v("%s", __func__);
        bool result      = false;
        char buffer[100] = { 0 };
        char a1          = 0x00;
        switch (this->info.mode) {
            case DRIVER_MODE::POSITION_FROM_PULSE:
            case DRIVER_MODE::POSITION_FROM_DIGITAL:
            case DRIVER_MODE::POSITION_FROM_ANALOG:
                a1 = 0x1C;
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::SPEED_FROM_DIGITAL:
            case DRIVER_MODE::SPEED_FROM_ANALOG:
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::TORQUE_FROM_DIGITAL:
            case DRIVER_MODE::TORQUE_FROM_ANALOG:
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::NOT_INITIALIZED:
            default:
                break;
        }
        if (0x00 != a1) {
            result = this->_send_target(__func__, target, a1, value >> 8, value & 0xFF, true);
        }
        if (true == result) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->info.left.position_feed_forward_gain = value;
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->info.right.position_feed_forward_gain = value;
            }
        }
        return result;
    }

    bool cmd_setting_inverted(ZLAC::TARGET_MOTOR target, bool value)
    {
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            this->info.left.interval = value;
        }
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            this->info.right.interval = value;
        }
        return true;
    }
    bool cmd_setting_acc(ZLAC::TARGET_MOTOR target, int value)
    {
        if (0 > value) {
            value = 0;
        }
        this->info.acceleration_ms = value;
        this->cmd_speed_set_acc_and_dec(this->info.acceleration_ms, this->info.deceleration_ms, target);

        return true;
    }
    bool cmd_setting_dcc(ZLAC::TARGET_MOTOR target, int value)
    {
        if (0 > value) {
            value = 0;
        }
        this->info.deceleration_ms = value;
        this->cmd_speed_set_acc_and_dec(this->info.acceleration_ms, this->info.deceleration_ms, target);
        return true;
    }
    bool cmd_setting_limit(ZLAC::TARGET_MOTOR target, int value)
    {
        if (0 > value) {
            value = 0;
        }
        this->info.SPEED_LIMIT = value;
        return true;
    }

public:
    bool cmd_modify_the_rated_current(int value_mW, ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        this->info.rated_current_mW = value_mW;
        return this->_send_target(__func__, target, 0x2D, (value_mW >> 8) & 0xFF, (value_mW >> 0) & 0xFF, true);
    }

    bool cmd_looking_for_z_signal(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        return this->_send_target(__func__, target, 0x53, 0x00, 0x00, true);
    }
    bool cmd_clear_fault(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        return this->_send_target(__func__, target, 0x4A, 0x00, 0x00, true);
    }

    void cmd_get_all_status(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        static int cnt = 0;
        this->cmd_get_alarm_status();
        cnt++;
        delay(this->INTERVAL_DRIVER_MS * 3);
        switch (cnt) {
            case 1:
                this->cmd_get_bus_voltage(target);
                this->cmd_get_output_current(target);
                break;
            case 2:
                this->cmd_get_position_given(target);
                this->cmd_get_position_feedback(target);
                break;
            default:
                this->cmd_get_motor_speed(target);
                cnt = 0;
                break;
        }
    }

    bool cmd_get_alarm_status(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        //log_v("%s", __func__);
        char buffer[255] = { 0 };
        int len          = 0;
        bool result_01   = true;
        bool result_02   = true;
        bool flag_output = false;
#ifndef DEBUG_TRACE
        flag_output = true;
#endif
        unsigned long current_time = millis();
        if (true == this->_send_target(__func__, target, 0x60, 0x00, 0x00, true, flag_output)) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_01 = false;
                if (4 <= this->_receive(__func__, this->_serial_driver_1, 4, buffer, flag_output)) {
                    this->info.left.error.not_connection = false;
                    this->info.left.error.stop_state     = ((buffer[3] & 0x01) > 0) ? false : true;
                    this->info.left.error.startup_state  = ((buffer[3] & 0x80) > 0) ? false : true;

                    this->info.left.error.over_current  = ((buffer[3] & 0x02) > 0) ? true : false;
                    this->info.left.error.over_voltage  = ((buffer[3] & 0x04) > 0) ? true : false;
                    this->info.left.error.encoder_error = ((buffer[3] & 0x08) > 0) ? true : false;
                    this->info.left.error.overheat      = ((buffer[3] & 0x10) > 0) ? true : false;
                    this->info.left.error.under_voltage = ((buffer[3] & 0x20) > 0) ? true : false;
                    this->info.left.error.overload      = ((buffer[3] & 0x40) > 0) ? true : false;
                    this->info.left.update_time         = current_time;
                    result_01                           = true;
                } else {
#if SETTING_MOTOR_ENABLE_LEFT
                    this->info.left.error.not_connection = true;
#else
                    this->info.left.error.not_connection  = false;
#endif
                }
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_02 = false;
                if (4 <= this->_receive(__func__, this->_serial_driver_2, 4, buffer, flag_output)) {
                    this->info.right.error.not_connection = false;
                    this->info.right.error.stop_state     = ((buffer[3] & 0x01) > 0) ? false : true;
                    this->info.right.error.startup_state  = ((buffer[3] & 0x80) > 0) ? false : true;

                    this->info.right.error.over_current  = ((buffer[3] & 0x02) > 0) ? true : false;
                    this->info.right.error.over_voltage  = ((buffer[3] & 0x04) > 0) ? true : false;
                    this->info.right.error.encoder_error = ((buffer[3] & 0x08) > 0) ? true : false;
                    this->info.right.error.overheat      = ((buffer[3] & 0x10) > 0) ? true : false;
                    this->info.right.error.under_voltage = ((buffer[3] & 0x20) > 0) ? true : false;
                    this->info.right.error.overload      = ((buffer[3] & 0x40) > 0) ? true : false;
                    this->info.right.update_time         = current_time;
                    result_02                            = true;
                } else {
#if SETTING_MOTOR_ENABLE_RIGHT
                    this->info.right.error.not_connection = true;
#else
                    this->info.right.error.not_connection = false;
#endif
                }
            }
        } else {
#if SETTING_MOTOR_ENABLE_LEFT
            this->info.left.error.not_connection = true;
#endif
#if SETTING_MOTOR_ENABLE_RIGHT
            this->info.right.error.not_connection = true;
#endif
        }
        return result_01 && result_02;
    }
    bool cmd_get_bus_voltage(ZLAC::TARGET_MOTOR target)
    {
        log_v("%s", __func__);
        bool result_01   = true;
        bool result_02   = true;
        char buffer[255] = { 0 };
        if (true == this->_send_target(__func__, target, 0x61, 0x00, 0x00, true)) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_01 = false;
                if (4 <= this->_receive(__func__, this->_serial_driver_1, 4, buffer)) {
                    this->info.left.voltage = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                    result_01               = true;
                }
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_02 = false;
                if (4 <= this->_receive(__func__, this->_serial_driver_2, 4, buffer)) {
                    this->info.right.voltage = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                    result_02                = true;
                }
            }
        }
        return result_01 && result_02;
    }

    bool cmd_get_output_current(ZLAC::TARGET_MOTOR target)
    {
        log_v("%s", __func__);
        bool result_01   = true;
        bool result_02   = true;
        char buffer[255] = { 0 };
        if (true == this->_send_target(__func__, target, 0x62, 0x00, 0x00, true)) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_01 = false;
                if (4 <= this->_receive(__func__, this->_serial_driver_1, 4, buffer)) {
                    this->info.left.current = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                    result_01               = true;
                }
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_02 = false;
                if (4 <= this->_receive(__func__, this->_serial_driver_2, 4, buffer)) {
                    this->info.right.current = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                    result_02                = true;
                }
            }
        }
        return result_01 && result_02;
    }
    bool cmd_get_motor_speed(ZLAC::TARGET_MOTOR target)
    {
        log_v("%s", __func__);

        static unsigned long next_time_l = 0;
        static unsigned long next_time_r = 0;
        unsigned long now_time           = millis();

        bool result_01   = true;
        bool result_02   = true;
        int get_value    = 0;
        char buffer[255] = { 0 };
        if (true == this->_send_target(__func__, target, 0x63, 0x00, 0x00, true)) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                if (now_time < next_time_l) {
                    result_01 = true;
                } else {
                    result_01 = false;
                    if (4 <= this->_receive(__func__, this->_serial_driver_1, 4, buffer)) {
                        this->info.left.speed_enc = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                        this->info.left.speed_rpm = (this->info.left.speed_enc * 3000) / 8192;
                        result_01                 = true;
                        next_time_l               = now_time + this->TIMEOUT_DRIVER_MS;
                    }
                }
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                if (now_time < next_time_r) {
                    result_02 = true;
                } else {
                    result_02 = false;
                    if (4 <= this->_receive(__func__, this->_serial_driver_2, 4, buffer)) {
                        this->info.right.speed_enc = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                        this->info.right.speed_rpm = (this->info.right.speed_enc * 3000) / 8192;
                        result_02                  = true;
                        next_time_r                = now_time + this->TIMEOUT_DRIVER_MS;
                    }
                }
            }
            this->speed_mps_feedback.set(this->rpm_to_mps(this->info.left.speed_rpm), this->rpm_to_mps(this->info.right.speed_rpm), 0, 0);
        }
        return result_01 && result_02;
    }
    bool cmd_get_position_given(ZLAC::TARGET_MOTOR target)
    {
        log_v("%s", __func__);
        bool result_01 = true;
        bool result_02 = true;

        char buffer[255] = { 0 };
        if (true == this->_send_target(__func__, target, 0x64, 0x00, 0x00, true)) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_01 = false;
                if (8 <= this->_receive(__func__, this->_serial_driver_1, 8, buffer)) {
                    this->info.left.position_given = (int32_t)((buffer[1] << 24) | (buffer[2] << 16) | (buffer[5] << 8) | (buffer[6] << 0));
                    result_01                      = true;
                }
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_02 = false;
                if (8 <= this->_receive(__func__, this->_serial_driver_2, 8, buffer)) {
                    this->info.right.position_given = (int32_t)((buffer[1] << 24) | (buffer[2] << 16) | (buffer[5] << 8) | (buffer[6] << 0));
                    result_02                       = true;
                }
            }
        }
        return result_01 && result_02;
    }
    bool cmd_get_position_feedback(ZLAC::TARGET_MOTOR target)
    {
        log_v("%s", __func__);
        bool result_01   = true;
        bool result_02   = true;
        char buffer[255] = { 0 };
        long temp        = 0;
        if (true == this->_send_target(__func__, target, 0x65, 0x00, 0x00, true)) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_01 = false;
                if (8 <= this->_receive(__func__, this->_serial_driver_1, 8, buffer)) {
                    temp                                  = (int32_t)((buffer[1] << 24) | (buffer[2] << 16) | (buffer[5] << 8) | (buffer[6] << 0));
                    this->info.left.position_feedback     = temp * ((true != this->info.left.interval) ? 1 : -1);
                    this->info.left.position_feedback_deg = ((temp % SETTING_SYSTEM_POSITION_AROUND) * 360.0) / SETTING_SYSTEM_POSITION_AROUND;
                    result_01                             = true;
                }
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result_02 = false;
                if (8 <= this->_receive(__func__, this->_serial_driver_2, 8, buffer)) {
                    temp                                   = (int32_t)((buffer[1] << 24) | (buffer[2] << 16) | (buffer[5] << 8) | (buffer[6] << 0));
                    this->info.right.position_feedback     = temp * ((true != this->info.right.interval) ? 1 : -1);
                    this->info.right.position_feedback_deg = ((temp % SETTING_SYSTEM_POSITION_AROUND) * 360.0) / SETTING_SYSTEM_POSITION_AROUND;
                    result_02                              = true;
                }
            }
        }
        return result_01 && result_02;
    }

    //////////////////////////////////////
    bool cmd_mode_selection(DRIVER_MODE mode)
    {
        log_v("%s", __func__);
        bool result      = false;
        char buffer[100] = { 0 };
        switch (mode) {
            case DRIVER_MODE::POSITION_FROM_PULSE:
                result = this->cmd_position_mode_pulse();
                break;
            case DRIVER_MODE::POSITION_FROM_DIGITAL:
                result = this->cmd_position_mode();
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::POSITION_FROM_ANALOG:
                this->info.mode = mode;
                this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::SPEED_FROM_DIGITAL:
                result = this->cmd_speed_mode();
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::SPEED_FROM_ANALOG:
                this->info.mode = mode;
                this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::TORQUE_FROM_DIGITAL:
                result = this->cmd_torque_mode();
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::TORQUE_FROM_ANALOG:
                this->info.mode = mode;
                this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::NOT_INITIALIZED:
            default:
                this->info.mode = mode;
                this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
                result = this->cmd_motor_stop(ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL);
                break;
        }
        return result;
    }
    //////////////////////////////////////

    bool cmd_motor_start(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        bool result = true;
        if (false == this->info.flag.running) {
            result = this->_send_target(__func__, target, 0x00, 0x00, 0x01, true);
            if (true == result) {
                this->info.flag.running = true;
            }
        }

        return result;
    }
    bool cmd_motor_stop(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        bool result = true;
        if (true == this->info.flag.running) {
            result = this->_send_target(__func__, target, 0x00, 0x00, 0x00, true);
            if (true == result) {
                this->info.left.speed_rpm  = 0;
                this->info.right.speed_rpm = 0;
                this->speed_mps_request.set(0, 0, 0, 0);
                this->info.flag.running = false;
            }
        }

        return result;
    }
    //////////////////////////////////////

    bool cmd_position_mode()
    {
        this->_flag_monitoring_speed = false;
        this->info.mode              = DRIVER_MODE::POSITION_FROM_DIGITAL;
        this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
        return this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL, 0x02, 0x00, 0xD0, true);
    }
    bool cmd_position_mode_pulse()
    {
        this->_flag_monitoring_speed = false;
        this->info.mode              = DRIVER_MODE::POSITION_FROM_PULSE;
        this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
        return this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL, 0x02, 0x00, 0xC0, true);
    }
    bool cmd_position_set_absolute()
    {
        log_v("%s", __func__);
        this->info.position_absolute = 1;
        return this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL, 0x51, 0x00, 0x00, true);
    }
    bool cmd_position_set_relative()
    {
        log_v("%s", __func__);
        this->info.position_absolute = 0;
        return this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL, 0x51, 0x00, 0x01, true);
    }
    bool cmd_position_set(long pos_l, int rpm_l, long pos_r, int rpm_r)
    {
        log_v("%s", __func__);
        static int MAX  = 6000;
        static int STEP = 16384;
        bool result_01  = true;
        bool result_02  = true;

        if (this->info.POSITION_LIMIT < rpm_l) {
            rpm_l = this->info.POSITION_LIMIT;
        } else if (rpm_l < -this->info.POSITION_LIMIT) {
            rpm_l = -this->info.POSITION_LIMIT;
        }
        this->info.left.position_rpm = rpm_l;
        rpm_l                        = (rpm_l * STEP) / MAX;
        if (this->info.POSITION_LIMIT < rpm_r) {
            rpm_r = this->info.POSITION_LIMIT;
        } else if (rpm_r < -this->info.POSITION_LIMIT) {
            rpm_r = -this->info.POSITION_LIMIT;
        }
        this->info.right.position_rpm = rpm_r;
        rpm_r                         = (rpm_r * STEP) / MAX;

        this->info.left.position_request  = pos_l;
        this->info.right.position_request = pos_r;

        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0x1D, (rpm_l >> 8) & 0xFF, (rpm_l >> 0) & 0xFF, true);
        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0x1D, (rpm_r >> 8) & 0xFF, (rpm_r >> 0) & 0xFF, true);
        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0x50, (pos_l >> 24) & 0xFF, (pos_l >> 16) & 0xFF, true);
        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0x50, (pos_r >> 24) & 0xFF, (pos_r >> 16) & 0xFF, true);
        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0x05, (pos_l >> 8) & 0xFF, (pos_l >> 0) & 0xFF, false);
        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0x05, (pos_r >> 8) & 0xFF, (pos_r >> 0) & 0xFF, false);

        result_01 = this->_confirm(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0x05);
        result_02 = this->_confirm(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0x05);

        return result_01 && result_02;
    }
    //////////////////////////////////////
    bool cmd_torque_mode()
    {
        this->_flag_monitoring_speed = false;
        this->info.mode              = DRIVER_MODE::TORQUE_FROM_DIGITAL;
        this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
        return this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL, 0x02, 0x00, 0xC1, true);
    }

    bool cmd_torque_set(int value_l_mA, int value_r_mA)
    {
        log_v("%s", __func__);
        bool result_01            = true;
        bool result_02            = true;
        static int start_value    = (600 * 7500) / 24000;
        static int threshold      = (25 * 7500) / 24000;
        static int flag_running_l = false;
        static int flag_running_r = false;

        if (this->info.TORQUE_LIMIT < value_l_mA) {
            value_l_mA = this->info.TORQUE_LIMIT;
        } else if (value_l_mA < -this->info.TORQUE_LIMIT) {
            value_l_mA = -this->info.TORQUE_LIMIT;
        }
        if (this->info.TORQUE_LIMIT < value_r_mA) {
            value_r_mA = this->info.TORQUE_LIMIT;
        } else if (value_r_mA < -this->info.TORQUE_LIMIT) {
            value_r_mA = -this->info.TORQUE_LIMIT;
        }
        this->info.left.torque_request_mA  = value_l_mA;
        this->info.right.torque_request_mA = value_r_mA;
        int value_l                        = (value_l_mA * 7500) / 24000;
        int value_r                        = (value_r_mA * 7500) / 24000;

        if (false == flag_running_l) {
            if (threshold < abs(value_l)) {
                this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0x08, (start_value >> 8) & 0xFF, (start_value >> 0) & 0xFF, false);
                this->cmd_motor_start(ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT);
                delay(this->TIMEOUT_DRIVER_MS);
            }
        }

        if (false == flag_running_r) {
            if (threshold < abs(value_r)) {
                this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0x08, (start_value >> 8) & 0xFF, (start_value >> 0) & 0xFF, false);
                this->cmd_motor_start(ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT);
                delay(this->TIMEOUT_DRIVER_MS);
            }
        }

        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0x08, (value_l >> 8) & 0xFF, (value_l >> 0) & 0xFF, false);
        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0x08, (value_r >> 8) & 0xFF, (value_r >> 0) & 0xFF, false);

        result_01 = this->_confirm(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0x08);
        result_02 = this->_confirm(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0x08);

        if (threshold < abs(value_l)) {
            this->cmd_motor_start(ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT);
            flag_running_l = true;
        } else {
            this->cmd_motor_stop(ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT);
            flag_running_l = false;
        }
        if (threshold < abs(value_r)) {
            this->cmd_motor_start(ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT);
            flag_running_r = true;
        } else {
            this->cmd_motor_stop(ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT);
            flag_running_r = false;
        }

        return result_01 && result_02;
    }
    //////////////////////////////////////
    bool cmd_speed_mode()
    {
        this->_flag_monitoring_speed = true;
        this->info.mode              = DRIVER_MODE::SPEED_FROM_DIGITAL;
        this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
        return this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL, 0x02, 0x00, 0xC4, true);
    }

    bool cmd_speed_set_acc_and_dec(int acceleration_ms, int deceleration_ms, ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        bool result      = false;
        char buffer[100] = { 0 };
        char a1          = 0x00;
        switch (this->info.mode) {
            case DRIVER_MODE::POSITION_FROM_PULSE:
            case DRIVER_MODE::POSITION_FROM_DIGITAL:
            case DRIVER_MODE::POSITION_FROM_ANALOG:
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::SPEED_FROM_DIGITAL:
                a1 = 0x0A;
                break;
            case DRIVER_MODE::SPEED_FROM_ANALOG:
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::TORQUE_FROM_DIGITAL:
            case DRIVER_MODE::TORQUE_FROM_ANALOG:
                break;
                ///////////////////////////////////////////
            case DRIVER_MODE::NOT_INITIALIZED:
            default:
                break;
        }
        if (0x00 != a1) {
            this->info.acceleration_ms = acceleration_ms;
            this->info.deceleration_ms = deceleration_ms;

            result = this->_send_target(__func__, target, a1, acceleration_ms / 100, deceleration_ms / 100, true);
        }
        return result;
    }
    bool cmd_speed_set(int milli_rpm_l, int milli_rpm_r)
    {
        log_v("%s", __func__);
        bool result_01 = true;
        bool result_02 = true;
        if (true == this->info.left.interval) {
            milli_rpm_l = -milli_rpm_l;
        }
        if (true == this->info.right.interval) {
            milli_rpm_r = -milli_rpm_r;
        }

        if ((true == this->info.flag.emergency) || (false == this->info.flag.heart_beat)) {
            milli_rpm_l = 0;
            milli_rpm_r = 0;
        }
        this->info.left.speed_request_rpm  = milli_rpm_l;
        this->info.right.speed_request_rpm = milli_rpm_r;

        if (this->info.SPEED_LIMIT < (milli_rpm_l / 60)) {
            milli_rpm_l = this->info.SPEED_LIMIT * 60;
        } else if ((milli_rpm_l / 60) < -this->info.SPEED_LIMIT) {
            milli_rpm_l = -this->info.SPEED_LIMIT * 60;
        }
        if (this->info.SPEED_LIMIT < (milli_rpm_r / 60)) {
            milli_rpm_r = this->info.SPEED_LIMIT * 60;
        } else if ((milli_rpm_r / 60) < -this->info.SPEED_LIMIT) {
            milli_rpm_r = -this->info.SPEED_LIMIT * 60;
        }
        this->speed_mps_request.set(this->rpm_to_mps(milli_rpm_l / 60), this->rpm_to_mps(milli_rpm_r / 60), 0, 0);

        int enc_per_s_l = (milli_rpm_l * 4096 * 4) / (60 * 1000);
        int enc_per_s_r = (milli_rpm_r * 4096 * 4) / (60 * 1000);

        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0x06, (enc_per_s_l >> 8) & 0xFF, (enc_per_s_l >> 0) & 0xFF, false);
        this->_send_target(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0x06, (enc_per_s_r >> 8) & 0xFF, (enc_per_s_r >> 0) & 0xFF, false);

        result_01 = this->_confirm(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0x06);
        result_02 = this->_confirm(__func__, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0x06);
        return result_01 && result_02;
    }

    bool cmd_speed_heart_beat()
    {
        static unsigned int TIME_LITTLE_HEART_BEAT_MS  = (10);
        static unsigned long next_little_heart_beat_ms = 0;

        static unsigned int TIME_HEART_BEAT_MS  = (500);
        static unsigned long next_heart_beat_ms = 0;
        static unsigned int TIME_GET_STATE_MS   = (1500);
        static unsigned long next_get_state_ms  = 0;
        bool result_01                          = false;
        bool result_02                          = false;
        bool flag_output                        = false;
#ifndef DEBUG_TRACE
        flag_output = true;
#endif
        unsigned long current_time = millis();
        if (false == this->_flag_monitoring_speed) {
            result_01 = true;
            result_02 = true;
            if (next_get_state_ms <= current_time) {
                next_get_state_ms = current_time + TIME_GET_STATE_MS;
                this->cmd_get_all_status();
            }
        } else {
            if (true != this->is_connection()) {
                // do nothing
            } else {
                if (next_little_heart_beat_ms <= current_time) {
                    next_little_heart_beat_ms = current_time + TIME_LITTLE_HEART_BEAT_MS;
                    this->cmd_get_position_feedback(ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL);
                }

                if (next_heart_beat_ms <= current_time) {
                    if (false == this->info.flag.running) {
                        if (next_get_state_ms <= current_time) {
                            next_get_state_ms = current_time + TIME_GET_STATE_MS;
                            this->cmd_get_all_status();
                        }
                    } else {
                        //log_v("%s", __func__);
                        this->_clear_receive();
                        char buffer[4]     = { 0x80, 0x00, 0x80 };
                        char receive[100]  = { 0 };
                        int16_t buf        = 0;
                        long temp          = 0;
                        next_heart_beat_ms = current_time + TIME_HEART_BEAT_MS;

                        if (nullptr != this->_serial_driver_1) {
                            this->_serial_driver_1->write(buffer, 3);
                            this->info.left.order.set(buffer[0], buffer[1], 0);
                        }
                        if (nullptr != this->_serial_driver_2) {
                            this->_serial_driver_2->write(buffer, 3);
                            this->info.right.order.set(buffer[0], buffer[1], 0);
                        }

                        this->_receive_wait(this->_serial_driver_1, 32);

                        if (4 <= this->_receive(__func__, this->_serial_driver_1, 4, receive, flag_output)) {
                            if (28 <= this->_receive(__func__, this->_serial_driver_1, 28, receive, flag_output)) {
                                this->info.left.update_time           = current_time;
                                this->info.left.voltage               = (int16_t)((receive[1] << 8) | (receive[2] << 0));
                                this->info.left.current               = (int16_t)((receive[5] << 8) | (receive[6] << 0));
                                buf                                   = (int16_t)((receive[9] << 8) | (receive[10] << 0));
                                this->info.left.speed_rpm             = (buf * 3000) / 8192;
                                this->info.left.position_given        = (int32_t)((receive[13] << 24) | (receive[14] << 16) | (receive[17] << 8) | (receive[18] << 0));
                                temp                                  = (int32_t)((receive[21] << 24) | (receive[22] << 16) | (receive[25] << 8) | (receive[26] << 0));
                                this->info.left.position_feedback     = temp * ((true != this->info.left.interval) ? 1 : -1);
                                this->info.left.position_feedback_deg = ((temp % SETTING_SYSTEM_POSITION_AROUND) * 360.0) / SETTING_SYSTEM_POSITION_AROUND;
                                result_01                             = true;
                            }
                        }
                        this->_receive_wait(this->_serial_driver_2, 32);

                        if (4 <= this->_receive(__func__, this->_serial_driver_2, 4, receive, flag_output)) {
                            if (28 <= this->_receive(__func__, this->_serial_driver_2, 28, receive, flag_output)) {
                                this->info.right.update_time           = current_time;
                                this->info.right.voltage               = (int16_t)((receive[1] << 8) | (receive[2] << 0));
                                this->info.right.current               = (int16_t)((receive[5] << 8) | (receive[6] << 0));
                                buf                                    = (int16_t)((receive[9] << 8) | (receive[10] << 0));
                                this->info.right.speed_rpm             = (buf * 3000) / 8192;
                                this->info.right.position_given        = (int32_t)((receive[13] << 24) | (receive[14] << 16) | (receive[17] << 8) | (receive[18] << 0));
                                temp                                   = (int32_t)((receive[21] << 24) | (receive[22] << 16) | (receive[25] << 8) | (receive[26] << 0));
                                this->info.right.position_feedback     = temp * ((true != this->info.right.interval) ? 1 : -1);
                                this->info.right.position_feedback_deg = ((temp % SETTING_SYSTEM_POSITION_AROUND) * 360.0) / SETTING_SYSTEM_POSITION_AROUND;
                                result_02                              = true;
                            }
                        }
                        this->speed_mps_feedback.set(this->rpm_to_mps(this->info.left.speed_rpm), this->rpm_to_mps(this->info.right.speed_rpm), 0, 0);
                    }

                    delay(this->INTERVAL_DRIVER_MS);
                }
            }
        }
        return result_01 && result_02;
    }

private:
    bool _confirm(const char *name, ZLAC::TARGET_MOTOR target, char cmd, bool output_log = true)
    {
        bool result      = false;
        char buffer[100] = { 0 };
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            if (2 <= this->_receive(name, this->_serial_driver_1, 2, buffer, output_log)) {
                if (cmd == buffer[0]) {
                    if (cmd == buffer[1]) {
                        result = true;
                    }
                }
            }
        }
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            if (2 <= this->_receive(name, this->_serial_driver_2, 2, buffer, output_log)) {
                if (cmd == buffer[0]) {
                    if (cmd == buffer[1]) {
                        result = true;
                    }
                }
            }
        }
        return result;
    }
    bool _send_target(const char *name, ZLAC::TARGET_MOTOR target, char a1, char a2, char a3, bool confirm, bool output_log = false)
    {
        bool result_01 = true;
        bool result_02 = true;
        bool result    = true;
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            result_01 = this->_send(name, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, this->_serial_driver_1, a1, a2, a3, confirm, output_log);
        }
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            result_02 = this->_send(name, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, this->_serial_driver_2, a1, a2, a3, confirm, output_log);
        }
        return result_01 && result_02;
    }
    bool _send(const char *name, ZLAC::TARGET_MOTOR target, HardwareSerial *serial, char a1, char a2, char a3, bool confirm, bool output_log = false)
    {
        //log_v("%s", __func__);
        unsigned int cs = a1 + a2 + a3;
        bool result     = true;
        char buffer[4]  = { a1, a2, a3, (char)(cs & 0xFF) };
        if (true == output_log) {
#if DEBUG_TRACE
#if DEBUG_ZLAC706_SERIAL
            std::string debug_message = "send    :";
            char buf[100];
            for (int i = 0; i < 4; i++) {
                sprintf(buf, " 0x%02X", buffer[i]);
                debug_message.append(buf);
            }
            log_v("%s", debug_message.c_str());
#endif
#endif
        }
        if (nullptr != serial) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->_clear_receive(ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT);
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->_clear_receive(ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT);
            }
            serial->write(buffer, 4);
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->info.left.order.set(a1, a2, a3);
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->info.right.order.set(a1, a2, a3);
            }
            if (true == confirm) {
                result = this->_confirm(name, target, a1, output_log);
            }
        }
        return result;
    }
    int _receive(const char *name, HardwareSerial *serial, int size, char *buffer, bool output_log = false)
    {
        //log_v("%s", __func__);
        char buf[100];
        int count               = 0;
        bool flag_timeout       = true;
        int index               = 0;
        unsigned long loop_time = 0;
        if (nullptr != serial) {
            loop_time    = millis() + this->TIMEOUT_DRIVER_MS;
            flag_timeout = true;
            index        = 0;
            do {
                count++;
                if (size <= index) {
                    flag_timeout = false;
                    break;
                }
                if (0 != serial->available()) {
                    buffer[index++] = serial->read();
                } else {
                    delay(this->INTERVAL_DRIVER_MS);
                }
            } while (millis() <= loop_time);
        }
        if (true == flag_timeout) {
#if DEBUG_ZLAC
            log_w("Timeout [%d] ms : size [%02d/%02d] :  (%s)", (count - 1) * this->INTERVAL_DRIVER_MS, index, size, name);
#endif
        } else {
            if (true == output_log) {
#if DEBUG_TRACE
#if DEBUG_ZLAC
                std::string debug_message_01 = "receive :";
                sprintf(buf, " count [%d] :", count - 1);
                debug_message_01.append(buf);
                for (int i = 0; i < size; i++) {
                    sprintf(buf, " 0x%02X", buffer[i]);
                    debug_message_01.append(buf);
                }
                log_v("%s", debug_message_01.c_str());
#endif
#endif
            }
        }
        return index;
    }
    bool _receive_wait(HardwareSerial *serial, int size)
    {
        bool result             = false;
        unsigned long loop_time = millis() + this->TIMEOUT_DRIVER_MS;
        do {
            if (size <= serial->available()) {
                result = true;
                break;
            } else {
                delay(this->INTERVAL_DRIVER_MS);
            }
        } while (millis() <= loop_time);
        return result;
    }
    //////////////////////////////////////

    void _clear_receive(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        char buf[100];
#if DEBUG_TRACE
        bool result_01            = false;
        bool result_02            = false;
        std::string debug_message = "";
#else
        size_t size = 0;
#endif
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            if (nullptr != this->_serial_driver_1) {
#if DEBUG_TRACE
                debug_message = "clear data[1] ->";
                while (0 != this->_serial_driver_1->available()) {
                    sprintf(buf, " 0x%02X", this->_serial_driver_1->read());
                    debug_message.append(buf);
                    result_01 = true;
                }
                if (true == result_01) {
                    log_v("%s", debug_message.c_str());
                }
#else
                do {
                    size = this->_serial_driver_1->available();
                    if (0 == size) {
                        break;
                    } else {
                        (void)this->_serial_driver_1->readBytes(buf, size);
                        delay(this->INTERVAL_DRIVER_MS);
                    }
                } while (0 != size);
#endif
                //this->info.system.set(LOG_CLEAR_RECEIVE, TARGET_MOTOR::TARGET_MOTOR_LEFT, 0);
            }
        }
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            if (nullptr != this->_serial_driver_2) {
#if DEBUG_TRACE
                debug_message = "clear data[2] ->";
                while (0 != this->_serial_driver_2->available()) {
                    sprintf(buf, " 0x%02X", this->_serial_driver_2->read());
                    debug_message.append(buf);
                    result_02 = true;
                }
                if (true == result_02) {
                    log_v("%s", debug_message.c_str());
                }
#else
                do {
                    size = this->_serial_driver_2->available();
                    if (0 == size) {
                        break;
                    } else {
                        (void)this->_serial_driver_2->readBytes(buf, size);
                        delay(this->INTERVAL_DRIVER_MS);
                    }
                } while (0 != size);

#endif
                //this->info.system.set(LOG_CLEAR_RECEIVE, TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0);
            }
        }
    }

private:
    HardwareSerial *_serial_driver_1 = nullptr;
    HardwareSerial *_serial_driver_2 = nullptr;
    bool _flag_invert                = SETTING_FLAG_INVERT;

    bool _flag_monitoring_speed = false;

private:
    const unsigned long TIMEOUT_DRIVER_MS  = 50;
    const unsigned long INTERVAL_DRIVER_MS = 1;
    const size_t RX_BUFFER_SIZE            = 512;
    const size_t TX_BUFFER_SIZE            = 512;
};

#endif
