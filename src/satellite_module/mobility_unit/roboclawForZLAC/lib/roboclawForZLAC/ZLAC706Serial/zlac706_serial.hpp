/**
 * @file zlac706_serial.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-05
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ZLAC706_SERIAL_HPP
#define ZLAC706_SERIAL_HPP

#include "../log_callback.hpp"
#include "config_zlac706_serial.hpp"

#include <Arduino.h>

class ZLAC706Serial : public LogCallback {
public:
    typedef enum driver_target
    {
        DRIVER_TARGET_ALL,
        DRIVER_TARGET_LEFT,
        DRIVER_TARGET_RIGHT,
    } DRIVER_TARGET;
    typedef enum system_log
    {
        LOG_BEGIN,         // 0
        LOG_IS_ERROR,      // 1
        LOG_CLEAR_RECEIVE, // 2
        LOG_DISCONNECTED,  // 3
        LOG_CONNECTED,     // 4
        LOG_MODE,          // 5
        LOG_EMERGENCY,     // 6
        LOG_CRC_ERROR,     // 7
        LOG_MAX
    } SYSTEM_LOG;

    typedef enum driver_mode
    {
        NOT_INITIALIZED,
        POSITION_FROM_PULSE,
        POSITION_FROM_DIGITAL,
        POSITION_FROM_ANALOG,
        SPEED_FROM_DIGITAL,
        SPEED_FROM_ANALOG,
        TORQUE_FROM_DIGITAL,
        TORQUE_FROM_ANALOG
    } DRIVER_MODE;

    struct order_info {
    public:
        char a1            = 0;
        char a2            = 0;
        char a3            = 0;
        unsigned long time = 0;

        char v00 = 0;
        char v01 = 0;
        char v02 = 0;
        char v03 = 0;
        char v04 = 0;
        char v05 = 0;
        char v06 = 0;
        char v07 = 0;
        char v08 = 0;
        char v09 = 0;
    };

    template <int LIST_SIZE = 20> class OrderList {
    public:
        OrderList() { this->reset(); }
        void reset() { this->length = 0; }
        void set(char a1,
                 char a2,
                 char a3, //
                 char v00 = 0,
                 char v01 = 0,
                 char v02 = 0,
                 char v03 = 0,
                 char v04 = 0,
                 char v05 = 0,
                 char v06 = 0,
                 char v07 = 0,
                 char v08 = 0,
                 char v09 = 0)
        {
            if (this->_max_size > this->length) {
                this->length++;
            }
            for (int i = (this->length - 1); 0 < i; i--) {
                this->data[i].a1   = this->data[i - 1].a1;
                this->data[i].a2   = this->data[i - 1].a2;
                this->data[i].a3   = this->data[i - 1].a3;
                this->data[i].time = this->data[i - 1].time;

                this->data[i].v00 = this->data[i - 1].v00;
                this->data[i].v01 = this->data[i - 1].v01;
                this->data[i].v02 = this->data[i - 1].v02;
                this->data[i].v03 = this->data[i - 1].v03;
                this->data[i].v04 = this->data[i - 1].v04;
                this->data[i].v05 = this->data[i - 1].v05;
                this->data[i].v06 = this->data[i - 1].v06;
                this->data[i].v07 = this->data[i - 1].v07;
                this->data[i].v08 = this->data[i - 1].v08;
                this->data[i].v09 = this->data[i - 1].v09;
            }
            this->data[0].a1   = a1;
            this->data[0].a2   = a2;
            this->data[0].a3   = a3;
            this->data[0].time = millis();
            this->data[0].v00  = v00;
            this->data[0].v01  = v01;
            this->data[0].v02  = v02;
            this->data[0].v03  = v03;
            this->data[0].v04  = v04;
            this->data[0].v05  = v05;
            this->data[0].v06  = v06;
            this->data[0].v07  = v07;
            this->data[0].v08  = v08;
            this->data[0].v09  = v09;
        }

        order_info data[LIST_SIZE]; /*!< array */
        int length = 0;             /*!< data size */
    private:
        int _max_size = LIST_SIZE; /*!< Maximum number of data */
    };
    struct error_info {
    public:
        bool stop_state     = false;
        bool startup_state  = false;
        bool over_current   = false;
        bool over_voltage   = false;
        bool encoder_error  = false;
        bool overheat       = false;
        bool under_voltage  = false;
        bool overload       = false;
        bool not_connection = false;
    };
    struct motor_info {
    public:
        OrderList<20> order;
        int voltage                 = 0;
        int current                 = 0;
        int speed_rpm               = 0;
        int speed_request_rpm       = 0;
        long position_request       = 0;
        long position_given         = 0;
        long position_feedback      = 0;
        float position_feedback_deg = 0;
        int position_rpm            = 0;
        int torque_request_mA       = 0;
        unsigned long update_time   = 0;
        error_info error;

        unsigned int speed_proportional_gain = SETTING_SPEED_PROPORTIONAL_GAIN;
        unsigned int speed_integral_gain     = SETTING_SPEED_INTEGRAL_GAIN;
        unsigned int speed_differential_gain = SETTING_SPEED_DIFFERENTIAL_GAIN;

        unsigned int position_proportional_gain = SETTING_POSITION_PROPORTIONAL_GAIN;
        unsigned int position_differential_gain = SETTING_POSITION_DIFFERENTIAL_GAIN;
        unsigned int position_feed_forward_gain = SETTING_POSITION_FEED_FORWARD_GAIN;

        unsigned int current_proportional_gain = SETTING_CURRENT_PROPORTIONAL_GAIN;
        unsigned int current_integral_gain     = SETTING_CURRENT_INTEGRAL_GAIN;

        bool interval = false;
    };
    struct control_flag {
    public:
        bool running    = false;
        bool emergency  = false;
        bool heart_beat = false;
    };

    struct zlac_info {
    public:
        OrderList<20> direct;
        OrderList<20> system;
        control_flag flag;
        motor_info left;
        motor_info right;

        DRIVER_MODE mode = DRIVER_MODE::NOT_INITIALIZED;

        int SPEED_LIMIT     = SETTING_SYSTEM_SPEED_LIMIT_RPM;
        int TORQUE_LIMIT    = SETTING_SYSTEM_TORQUE_LIMIT_MA;
        int POSITION_LIMIT  = SETTING_SYSTEM_POSITION_LIMIT_RPM;
        int acceleration_ms = SETTING_SPEED_ACCELERATION_MS;
        int deceleration_ms = SETTING_SPEED_DECELERATION_MS;

        int position_absolute = -1;
        int rated_current_mW  = -1;
    };

public:
    zlac_info info;

public:
    ZLAC706Serial();
    ~ZLAC706Serial();
    bool setup_serial_driver(HardwareSerial *serial_l, HardwareSerial *serial_r, unsigned long baud = 57600);

public:
    bool begin();
    bool loop();

    bool setting_save();
    bool setting_load();

public:
    //////////////////////////////////////
    bool is_error();
    bool is_error_flag();
    bool is_connection();
    DRIVER_MODE get_mode();
    //////////////////////////////////////

    bool cmd_setting_proportional_gain(DRIVER_TARGET target, int value);
    bool cmd_setting_integral_gain(DRIVER_TARGET target, int value);
    bool cmd_setting_differential_gain(DRIVER_TARGET target, int value);
    bool cmd_setting_feed_forward_gain(DRIVER_TARGET target, int value);
    bool cmd_setting_inverted(DRIVER_TARGET target, bool value);
    bool cmd_setting_acc(DRIVER_TARGET target, int value);
    bool cmd_setting_dcc(DRIVER_TARGET target, int value);

public:
    bool cmd_modify_the_rated_current(int value_mW, DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);

    bool cmd_looking_for_z_signal(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);
    bool cmd_clear_fault(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);

    void cmd_get_all_status(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);
    bool cmd_get_alarm_status(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);
    bool cmd_get_bus_voltage(DRIVER_TARGET targe);
    bool cmd_get_output_current(DRIVER_TARGET target);
    bool cmd_get_motor_speed(DRIVER_TARGET target);
    bool cmd_get_position_given(DRIVER_TARGET target);
    bool cmd_get_position_feedback(DRIVER_TARGET target);

    //////////////////////////////////////
    bool cmd_mode_selection(DRIVER_MODE mode);
    //////////////////////////////////////
    bool cmd_motor_start(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);
    bool cmd_motor_stop(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);
    //////////////////////////////////////
    bool cmd_position_mode();
    bool cmd_position_mode_pulse();
    bool cmd_position_set_absolute();
    bool cmd_position_set_relative();
    bool cmd_position_set(long pos_l, int rpm_l, long pos_r, int rpm_r);
    //////////////////////////////////////
    bool cmd_torque_mode();
    bool cmd_torque_set(int value_l_mA, int value_r_mA);
    //////////////////////////////////////
    bool cmd_speed_mode();
    bool cmd_speed_set_acc_and_dec(int acceleration_ms, int deceleration_ms, DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);
    bool cmd_speed_set(int rpm_l, int rpm_r);
    bool cmd_speed_heart_beat();

private:
    const unsigned long TIMEOUT_DRIVER_MS  = 50;
    const unsigned long INTERVAL_DRIVER_MS = 5;
    bool _confirm(const char *name, DRIVER_TARGET target, char cmd, bool output_log = true);
    bool _send_target(const char *name, DRIVER_TARGET target, char a1, char a2, char a3, bool confirm, bool output_log = true);
    bool _send(const char *name, DRIVER_TARGET target, HardwareSerial *serial, char a1, char a2, char a3, bool confirm, bool output_log = true);

    int _receive(const char *name, HardwareSerial *serial, int size, char *buffer, bool output_log = true);
    bool _receive_wait(HardwareSerial *serial, int size);
    //////////////////////////////////////
    void _clear_receive(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);
    int _to_int(String data, int default_value);

private:
    HardwareSerial *_serial_driver_1 = nullptr;
    HardwareSerial *_serial_driver_2 = nullptr;
    bool _flag_invert                = SETTING_FLAG_INVERT;

    bool _flag_monitoring_speed = false;
    bool _flag_error            = false;

private:
    const size_t RX_BUFFER_SIZE = 512;
    const size_t TX_BUFFER_SIZE = 512;
};

#endif
