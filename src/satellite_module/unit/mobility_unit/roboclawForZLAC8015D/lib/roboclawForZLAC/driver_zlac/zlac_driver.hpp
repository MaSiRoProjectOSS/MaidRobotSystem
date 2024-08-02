/**
 * @file zlac_driver.hpp
 * @brief
 * @version 0.23.1
 * @date 2024-07-17
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef ZLAC_DRIVER_HPP
#define ZLAC_DRIVER_HPP

///////////////////////////////////////////////////////////////////
// Define [LOGGER]
///////////////////////////////////////////////////////////////////
#ifndef DEBUG_ZLAC
#define DEBUG_ZLAC (0)
#endif
#ifndef DEBUG_TRACE
#define DEBUG_TRACE (0)
#endif

#include "config_zlac.hpp"

#include <Arduino.h>
#include <SPIFFS.h>
#include <maid_robot_system/common/chart/four_dimensional.hpp>

class ZlacDriver {
public:
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
        LOG_MOTOR_FREE,    // 8
        LOG_UPLOAD_DATA,   // 9
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

    template <int LIST_SIZE = 20>
    class OrderList {
    public:
        OrderList()
        {
            this->reset();
        }
        void reset()
        {
            this->length = 0;
        }
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
        int speed_enc               = 0;
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
        bool motor_free = false;
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
    FourDimensionalChart speed_mps_feedback;
    FourDimensionalChart speed_mps_request;

public:
    ZlacDriver()
    {
    }
    ~ZlacDriver()
    {
    }

public:
    virtual bool begin() = 0;
    virtual bool loop()  = 0;

public:
    virtual bool cmd_setting_proportional_gain(ZLAC::TARGET_MOTOR target, int value) = 0;
    virtual bool cmd_setting_integral_gain(ZLAC::TARGET_MOTOR target, int value)     = 0;
    virtual bool cmd_setting_differential_gain(ZLAC::TARGET_MOTOR target, int value) = 0;
    virtual bool cmd_setting_feed_forward_gain(ZLAC::TARGET_MOTOR target, int value) = 0;
    virtual bool cmd_setting_inverted(ZLAC::TARGET_MOTOR target, bool value)         = 0;
    virtual bool cmd_setting_acc(ZLAC::TARGET_MOTOR target, int value)               = 0;
    virtual bool cmd_setting_dcc(ZLAC::TARGET_MOTOR target, int value)               = 0;
    virtual bool cmd_setting_limit(ZLAC::TARGET_MOTOR target, int value)             = 0;

public:
    virtual bool cmd_modify_the_rated_current(int value_mW, ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL) = 0;

    virtual bool cmd_looking_for_z_signal(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL) = 0;
    virtual bool cmd_clear_fault(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)          = 0;

    virtual void cmd_get_all_status(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)   = 0;
    virtual bool cmd_get_alarm_status(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL) = 0;
    virtual bool cmd_get_bus_voltage(ZLAC::TARGET_MOTOR target)                                         = 0;
    virtual bool cmd_get_output_current(ZLAC::TARGET_MOTOR target)                                      = 0;
    virtual bool cmd_get_motor_speed(ZLAC::TARGET_MOTOR target)                                         = 0;
    virtual bool cmd_get_position_given(ZLAC::TARGET_MOTOR target)                                      = 0;
    virtual bool cmd_get_position_feedback(ZLAC::TARGET_MOTOR target)                                   = 0;

    //////////////////////////////////////
    virtual bool cmd_mode_selection(DRIVER_MODE mode) = 0;
    //////////////////////////////////////
    virtual bool cmd_motor_start(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL) = 0;
    virtual bool cmd_motor_stop(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)  = 0;
    //////////////////////////////////////
    virtual bool cmd_position_mode()                                            = 0;
    virtual bool cmd_position_mode_pulse()                                      = 0;
    virtual bool cmd_position_set_absolute()                                    = 0;
    virtual bool cmd_position_set_relative()                                    = 0;
    virtual bool cmd_position_set(long pos_l, int rpm_l, long pos_r, int rpm_r) = 0;
    //////////////////////////////////////
    virtual bool cmd_torque_mode()                              = 0;
    virtual bool cmd_torque_set(int value_l_mA, int value_r_mA) = 0;
    //////////////////////////////////////
    virtual bool cmd_speed_mode()                                                                                                                      = 0;
    virtual bool cmd_speed_set_acc_and_dec(int acceleration_ms, int deceleration_ms, ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL) = 0;
    virtual bool cmd_speed_set(int milli_rpm_l, int milli_rpm_r)                                                                                       = 0;
    virtual bool cmd_speed_heart_beat()                                                                                                                = 0;

public:
    float rpm_to_mps(int value)
    {
        return ((float)value * SETTING_SYSTEM_WHEEL_DIAMETER_MM_X_PI) / (60.0 * 1000.0);
    }

    float mps_to_rpm(int value)
    {
        return ((float)value * 60.0 * 1000.0) / (SETTING_SYSTEM_WHEEL_DIAMETER_MM_X_PI);
    }
    //////////////////////////////////////

    bool _is_right(ZLAC::TARGET_MOTOR target)
    {
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            return true;
        } else {
            return false;
        }
    }
    bool _is_left(ZLAC::TARGET_MOTOR target)
    {
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            return true;
        } else {
            return false;
        }
    }

    bool is_error()
    {
        bool result_01                       = false;
        bool result_02                       = false;
        static bool flag_previous_01         = false;
        static bool flag_previous_02         = false;
        static unsigned int TIME_INTERVAL_MS = (1000) * 5;
        static unsigned long next_time_ms    = 0;
        if (next_time_ms <= millis()) {
            next_time_ms = millis() + TIME_INTERVAL_MS;
            this->cmd_get_alarm_status();
        }
#if SETTING_MOTOR_ENABLE_LEFT
        // result_01 |= this->info.left.error.stop_state || this->info.left.error.startup_state;
        result_01 |= this->info.left.error.over_current || this->info.left.error.over_voltage || this->info.left.error.under_voltage;
        result_01 |= this->info.left.error.encoder_error || this->info.left.error.overheat || this->info.left.error.overload;
        result_01 |= this->info.left.error.not_connection;
#else
        result_02 |= false;
#endif
#if SETTING_MOTOR_ENABLE_RIGHT
        // result_02 |= this->info.right.error.stop_state || this->info.right.error.startup_state;
        result_02 |= this->info.right.error.over_current || this->info.right.error.over_voltage || this->info.right.error.under_voltage;
        result_02 |= this->info.right.error.encoder_error || this->info.right.error.overheat || this->info.right.error.overload;
        result_02 |= this->info.right.error.not_connection;
#else
        result_02 |= false;
#endif

        if (flag_previous_01 != result_01) {
            this->info.system.set(LOG_IS_ERROR,
                                  ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT,
                                  ((true == result_01) ? 1 : 2),
                                  0,
                                  this->info.left.error.over_current,
                                  this->info.left.error.over_voltage,
                                  this->info.left.error.under_voltage,
                                  this->info.left.error.encoder_error,
                                  this->info.left.error.overheat,
                                  this->info.left.error.overload,
                                  this->info.left.error.not_connection);
        }
        if (flag_previous_02 != result_02) {
            this->info.system.set(LOG_IS_ERROR,
                                  ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT,
                                  ((true == result_02) ? 1 : 2),
                                  0,
                                  this->info.right.error.over_current,
                                  this->info.right.error.over_voltage,
                                  this->info.right.error.under_voltage,
                                  this->info.right.error.encoder_error,
                                  this->info.right.error.overheat,
                                  this->info.right.error.overload,
                                  this->info.right.error.not_connection);
        }
        flag_previous_01 = result_01;
        flag_previous_02 = result_02;

        this->_flag_error = result_01 || result_02;
        return result_01 || result_02;
    }
    bool is_error_flag()
    {
        return this->_flag_error;
    }
    bool is_connection()
    {
        static bool flag_previous = false;
        bool result               = (!this->info.left.error.not_connection) && (!this->info.right.error.not_connection);

        if (flag_previous != result) {
            this->info.system.set(((true == result) ? LOG_CONNECTED : LOG_DISCONNECTED), 0, 0);
        }
        flag_previous = result;
        return result;
    }
    DRIVER_MODE get_mode()
    {
        return this->info.mode;
    }
    //////////////////////////////////////
    bool setting_save()
    {
        bool result = false;
        char buffer[255];
        if (true == SPIFFS.begin(false)) {
            sprintf(buffer,
                    "%s\n%s\n"                 //
                    "%d\n%d\n%d\n%d\n%d\n%d\n" // +6(8)
                    "%d\n%d\n%d\n%d\n%d\n%d\n" // +6(14)
                    "%d\n%d\n%d\n%d\n"         // +4(18)
                    "%d\n%d\n%d\n"             // +3(21)
                    ,                          //

                    this->info.left.interval ? "t" : "f",
                    this->info.right.interval ? "t" : "f",

                    this->info.left.speed_proportional_gain,  // s_skp
                    this->info.right.speed_proportional_gain, // s_skp
                    this->info.left.speed_integral_gain,      // s_ski
                    this->info.right.speed_integral_gain,     // s_ski
                    this->info.left.speed_differential_gain,  // s_skd
                    this->info.right.speed_differential_gain, // s_skd

                    this->info.left.position_proportional_gain,  // s_pkp
                    this->info.right.position_proportional_gain, // s_pkp
                    this->info.left.position_differential_gain,  // s_pkd
                    this->info.right.position_differential_gain, // s_pkd
                    this->info.left.position_feed_forward_gain,  // s_pkf
                    this->info.right.position_feed_forward_gain, // s_pkf

                    this->info.left.current_proportional_gain,  // s_ckp
                    this->info.right.current_proportional_gain, // s_ckp
                    this->info.left.current_integral_gain,      // s_cki
                    this->info.right.current_integral_gain,     // s_cki

                    this->info.acceleration_ms, //
                    this->info.deceleration_ms, //
                    this->info.SPEED_LIMIT      //

            );
            File dataFile = SPIFFS.open(SETTING_ZLAC_SETTING_FILE, FILE_WRITE);
            if (!dataFile) {
                result = false;
            } else {
                dataFile.println(buffer);
                dataFile.close();
                result = true;
            }
            SPIFFS.end();
        }

        return result;
    }
    bool setting_load()
    {
        bool result    = false;
        int totalBytes = 0;
        int line       = 0;
        if (true == SPIFFS.begin(false)) {
            if (true == SPIFFS.exists(SETTING_ZLAC_SETTING_FILE)) {
                File dataFile = SPIFFS.open(SETTING_ZLAC_SETTING_FILE, FILE_READ);
                if (!dataFile) {
                    result = false;
                } else {
                    result     = true;
                    totalBytes = dataFile.size();
                    while (0 < dataFile.available()) {
                        String word = dataFile.readStringUntil('\n');
                        switch (line) {
                            case 0:
#if SETTING_LOAD_FILE_SETTING_INTERVAL
                                if (true == word.equals("t")) {
                                    this->info.left.interval = true;
                                } else {
                                    this->info.left.interval = false;
                                }
#endif
                                break;
                            case 1:
#if SETTING_LOAD_FILE_SETTING_INTERVAL
                                if (true == word.equals("t")) {
                                    this->info.right.interval = true;
                                } else {
                                    this->info.right.interval = false;
                                }
#endif
                                break;
                            case 2:
                                this->info.left.speed_proportional_gain = this->_to_int(word, SETTING_SPEED_PROPORTIONAL_GAIN);
                                break;
                            case 3:
                                this->info.right.speed_proportional_gain = this->_to_int(word, SETTING_SPEED_PROPORTIONAL_GAIN);
                                break;
                            case 4:
                                this->info.left.speed_integral_gain = this->_to_int(word, SETTING_SPEED_INTEGRAL_GAIN);
                                break;
                            case 5:
                                this->info.right.speed_integral_gain = this->_to_int(word, SETTING_SPEED_INTEGRAL_GAIN);
                                break;
                            case 6:
                                this->info.left.speed_differential_gain = this->_to_int(word, SETTING_SPEED_DIFFERENTIAL_GAIN);
                                break;
                            case 7:
                                this->info.right.speed_differential_gain = this->_to_int(word, SETTING_SPEED_DIFFERENTIAL_GAIN);
                                break;

                            case 8:
                                this->info.left.position_proportional_gain = this->_to_int(word, SETTING_POSITION_PROPORTIONAL_GAIN);
                                break;
                            case 9:
                                this->info.right.position_proportional_gain = this->_to_int(word, SETTING_POSITION_PROPORTIONAL_GAIN);
                                break;
                            case 10:
                                this->info.left.position_differential_gain = this->_to_int(word, SETTING_POSITION_DIFFERENTIAL_GAIN);
                                break;
                            case 11:
                                this->info.right.position_differential_gain = this->_to_int(word, SETTING_POSITION_DIFFERENTIAL_GAIN);
                                break;
                            case 12:
                                this->info.left.position_feed_forward_gain = this->_to_int(word, SETTING_POSITION_FEED_FORWARD_GAIN);
                                break;
                            case 13:
                                this->info.right.position_feed_forward_gain = this->_to_int(word, SETTING_POSITION_FEED_FORWARD_GAIN);
                                break;

                            case 14:
                                this->info.left.current_proportional_gain = this->_to_int(word, SETTING_CURRENT_PROPORTIONAL_GAIN);
                                break;
                            case 15:
                                this->info.right.current_proportional_gain = this->_to_int(word, SETTING_CURRENT_PROPORTIONAL_GAIN);
                                break;
                            case 16:
                                this->info.left.current_integral_gain = this->_to_int(word, SETTING_CURRENT_INTEGRAL_GAIN);
                                break;
                            case 17:
                                this->info.right.current_integral_gain = this->_to_int(word, SETTING_CURRENT_INTEGRAL_GAIN);
                                break;
                            case 18:
                                this->info.acceleration_ms = this->_to_int(word, SETTING_SPEED_ACCELERATION_MS);
                                break;
                            case 19:
                                this->info.deceleration_ms = this->_to_int(word, SETTING_SPEED_DECELERATION_MS);
                                break;
                            case 20:
                                this->info.SPEED_LIMIT = this->_to_int(word, SETTING_SYSTEM_SPEED_LIMIT_RPM);
                                break;
                            default:
                                break;
                        }
                        line++;
                        if (20 < line) {
                            break;
                        }
                    }
                    dataFile.close();
                }
            }
            SPIFFS.end();
        }

        return result;
    }

protected:
    int _to_int(String data, int default_value)
    {
        int value = default_value;
        if (true != data.isEmpty()) {
            value = std::stoi(data.c_str());
        }
        return value;
    }

private:
    bool _flag_error = false;
};

#endif
