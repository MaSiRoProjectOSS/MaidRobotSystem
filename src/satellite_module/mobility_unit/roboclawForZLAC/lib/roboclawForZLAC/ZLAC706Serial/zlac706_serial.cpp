/**
 * @file zlac706_serial.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-05
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "zlac706_serial.hpp"

#include <SPIFFS.h>

///////////////////////////////////////////////////////////////////
// Define [LOGGER]
///////////////////////////////////////////////////////////////////
#pragma region LOGGER
#ifndef DEBUG_ZLAC706_SERIAL
#define DEBUG_ZLAC706_SERIAL 0
#endif
#ifndef DEBUG_TRACE
#define DEBUG_TRACE 0
#endif

#if DEBUG_ZLAC706_SERIAL
#if DEBUG_TRACE
#define LOG_TRACE(mess) this->log_trace((mess), __func__, __FILENAME__, __LINE__)
#else
#define LOG_TRACE(mess)
#endif
#else
#define LOG_TRACE(mess)
#endif
#pragma endregion

///////////////////////////////////////////////////////////////////
// constructor
///////////////////////////////////////////////////////////////////
#pragma region constructor
ZLAC706Serial::ZLAC706Serial()
{
    this->info.left.interval  = SETTING_INTERVAL_LEFT;
    this->info.right.interval = SETTING_INTERVAL_RIGHT;
}

ZLAC706Serial::~ZLAC706Serial()
{
    if (nullptr != this->_serial_driver_1) {
        this->_serial_driver_1->end();
    }
    if (nullptr != this->_serial_driver_2) {
        this->_serial_driver_2->end();
    }
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// public function
///////////////////////////////////////////////////////////////////
#pragma region public_function
bool ZLAC706Serial::begin()
{
    bool result = true;
#if DEBUG_ZLAC706_SERIAL
    this->output_log_enable();
#if DEBUG_TRACE
    this->output_log_level(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_ALL);
#endif
#endif
    LOG_TRACE(__func__);
    delay(100);
    this->_clear_receive(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_ALL);
    this->info.system.set(LOG_BEGIN, 0, 0);

    return result;
}

bool ZLAC706Serial::loop()
{
    bool result = true;
    LOG_TRACE(__func__);
    return result;
}
bool ZLAC706Serial::setting_save()
{
    bool result = false;
    char buffer[255];
    if (true == SPIFFS.begin(false)) {
        sprintf(buffer,
                "%s\n%s\n"                 //
                "%d\n%d\n%d\n%d\n%d\n%d\n" //
                "%d\n%d\n%d\n%d\n%d\n%d\n" //
                "%d\n%d\n%d\n%d\n"         //
                ,                          //

                info.left.interval ? "t" : "f",
                info.right.interval ? "t" : "f",

                info.left.speed_proportional_gain,  // s_skp
                info.right.speed_proportional_gain, // s_skp
                info.left.speed_integral_gain,      // s_ski
                info.right.speed_integral_gain,     // s_ski
                info.left.speed_differential_gain,  // s_skd
                info.right.speed_differential_gain, // s_skd

                info.left.position_proportional_gain,  // s_pkp
                info.right.position_proportional_gain, // s_pkp
                info.left.position_differential_gain,  // s_pkd
                info.right.position_differential_gain, // s_pkd
                info.left.position_feed_forward_gain,  // s_pkf
                info.right.position_feed_forward_gain, // s_pkf

                info.left.current_proportional_gain,  // s_ckp
                info.right.current_proportional_gain, // s_ckp
                info.left.current_integral_gain,      // s_cki
                info.right.current_integral_gain      // s_cki
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

bool ZLAC706Serial::setting_load()
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
                while (dataFile.available()) {
                    String word = dataFile.readStringUntil('\n');
                    switch (line) {
                        case 0:
                            if (true == word.equals("t")) {
                                this->info.left.interval = true;
                            } else {
                                this->info.left.interval = false;
                            }
                            break;
                        case 1:
                            if (true == word.equals("t")) {
                                this->info.right.interval = true;
                            } else {
                                this->info.right.interval = false;
                            }
                            break;
                        case 2:
                            info.left.speed_proportional_gain = this->_to_int(word, SETTING_SPEED_PROPORTIONAL_GAIN);
                            break;
                        case 3:
                            info.right.speed_proportional_gain = this->_to_int(word, SETTING_SPEED_PROPORTIONAL_GAIN);
                            break;
                        case 4:
                            info.left.speed_integral_gain = this->_to_int(word, SETTING_SPEED_INTEGRAL_GAIN);
                            break;
                        case 5:
                            info.right.speed_integral_gain = this->_to_int(word, SETTING_SPEED_INTEGRAL_GAIN);
                            break;
                        case 6:
                            info.left.speed_differential_gain = this->_to_int(word, SETTING_SPEED_DIFFERENTIAL_GAIN);
                            break;
                        case 7:
                            info.right.speed_differential_gain = this->_to_int(word, SETTING_SPEED_DIFFERENTIAL_GAIN);
                            break;

                        case 8:
                            info.left.position_proportional_gain = this->_to_int(word, SETTING_POSITION_PROPORTIONAL_GAIN);
                            break;
                        case 9:
                            info.right.position_proportional_gain = this->_to_int(word, SETTING_POSITION_PROPORTIONAL_GAIN);
                            break;
                        case 10:
                            info.left.position_differential_gain = this->_to_int(word, SETTING_POSITION_DIFFERENTIAL_GAIN);
                            break;
                        case 11:
                            info.right.position_differential_gain = this->_to_int(word, SETTING_POSITION_DIFFERENTIAL_GAIN);
                            break;
                        case 12:
                            info.left.position_feed_forward_gain = this->_to_int(word, SETTING_POSITION_FEED_FORWARD_GAIN);
                            break;
                        case 13:
                            info.right.position_feed_forward_gain = this->_to_int(word, SETTING_POSITION_FEED_FORWARD_GAIN);
                            break;

                        case 14:
                            info.left.current_proportional_gain = this->_to_int(word, SETTING_CURRENT_PROPORTIONAL_GAIN);
                            break;
                        case 15:
                            info.right.current_proportional_gain = this->_to_int(word, SETTING_CURRENT_PROPORTIONAL_GAIN);
                            break;
                        case 16:
                            info.left.current_integral_gain = this->_to_int(word, SETTING_CURRENT_INTEGRAL_GAIN);
                            break;
                        case 17:
                            info.right.current_integral_gain = this->_to_int(word, SETTING_CURRENT_INTEGRAL_GAIN);
                            break;
                        default:
                            break;
                    }
                    line++;
                    if (17 < line) {
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
int ZLAC706Serial::_to_int(String data, int default_value)
{
    if (true != data.isEmpty()) {
        int value = std::stoi(data.c_str());
        return value;
    } else {
        return default_value;
    }
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// Setup
///////////////////////////////////////////////////////////////////
#pragma region setup
bool ZLAC706Serial::setup_serial_driver(HardwareSerial *serial_l, HardwareSerial *serial_r, unsigned long baud)
{
    bool result = false;
    try {
#if SETTING_MOTOR_ENABLE_LEFT
        if (nullptr != serial_l) {
            this->_serial_driver_1 = serial_l;
            this->_serial_driver_1->setTimeout(this->TIMEOUT_DRIVER_MS);
            this->_serial_driver_1->setRxBufferSize(this->RX_BUFFER_SIZE);
            this->_serial_driver_1->setTxBufferSize(this->TX_BUFFER_SIZE);
            this->_serial_driver_1->begin(baud, SERIAL_8N1, -1, -1, this->_flag_invert, this->TIMEOUT_DRIVER_MS);
            result = true;
        }
#endif
#if SETTING_MOTOR_ENABLE_RIGHT
        if (nullptr != serial_r) {
            this->_serial_driver_2 = serial_r;
            this->_serial_driver_2->setTimeout(this->TIMEOUT_DRIVER_MS);
            this->_serial_driver_2->setRxBufferSize(this->RX_BUFFER_SIZE);
            this->_serial_driver_2->setTxBufferSize(this->TX_BUFFER_SIZE);
            this->_serial_driver_2->begin(baud, SERIAL_8N1, -1, -1, this->_flag_invert, this->TIMEOUT_DRIVER_MS);
            result = true;
        }
#endif
    } catch (...) {
    }
    return result;
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// Check status
///////////////////////////////////////////////////////////////////
#pragma region check_status
bool ZLAC706Serial::is_connection()
{
    static bool flag_previous = false;
    bool result               = (!this->info.left.error.not_connection) && (!this->info.right.error.not_connection);

    if (flag_previous != result) {
        this->info.system.set(((true == result) ? LOG_CONNECTED : LOG_DISCONNECTED), 0, 0);
    }
    flag_previous = result;
    return result;
}

bool ZLAC706Serial::is_error_flag()
{
    return this->_flag_error;
}

bool ZLAC706Serial::is_error()
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
    // result |= error_left.stop_state || error_left.startup_state;
    result_01 |= this->info.left.error.over_current || this->info.left.error.over_voltage || this->info.left.error.under_voltage;
    result_01 |= this->info.left.error.encoder_error || this->info.left.error.overheat || this->info.left.error.overload;
    result_01 |= this->info.left.error.not_connection;
#else
    result_02 |= false;
#endif
#if SETTING_MOTOR_ENABLE_RIGHT
    // result |= error_right.stop_state || error_right.startup_state;
    result_02 |= this->info.right.error.over_current || this->info.right.error.over_voltage || this->info.right.error.under_voltage;
    result_02 |= this->info.right.error.encoder_error || this->info.right.error.overheat || this->info.right.error.overload;
    result_02 |= this->info.right.error.not_connection;
#else
    result_02 |= false;
#endif

    if (flag_previous_01 != result_01) {
        this->info.system.set(LOG_IS_ERROR,
                              DRIVER_TARGET::DRIVER_TARGET_LEFT,
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
                              DRIVER_TARGET::DRIVER_TARGET_LEFT,
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
ZLAC706Serial::DRIVER_MODE ZLAC706Serial::get_mode()
{
    return this->info.mode;
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// Command [commutation]
///////////////////////////////////////////////////////////////////
#pragma region commutation
void ZLAC706Serial::_clear_receive(DRIVER_TARGET target)
{
    char buf[100];
#if DEBUG_TRACE
    bool result_01            = false;
    bool result_02            = false;
    std::string debug_message = "";
#else
    size_t size = 0;
#endif
    if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
        if (nullptr != this->_serial_driver_1) {
#if DEBUG_TRACE
            debug_message = "clear data[1] ->";
            while (0 != this->_serial_driver_1->available()) {
                sprintf(buf, " 0x%02X", this->_serial_driver_1->read());
                debug_message.append(buf);
                result_01 = true;
            }
            if (true == result_01) {
                LOG_TRACE(debug_message.c_str());
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
            //this->info.system.set(LOG_CLEAR_RECEIVE, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0);
        }
    }
    if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
        if (nullptr != this->_serial_driver_2) {
#if DEBUG_TRACE
            debug_message = "clear data[2] ->";
            while (0 != this->_serial_driver_2->available()) {
                sprintf(buf, " 0x%02X", this->_serial_driver_2->read());
                debug_message.append(buf);
                result_02 = true;
            }
            if (true == result_02) {
                LOG_TRACE(debug_message.c_str());
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
            //this->info.system.set(LOG_CLEAR_RECEIVE, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0);
        }
    }
}

#pragma endregion

///////////////////////////////////////////////////////////////////
// Command [Setup]
///////////////////////////////////////////////////////////////////
#pragma region command_setting
bool ZLAC706Serial::cmd_setting_proportional_gain(DRIVER_TARGET target, int value)
{
    LOG_TRACE(__func__);
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
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            if (0x40 != a1) {
                this->info.left.position_proportional_gain = value;
            } else {
                this->info.left.speed_proportional_gain = value;
            }
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            if (0x40 != a1) {
                this->info.right.position_proportional_gain = value;
            } else {
                this->info.right.speed_proportional_gain = value;
            }
        }
    }
    return result;
}
bool ZLAC706Serial::cmd_setting_integral_gain(DRIVER_TARGET target, int value)
{
    LOG_TRACE(__func__);
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
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            this->info.left.speed_integral_gain = value;
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            this->info.right.speed_integral_gain = value;
        }
    }
    return result;
}
bool ZLAC706Serial::cmd_setting_differential_gain(DRIVER_TARGET target, int value)
{
    LOG_TRACE(__func__);
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
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            if (0x42 != a1) {
                this->info.left.position_differential_gain = value;
            } else {
                this->info.left.speed_differential_gain = value;
            }
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            if (0x42 != a1) {
                this->info.right.position_differential_gain = value;
            } else {
                this->info.right.speed_differential_gain = value;
            }
        }
    }
    return result;
}
bool ZLAC706Serial::cmd_setting_feed_forward_gain(DRIVER_TARGET target, int value)
{
    LOG_TRACE(__func__);
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
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            this->info.left.position_feed_forward_gain = value;
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            this->info.right.position_feed_forward_gain = value;
        }
    }
    return result;
}
bool ZLAC706Serial::cmd_setting_inverted(DRIVER_TARGET target, bool value)
{
    if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
        this->info.left.interval = value;
    }
    if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
        this->info.right.interval = value;
    }
    return true;
}
bool ZLAC706Serial::cmd_setting_acc(DRIVER_TARGET target, int value)
{
    if (0 > value) {
        value = 0;
    }
    this->info.acceleration_ms = value;
    this->cmd_speed_set_acc_and_dec(this->info.acceleration_ms, this->info.deceleration_ms, target);

    return true;
}
bool ZLAC706Serial::cmd_setting_dcc(DRIVER_TARGET target, int value)
{
    if (0 > value) {
        value = 0;
    }
    this->info.deceleration_ms = value;
    this->cmd_speed_set_acc_and_dec(this->info.acceleration_ms, this->info.deceleration_ms, target);
    return true;
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// Command [Controller]
///////////////////////////////////////////////////////////////////
#pragma region command_controller
bool ZLAC706Serial::cmd_motor_start(DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    bool result = true;
    if (false == this->info.flag.running) {
        result = this->_send_target(__func__, target, 0x00, 0x00, 0x01, true);
        if (true == result) {
            this->info.flag.running = true;
        }
    }

    return result;
}
bool ZLAC706Serial::cmd_motor_stop(DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    bool result = true;
    if (true == this->info.flag.running) {
        result = this->_send_target(__func__, target, 0x00, 0x00, 0x00, true);
        if (true == result) {
            this->info.left.speed_rpm  = 0;
            this->info.right.speed_rpm = 0;
            this->info.flag.running    = false;
        }
    }

    return result;
}
bool ZLAC706Serial::cmd_position_mode()
{
    this->_flag_monitoring_speed = false;
    this->info.mode              = DRIVER_MODE::POSITION_FROM_DIGITAL;
    this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
    return this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_ALL, 0x02, 0x00, 0xD0, true);
}
bool ZLAC706Serial::cmd_position_mode_pulse()
{
    this->_flag_monitoring_speed = false;
    this->info.mode              = DRIVER_MODE::POSITION_FROM_PULSE;
    this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
    return this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_ALL, 0x02, 0x00, 0xC0, true);
}
bool ZLAC706Serial::cmd_torque_mode()
{
    this->_flag_monitoring_speed = false;
    this->info.mode              = DRIVER_MODE::TORQUE_FROM_DIGITAL;
    this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
    return this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_ALL, 0x02, 0x00, 0xC1, true);
}
bool ZLAC706Serial::cmd_speed_mode()
{
    this->_flag_monitoring_speed = true;
    this->info.mode              = DRIVER_MODE::SPEED_FROM_DIGITAL;
    this->info.system.set(LOG_MODE, 0, 0, this->info.mode);
    return this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_ALL, 0x02, 0x00, 0xC4, true);
}

bool ZLAC706Serial::cmd_mode_selection(DRIVER_MODE mode)
{
    LOG_TRACE(__func__);
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
            result = this->cmd_motor_stop(DRIVER_TARGET::DRIVER_TARGET_ALL);
            break;
    }
    return result;
}
bool ZLAC706Serial::cmd_speed_set_acc_and_dec(int acceleration_ms, int deceleration_ms, DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
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
bool ZLAC706Serial::cmd_position_set(long pos_l, int rpm_l, long pos_r, int rpm_r)
{
    LOG_TRACE(__func__);
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

    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0x1D, (rpm_l >> 8) & 0xFF, (rpm_l >> 0) & 0xFF, true);
    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0x1D, (rpm_r >> 8) & 0xFF, (rpm_r >> 0) & 0xFF, true);
    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0x50, (pos_l >> 24) & 0xFF, (pos_l >> 16) & 0xFF, true);
    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0x50, (pos_r >> 24) & 0xFF, (pos_r >> 16) & 0xFF, true);
    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0x05, (pos_l >> 8) & 0xFF, (pos_l >> 0) & 0xFF, false);
    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0x05, (pos_r >> 8) & 0xFF, (pos_r >> 0) & 0xFF, false);

    result_01 = this->_confirm(__func__, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0x05);
    result_02 = this->_confirm(__func__, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0x05);

    return result_01 && result_02;
}
bool ZLAC706Serial::cmd_torque_set(int value_l_mA, int value_r_mA)
{
    LOG_TRACE(__func__);
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
            this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0x08, (start_value >> 8) & 0xFF, (start_value >> 0) & 0xFF, false);
            this->cmd_motor_start(DRIVER_TARGET::DRIVER_TARGET_LEFT);
            delay(this->TIMEOUT_DRIVER_MS);
        }
    }

    if (false == flag_running_r) {
        if (threshold < abs(value_r)) {
            this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0x08, (start_value >> 8) & 0xFF, (start_value >> 0) & 0xFF, false);
            this->cmd_motor_start(DRIVER_TARGET::DRIVER_TARGET_RIGHT);
            delay(this->TIMEOUT_DRIVER_MS);
        }
    }

    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0x08, (value_l >> 8) & 0xFF, (value_l >> 0) & 0xFF, false);
    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0x08, (value_r >> 8) & 0xFF, (value_r >> 0) & 0xFF, false);

    result_01 = this->_confirm(__func__, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0x08);
    result_02 = this->_confirm(__func__, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0x08);

    if (threshold < abs(value_l)) {
        this->cmd_motor_start(DRIVER_TARGET::DRIVER_TARGET_LEFT);
        flag_running_l = true;
    } else {
        this->cmd_motor_stop(DRIVER_TARGET::DRIVER_TARGET_LEFT);
        flag_running_l = false;
    }
    if (threshold < abs(value_r)) {
        this->cmd_motor_start(DRIVER_TARGET::DRIVER_TARGET_RIGHT);
        flag_running_r = true;
    } else {
        this->cmd_motor_stop(DRIVER_TARGET::DRIVER_TARGET_RIGHT);
        flag_running_r = false;
    }

    return result_01 && result_02;
}
bool ZLAC706Serial::cmd_speed_set(int rpm_l, int rpm_r)
{
    LOG_TRACE(__func__);
    bool result_01 = true;
    bool result_02 = true;
    if (true == this->info.left.interval) {
        rpm_l = -rpm_l;
    }
    if (true == this->info.right.interval) {
        rpm_r = -rpm_r;
    }

    if ((true == this->info.flag.emergency) || (false == this->info.flag.heart_beat)) {
        rpm_l = 0;
        rpm_r = 0;
    }
    this->info.left.speed_request_rpm  = rpm_l;
    this->info.right.speed_request_rpm = rpm_r;

    if (this->info.SPEED_LIMIT < rpm_l) {
        rpm_l = this->info.SPEED_LIMIT;
    } else if (rpm_l < -this->info.SPEED_LIMIT) {
        rpm_l = -this->info.SPEED_LIMIT;
    }
    if (this->info.SPEED_LIMIT < rpm_r) {
        rpm_r = this->info.SPEED_LIMIT;
    } else if (rpm_r < -this->info.SPEED_LIMIT) {
        rpm_r = -this->info.SPEED_LIMIT;
    }
    int value_l = (rpm_l * 8192) / 3000;
    int value_r = (rpm_r * 8192) / 3000;

    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0x06, (value_l >> 8) & 0xFF, (value_l >> 0) & 0xFF, false);
    this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0x06, (value_r >> 8) & 0xFF, (value_r >> 0) & 0xFF, false);

    result_01 = this->_confirm(__func__, DRIVER_TARGET::DRIVER_TARGET_LEFT, 0x06);
    result_02 = this->_confirm(__func__, DRIVER_TARGET::DRIVER_TARGET_RIGHT, 0x06);
    return result_01 && result_02;
}
bool ZLAC706Serial::cmd_looking_for_z_signal(DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    return this->_send_target(__func__, target, 0x53, 0x00, 0x00, true);
}

bool ZLAC706Serial::cmd_clear_fault(DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    return this->_send_target(__func__, target, 0x4A, 0x00, 0x00, true);
}
bool ZLAC706Serial::cmd_position_set_absolute()
{
    LOG_TRACE(__func__);
    this->info.position_absolute = 1;
    return this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_ALL, 0x51, 0x00, 0x00, true);
}
bool ZLAC706Serial::cmd_position_set_relative()
{
    LOG_TRACE(__func__);
    this->info.position_absolute = 0;
    return this->_send_target(__func__, DRIVER_TARGET::DRIVER_TARGET_ALL, 0x51, 0x00, 0x01, true);
}
bool ZLAC706Serial::cmd_modify_the_rated_current(int value_mW, DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    this->info.rated_current_mW = value_mW;
    return this->_send_target(__func__, target, 0x2D, (value_mW >> 8) & 0xFF, (value_mW >> 0) & 0xFF, true);
}
void ZLAC706Serial::cmd_get_all_status(DRIVER_TARGET target)
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
bool ZLAC706Serial::cmd_get_alarm_status(DRIVER_TARGET target)
{
#ifndef DEBUG_TRACE
    LOG_TRACE(__func__);
#endif
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
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            result_01 = false;
            if (4 <= this->_receive(__func__, this->_serial_driver_1, 4, buffer, flag_output)) {
                this->info.left.error.not_connection = false;
                this->info.left.error.stop_state     = ((buffer[3] & 0x01) > 0) ? false : true;
                this->info.left.error.startup_state  = ((buffer[3] & 0x80) > 0) ? true : false;

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
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            result_02 = false;
            if (4 <= this->_receive(__func__, this->_serial_driver_2, 4, buffer, flag_output)) {
                this->info.right.error.not_connection = false;
                this->info.right.error.stop_state     = ((buffer[3] & 0x01) > 0) ? false : true;
                this->info.right.error.startup_state  = ((buffer[3] & 0x80) > 0) ? true : false;

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

bool ZLAC706Serial::cmd_speed_heart_beat()
{
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
            if (next_heart_beat_ms <= current_time) {
                if (false == this->info.flag.running) {
                    if (next_get_state_ms <= current_time) {
                        next_get_state_ms = current_time + TIME_GET_STATE_MS;
                        this->cmd_get_all_status();
                    }
                } else {
#ifndef DEBUG_TRACE
                    LOG_TRACE(__func__);
#endif
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
                }

                delay(this->INTERVAL_DRIVER_MS);
            }
        }
    }
    return result_01 && result_02;
}

bool ZLAC706Serial::cmd_get_bus_voltage(DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    bool result_01   = true;
    bool result_02   = true;
    char buffer[255] = { 0 };
    if (true == this->_send_target(__func__, target, 0x61, 0x00, 0x00, true)) {
        result_01 = false;
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            if (4 <= this->_receive(__func__, this->_serial_driver_1, 4, buffer)) {
                this->info.left.voltage = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                result_01               = true;
            }
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            result_02 = false;
            if (4 <= this->_receive(__func__, this->_serial_driver_2, 4, buffer)) {
                this->info.right.voltage = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                result_02                = true;
            }
        }
    }
    return result_01 && result_02;
}

bool ZLAC706Serial::cmd_get_output_current(DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    bool result_01   = true;
    bool result_02   = true;
    char buffer[255] = { 0 };
    if (true == this->_send_target(__func__, target, 0x62, 0x00, 0x00, true)) {
        result_01 = false;
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            if (4 <= this->_receive(__func__, this->_serial_driver_1, 4, buffer)) {
                this->info.left.current = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                result_01               = true;
            }
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            result_02 = false;
            if (4 <= this->_receive(__func__, this->_serial_driver_2, 4, buffer)) {
                this->info.right.current = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                result_02                = true;
            }
        }
    }
    return result_01 && result_02;
}

bool ZLAC706Serial::cmd_get_motor_speed(DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    bool result_01   = true;
    bool result_02   = true;
    int get_value    = 0;
    char buffer[255] = { 0 };
    if (true == this->_send_target(__func__, target, 0x63, 0x00, 0x00, true)) {
        result_01 = false;
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            if (4 <= this->_receive(__func__, this->_serial_driver_1, 4, buffer)) {
                get_value                 = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                this->info.left.speed_rpm = (get_value * 3000) / 8192;
                result_01                 = true;
            }
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            result_02 = false;
            if (4 <= this->_receive(__func__, this->_serial_driver_2, 4, buffer)) {
                get_value                  = (int16_t)((buffer[1] << 8) | (buffer[2] << 0));
                this->info.right.speed_rpm = (get_value * 3000) / 8192;
                result_02                  = true;
            }
        }
    }
    return result_01 && result_02;
}

bool ZLAC706Serial::cmd_get_position_given(DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    bool result_01 = true;
    bool result_02 = true;

    char buffer[255] = { 0 };
    if (true == this->_send_target(__func__, target, 0x64, 0x00, 0x00, true)) {
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            result_01 = false;
            if (8 <= this->_receive(__func__, this->_serial_driver_1, 8, buffer)) {
                this->info.left.position_given = (int32_t)((buffer[1] << 24) | (buffer[2] << 16) | (buffer[5] << 8) | (buffer[6] << 0));
                result_01                      = true;
            }
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            result_02 = false;
            if (8 <= this->_receive(__func__, this->_serial_driver_2, 8, buffer)) {
                this->info.right.position_given = (int32_t)((buffer[1] << 24) | (buffer[2] << 16) | (buffer[5] << 8) | (buffer[6] << 0));
                result_02                       = true;
            }
        }
    }
    return result_01 && result_02;
}

bool ZLAC706Serial::cmd_get_position_feedback(DRIVER_TARGET target)
{
    LOG_TRACE(__func__);
    bool result_01   = true;
    bool result_02   = true;
    char buffer[255] = { 0 };
    long temp        = 0;
    if (true == this->_send_target(__func__, target, 0x65, 0x00, 0x00, true)) {
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            result_01 = false;
            if (8 <= this->_receive(__func__, this->_serial_driver_1, 8, buffer)) {
                temp                                  = (int32_t)((buffer[1] << 24) | (buffer[2] << 16) | (buffer[5] << 8) | (buffer[6] << 0));
                this->info.left.position_feedback     = temp * ((true != this->info.left.interval) ? 1 : -1);
                this->info.left.position_feedback_deg = ((temp % SETTING_SYSTEM_POSITION_AROUND) * 360.0) / SETTING_SYSTEM_POSITION_AROUND;
                result_01                             = true;
            }
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
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

#pragma endregion

///////////////////////////////////////////////////////////////////
// private fuction
///////////////////////////////////////////////////////////////////
#pragma region private_fuction
bool ZLAC706Serial::_receive_wait(HardwareSerial *serial, int size)
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

int ZLAC706Serial::_receive(const char *name, HardwareSerial *serial, int size, char *buffer, bool output_log)
{
    //LOG_TRACE(__func__);
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
#if DEBUG_ZLAC706_SERIAL
        sprintf(buf, "Timeout : %d ms [%s]", (count - 1) * this->INTERVAL_DRIVER_MS, name);
        this->log_warn(buf);
#endif
    } else {
        if (true == output_log) {
#if DEBUG_TRACE
#if DEBUG_ZLAC706_SERIAL
            std::string debug_message_01 = "receive :";
            sprintf(buf, " count [%d] :", count - 1);
            debug_message_01.append(buf);
            for (int i = 0; i < size; i++) {
                sprintf(buf, " 0x%02X", buffer[i]);
                debug_message_01.append(buf);
            }
            LOG_TRACE(debug_message_01.c_str());
#endif
#endif
        }
    }
    return index;
}

bool ZLAC706Serial::_send_target(const char *name, DRIVER_TARGET target, char a1, char a2, char a3, bool confirm, bool output_log)
{
    bool result_01 = true;
    bool result_02 = true;
    bool result    = true;
    if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
        result_01 = this->_send(name, DRIVER_TARGET::DRIVER_TARGET_LEFT, this->_serial_driver_1, a1, a2, a3, confirm, output_log);
    }
    if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
        result_02 = this->_send(name, DRIVER_TARGET::DRIVER_TARGET_RIGHT, this->_serial_driver_2, a1, a2, a3, confirm, output_log);
    }
    return result_01 && result_02;
}

bool ZLAC706Serial::_send(const char *name, DRIVER_TARGET target, HardwareSerial *serial, char a1, char a2, char a3, bool confirm, bool output_log)
{
    //LOG_TRACE(__func__);
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
        LOG_TRACE(debug_message.c_str());
#endif
#endif
    }
    if (nullptr != serial) {
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            this->_clear_receive(DRIVER_TARGET::DRIVER_TARGET_LEFT);
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            this->_clear_receive(DRIVER_TARGET::DRIVER_TARGET_RIGHT);
        }
        serial->write(buffer, 4);
        if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            this->info.left.order.set(a1, a2, a3);
        }
        if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
            this->info.right.order.set(a1, a2, a3);
        }
        if (true == confirm) {
            result = this->_confirm(name, target, a1, output_log);
        }
    }
    return result;
}

bool ZLAC706Serial::_confirm(const char *name, DRIVER_TARGET target, char cmd, bool output_log)
{
    bool result      = false;
    char buffer[100] = { 0 };
    if ((DRIVER_TARGET::DRIVER_TARGET_LEFT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
        if (2 <= this->_receive(name, this->_serial_driver_1, 2, buffer, output_log)) {
            if (cmd == buffer[0]) {
                if (cmd == buffer[1]) {
                    result = true;
                }
            }
        }
    }
    if ((DRIVER_TARGET::DRIVER_TARGET_RIGHT == target) || (DRIVER_TARGET::DRIVER_TARGET_ALL == target)) {
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

#pragma endregion
