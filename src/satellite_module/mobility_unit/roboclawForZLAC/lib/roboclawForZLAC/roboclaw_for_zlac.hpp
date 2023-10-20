/**
 * @file roboclaw_for_zlac.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-01
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ROBOCLAW_FOR_ZLAC_HPP
#define ROBOCLAW_FOR_ZLAC_HPP
#include "ZLAC706Serial/zlac706_serial.hpp"
#include "config_roboclaw_for_zlac.hpp"
#include "log_callback.hpp"

#ifndef DRIVE_ID
#define DRIVE_ID 0x80
#endif

class RoboClawForZlac : public LogCallback {
public:
    typedef enum roboclaw_state
    {
        STATE_NOT_INITIALIZED,
        STATE_NOT_SETTING,
        STATE_RUNNING,
        STATE_ERROR,
        STATE_EMERGENCY,

    } ROBOCLAW_STATE;

public:
    RoboClawForZlac();

    bool setup(HardwareSerial *input_serial, //
               HardwareSerial *motor_driver_left,
               HardwareSerial *motor_driver_right,
               unsigned long input_baud = 115200);
    bool update_id(int id = DRIVE_ID);

    bool begin();
    bool loop();

    bool reset();
    roboclaw_state get_state();
    ZLAC706Serial::zlac_info get_zlac_info();
    void set_emergency(bool emergency);
    bool setting_proportional_gain(ZLAC706Serial::DRIVER_TARGET target, int value);
    bool setting_integral_gain(ZLAC706Serial::DRIVER_TARGET target, int value);
    bool setting_differential_gain(ZLAC706Serial::DRIVER_TARGET target, int value);
    bool setting_feed_forward_gain(ZLAC706Serial::DRIVER_TARGET target, int value);
    bool setting_inverted(ZLAC706Serial::DRIVER_TARGET target, bool value);
    bool setting_acc(ZLAC706Serial::DRIVER_TARGET target, int value);
    bool setting_dcc(ZLAC706Serial::DRIVER_TARGET target, int value);
    bool setting_save();

public:
    void log_message_mode(bool enable);

private:
    const unsigned long TIMEOUT_INPUT_SERIAL_MS = 10;
    uint8_t _id                                 = DRIVE_ID;
    HardwareSerial *_input_serial;
    ZLAC706Serial *_zlac;
    bool _flag_initialized = false;
    bool _flag_setting     = false;
    void _set_log_level();
    void _receive();
    bool _check_id(uint8_t id);
    bool _receive_wait(int size);

    void _response(unsigned int crc, uint8_t data[100], int size, bool add_crc = true);
    unsigned int _crc_update(unsigned int crc, uint8_t data);
    int _rpm_to_mps(int value);
    int _mps_to_rpm(int value);

    bool _setting_load();

private:
    void _controller(uint8_t command, size_t command_size, uint8_t value[100], size_t value_size);
    void _read_version(unsigned int crc);
    void _speed_m1m2(unsigned int crc, uint8_t value[100], size_t value_size);
    void _reset_encoders(unsigned int crc, uint8_t value[100], size_t value_size);
    void _forward_m1(unsigned int crc, uint8_t value[100], size_t value_size);
    void _forward_m2(unsigned int crc, uint8_t value[100], size_t value_size);
    void _backwards_m1(unsigned int crc, uint8_t value[100], size_t value_size);
    void _backwards_m2(unsigned int crc, uint8_t value[100], size_t value_size);
    void _read_enc_m1(unsigned int crc);
    void _read_enc_m2(unsigned int crc);
    void _read_error(unsigned int crc);
    void _read_main_battery_voltage(unsigned int crc);
    void _read_logic_battery_voltage(unsigned int crc);
    void _read_temp(unsigned int crc);
    void _read_temp2(unsigned int crc);

    void _set_speed(int speed_left, int speed_right, bool enable_left = true, bool enable_right = true);

    long _enc_difference = 0;
    long _enc_postion    = 0;
    inline int _get_sign(double num);
    bool _check_crc(uint8_t id, uint8_t command, uint8_t *packet, int nBytes);

protected:
    bool callback_message(MessageFunction callback) override;

private:
    unsigned long call_time = 0;

private:
    const unsigned long HEARTBEAT_INTERVAL = SETTING_HEARTBEAT_INTERVAL_MS;
};

#endif
