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
#include "config_roboclaw_for_zlac.hpp"

#define ZLAC8015D_MODBUS 0x01
#define ZLAC706_SERIAL   0x02
#ifndef ZLAC_DRIVER
#define ZLAC_DRIVER ZLAC706_SERIAL
#endif

#if ZLAC_DRIVER == ZLAC8015D_MODBUS
#include "driver_zlac/zlac8015d_modbus.hpp"
#elif ZLAC_DRIVER == ZLAC706_SERIAL
#include "driver_zlac/zlac706_serial.hpp"
#endif

#ifndef DRIVE_ID
#define DRIVE_ID 0x80
#endif

class RoboClawForZlac {
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
    ZlacDriver::zlac_info get_zlac_info();
    void set_log_update();
    void set_emergency(bool emergency);
    void set_motor_free(bool emergency);
    bool setting_proportional_gain(ZLAC::TARGET_MOTOR target, int value);
    bool setting_integral_gain(ZLAC::TARGET_MOTOR target, int value);
    bool setting_differential_gain(ZLAC::TARGET_MOTOR target, int value);
    bool setting_feed_forward_gain(ZLAC::TARGET_MOTOR target, int value);
    bool setting_inverted(ZLAC::TARGET_MOTOR target, bool value);
    bool setting_acc(ZLAC::TARGET_MOTOR target, int value);
    bool setting_dcc(ZLAC::TARGET_MOTOR target, int value);
    bool setting_save();
    bool setting_limit(ZLAC::TARGET_MOTOR target, int value);

public:
    bool is_error_flag();

    FourDimensionalChart *speed_mps_feedback();
    FourDimensionalChart *speed_mps_request();
    int to_motor_value(int value);

private:
    const unsigned long TIMEOUT_INPUT_SERIAL_MS = 10;
    uint8_t _id                                 = DRIVE_ID;
    HardwareSerial *_input_serial;

#if ZLAC_DRIVER == ZLAC8015D_MODBUS
    ZLAC8015DModbus *_zlac;
#elif ZLAC_DRIVER == ZLAC706_SERIAL
    ZLAC706Serial *_zlac;
#endif
    bool _flag_initialized = false;
    bool _flag_setting     = false;
    void _receive();
    bool _check_id(uint8_t id);
    bool _receive_wait(int size);

    void _response(unsigned int crc, uint8_t command, uint8_t data[100], int size, bool add_crc = true);
    unsigned int _crc_update(unsigned int crc, uint8_t data);

    bool _setting_load();

private:
    void _controller(uint8_t command, size_t command_size, uint8_t value[100], size_t value_size);
    void _read_version(unsigned int crc, uint8_t command);
    void _speed_m1m2(unsigned int crc, uint8_t value[100], size_t value_size);
    void _speed_m1(unsigned int crc, uint8_t value[100], size_t value_size);
    void _speed_m2(unsigned int crc, uint8_t value[100], size_t value_size);
    void _reset_encoders(unsigned int crc, uint8_t value[100], size_t value_size);
    void _forward_m1(unsigned int crc, uint8_t value[100], size_t value_size);
    void _forward_m2(unsigned int crc, uint8_t value[100], size_t value_size);
    void _backwards_m1(unsigned int crc, uint8_t value[100], size_t value_size);
    void _backwards_m2(unsigned int crc, uint8_t value[100], size_t value_size);
    void _read_enc_m1(unsigned int crc, uint8_t command);
    void _read_enc_m2(unsigned int crc, uint8_t command);
    void _read_error(unsigned int crc, uint8_t command);
    void _read_main_battery_voltage(unsigned int crc, uint8_t command);
    void _read_logic_battery_voltage(unsigned int crc, uint8_t command);
    void _read_temp(unsigned int crc, uint8_t command);
    void _read_temp2(unsigned int crc, uint8_t command);

    void _get_speed_m1(unsigned int crc, uint8_t command);
    void _get_speed_m2(unsigned int crc, uint8_t command);
    void _get_speed_m1m2(unsigned int crc, uint8_t command);

    void _set_speed(int speed_left, int speed_right, bool enable_left = true, bool enable_right = true);

    long _enc_difference = 0;
    long _enc_postion    = 0;
    inline int _get_sign(double num);
    bool _check_crc(uint8_t id, uint8_t command, uint8_t *packet, int nBytes);

private:
    unsigned long call_time = 0;

private:
    const unsigned long HEARTBEAT_INTERVAL = SETTING_HEARTBEAT_INTERVAL_MS;
};

#endif
