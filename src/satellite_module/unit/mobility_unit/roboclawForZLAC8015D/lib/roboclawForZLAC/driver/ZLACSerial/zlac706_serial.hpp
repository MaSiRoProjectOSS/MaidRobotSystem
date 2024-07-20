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

#include "../ZlacDriver.hpp"

#include <Arduino.h>

class ZLAC706Serial : public ZlacDriver {
public:
public:
    ZLAC706Serial();
    ~ZLAC706Serial();
    bool setup(HardwareSerial *serial_l, HardwareSerial *serial_r, unsigned long baud = 57600);

public:
    bool begin() override;
    bool loop() override;

    // Function : setting
    bool cmd_setting_proportional_gain(DRIVER_TARGET target, int value) override;
    bool cmd_setting_integral_gain(DRIVER_TARGET target, int value) override;
    bool cmd_setting_differential_gain(DRIVER_TARGET target, int value) override;
    bool cmd_setting_feed_forward_gain(DRIVER_TARGET target, int value) override;
    bool cmd_setting_inverted(DRIVER_TARGET target, bool value) override;
    bool cmd_setting_acc(DRIVER_TARGET target, int value) override;
    bool cmd_setting_dcc(DRIVER_TARGET target, int value) override;
    bool cmd_setting_limit(DRIVER_TARGET target, int value) override;

    // Function : command
    bool cmd_modify_the_rated_current(int value_mW, DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL) override;

    bool cmd_looking_for_z_signal(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL) override;
    bool cmd_clear_fault(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL) override;

    void cmd_get_all_status(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL) override;
    bool cmd_get_alarm_status(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL) override;
    bool cmd_get_bus_voltage(DRIVER_TARGET targe) override;
    bool cmd_get_output_current(DRIVER_TARGET target) override;
    bool cmd_get_motor_speed(DRIVER_TARGET target) override;
    bool cmd_get_position_given(DRIVER_TARGET target) override;
    bool cmd_get_position_feedback(DRIVER_TARGET target) override;

    //////////////////////////////////////
    bool cmd_mode_selection(DRIVER_MODE mode) override;
    //////////////////////////////////////
    bool cmd_motor_start(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL) override;
    bool cmd_motor_stop(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL) override;
    //////////////////////////////////////
    bool cmd_position_mode() override;
    bool cmd_position_mode_pulse() override;
    bool cmd_position_set_absolute() override;
    bool cmd_position_set_relative() override;
    bool cmd_position_set(long pos_l, int rpm_l, long pos_r, int rpm_r) override;
    //////////////////////////////////////
    bool cmd_torque_mode() override;
    bool cmd_torque_set(int value_l_mA, int value_r_mA) override;
    //////////////////////////////////////
    bool cmd_speed_mode() override;
    bool cmd_speed_set_acc_and_dec(int acceleration_ms, int deceleration_ms, DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL) override;
    bool cmd_speed_set(int milli_rpm_l, int milli_rpm_r) override;
    bool cmd_speed_heart_beat() override;

private:
    bool _confirm(const char *name, DRIVER_TARGET target, char cmd, bool output_log = true);
    bool _send_target(const char *name, DRIVER_TARGET target, char a1, char a2, char a3, bool confirm, bool output_log = false);
    bool _send(const char *name, DRIVER_TARGET target, HardwareSerial *serial, char a1, char a2, char a3, bool confirm, bool output_log = false);

    int _receive(const char *name, HardwareSerial *serial, int size, char *buffer, bool output_log = false);
    bool _receive_wait(HardwareSerial *serial, int size);
    //////////////////////////////////////
    void _clear_receive(DRIVER_TARGET target = DRIVER_TARGET::DRIVER_TARGET_ALL);

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
