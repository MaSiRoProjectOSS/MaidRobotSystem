/**
 * @file roboclaw_for_zlac.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-01
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "roboclaw_for_zlac.hpp"

#include "cmd_roboclaw_list.hpp"
#include "config_roboclaw_for_zlac.hpp"

#include <Arduino.h>

///////////////////////////////////////////////////////////////////
// Setup
///////////////////////////////////////////////////////////////////
#pragma region Setup
bool RoboClawForZlac::_check_crc(uint8_t id, uint8_t command, uint8_t *packet, int nBytes)
{
    bool result = false;
    if (2 < nBytes) {
        unsigned int crc_receive = (unsigned int)((packet[nBytes - 2] << 8) | (packet[nBytes - 1] << 0));
        unsigned int crc         = 0;
        crc                      = this->_crc_update(crc, id);
        crc                      = this->_crc_update(crc, command);
        for (int i = 0; i < (nBytes - 2); i++) {
            crc = this->_crc_update(crc, packet[i]);
        }
        if ((crc & 0xFFFF) == (crc_receive)) {
            result = true;
        } else {
            this->_zlac->info.system.set(ZLAC706Serial::SYSTEM_LOG::LOG_CRC_ERROR, 0, 0);
        }
    } else {
        result = true;
    }
    return result;
}
void RoboClawForZlac::_receive()
{
#if DEBUG_ZLAC706_SERIAL
    // [[[not all api support]]]

    const int SPEED_SPAN         = 2;
    const int SPEED_SPAN_ACC_DCC = 100;
    const int POSITION_SPAN      = 512;
    const int TORQUE_SPAN        = 50;
    ///////////////////////////////////////////
    // DEBUG_ZLAC706_SERIAL_MODE_SPEED
    // DEBUG_ZLAC706_SERIAL_MODE_POSITION
    static int left            = 0;
    static int right           = 0;
    static int acceleration_ms = ROBOCLAW_SPEED_ACCELERATION_MS;
    static int deceleration_ms = ROBOCLAW_SPEED_DECELERATION_MS;
    ///////////////////////////////////////////
    // DEBUG_ZLAC706_SERIAL_MODE_POSITION
    static long left_pos  = (4096) * 1;
    static long right_pos = (4096) * 1;
    ///////////////////////////////////////////
    // DEBUG_ZLAC706_SERIAL_MODE_TORQUE
    static int value_l_mA = 0;
    static int value_r_mA = 0;
    ///////////////////////////////////////////
    int crc            = 0;
    char mess[255]     = "";
    uint8_t buffer[10] = { 0 };

    uint8_t id         = 0;
    uint8_t command    = 0;
    int command_size   = 0;
    uint8_t value[100] = { 0 };
    size_t value_size  = 0;

    if (0 != this->_input_serial->available()) {
        buffer[0]                         = Serial.read();
        this->_zlac->info.flag.heart_beat = true;

#if DEBUG_ZLAC706_SERIAL_MODE == DEBUG_ZLAC706_SERIAL_MODE_POSITION
        bool flag_set_value = false;
        switch (buffer[0]) {
            case '1':
                this->_zlac->cmd_position_set_absolute();
                break;
            case '2':
                this->_zlac->cmd_position_set_relative();
                left_pos  = 0;
                right_pos = 0;
                break;
                ////////////////////////////////////////
            case 'q':
                left += SPEED_SPAN;
                flag_set_value = true;
                break;
            case 'a':
                left -= SPEED_SPAN;
                flag_set_value = true;
                break;
            case 'w':
                left_pos += POSITION_SPAN;
                flag_set_value = true;
                break;
            case 's':
                left_pos -= POSITION_SPAN;
                flag_set_value = true;
                break;
                ////////////////////////////////////////
            case 'e':
                right += SPEED_SPAN;
                flag_set_value = true;
                break;
            case 'd':
                right -= SPEED_SPAN;
                flag_set_value = true;
                break;
            case 'r':
                right_pos += POSITION_SPAN;
                flag_set_value = true;
                break;
            case 'f':
                right_pos -= POSITION_SPAN;
                flag_set_value = true;
                break;
                ////////////////////////////////////////
            default:
                left           = 0;
                right          = 0;
                flag_set_value = true;
                this->_zlac->cmd_position_set(left_pos, left, right_pos, right);
                this->_zlac->cmd_motor_stop();
                break;
        }
        if (true == flag_set_value) {
            this->_zlac->cmd_position_set(left_pos, left, right_pos, right);
            this->_zlac->cmd_motor_start();
        }
#elif DEBUG_ZLAC706_SERIAL_MODE == DEBUG_ZLAC706_SERIAL_MODE_TORQUE
        bool flag_send = false;
        switch (buffer[0]) {
            case 'q':
                value_l_mA += TORQUE_SPAN;
                flag_send = true;
                break;
            case 'a':
                value_l_mA -= TORQUE_SPAN;
                flag_send = true;
                break;
            case 'e':
                value_r_mA += TORQUE_SPAN;
                flag_send = true;
                break;
            case 'd':
                value_r_mA -= TORQUE_SPAN;
                flag_send = true;
                break;
            default:
                value_l_mA = 0;
                value_r_mA = 0;
                this->_zlac->cmd_torque_set(value_l_mA, value_r_mA);
                this->_zlac->cmd_motor_stop();
                this->_zlac->info.direct.set(this->_id, 0x00, 0x00, (uint8_t)value_l_mA, (uint8_t)value_r_mA);
                break;
        }
        if (true == flag_send) {
            value_l_mA = min(7500, max(value_l_mA, -7500));
            value_r_mA = min(7500, max(value_r_mA, -7500));
            this->_zlac->info.direct.set(this->_id, 0x08, 0x02, (uint8_t)value_l_mA, (uint8_t)value_r_mA);

            this->_zlac->cmd_torque_set(value_l_mA, value_r_mA);
            if (10 < abs(value_l_mA)) {
                this->_zlac->cmd_motor_start(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT);
            } else {
                this->_zlac->cmd_motor_stop(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT);
            }
            if (10 < abs(value_r_mA)) {
                this->_zlac->cmd_motor_start(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT);
            } else {
                this->_zlac->cmd_motor_stop(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT);
            }
        }
#else
        switch (buffer[0]) {
            case 'z':
                acceleration_ms += SPEED_SPAN_ACC_DCC;
                this->_zlac->cmd_speed_set_acc_and_dec(acceleration_ms, deceleration_ms);
                break;
            case 'x':
                acceleration_ms -= SPEED_SPAN_ACC_DCC;
                if (acceleration_ms < SPEED_SPAN_ACC_DCC) {
                    acceleration_ms = SPEED_SPAN_ACC_DCC;
                }
                this->_zlac->cmd_speed_set_acc_and_dec(acceleration_ms, deceleration_ms);
                break;
            case 'c':
                deceleration_ms += SPEED_SPAN_ACC_DCC;
                this->_zlac->cmd_speed_set_acc_and_dec(acceleration_ms, deceleration_ms);
                break;
            case 'v':
                deceleration_ms -= SPEED_SPAN_ACC_DCC;
                if (acceleration_ms < SPEED_SPAN_ACC_DCC) {
                    acceleration_ms = SPEED_SPAN_ACC_DCC;
                }
                this->_zlac->cmd_speed_set_acc_and_dec(acceleration_ms, deceleration_ms);
                break;
            case 'r':
                this->_zlac->cmd_clear_fault(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_ALL);
                break;
            case 'm':
                this->_zlac->cmd_looking_for_z_signal();
                break;
            case '0':
                id           = 0x80;
                command      = 0x5A;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                break;
            case '1':
                // roboclaw.ReadEncM1
                id           = 0x80;
                command      = 0x10;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                break;
            case '2':
                // roboclaw.ReadEncM2
                id           = 0x80;
                command      = 0x11;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                break;
            case '3':
                // roboclaw.ReadEncM1
                id           = 0x80;
                command      = 0x18;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                break;
            case '4':
                // roboclaw.ReadEncM2
                id           = 0x80;
                command      = 0x19;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                break;
            case '5':
                // roboclaw.ReadTemp
                id           = 0x80;
                command      = 0x52;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                break;
            case '6':
                // roboclaw.ReadTemp2
                id           = 0x80;
                command      = 0x53;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                break;
            case '7':
                // roboclaw.ReadVersion
                id           = 0x80;
                command      = 0x15;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                break;
                /////////////////////////////
            case 'o':
                left         = 0;
                right        = 0;
                id           = 0x80;
                command      = 0x25;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                // ==========================================================
                value[value_size++] = (right >> 24) & 0xFF;
                value[value_size++] = (right >> 16) & 0xFF;
                value[value_size++] = (right >> 8) & 0xFF;
                value[value_size++] = (right >> 0) & 0xFF;
                value[value_size++] = (left >> 24) & 0xFF;
                value[value_size++] = (left >> 16) & 0xFF;
                value[value_size++] = (left >> 8) & 0xFF;
                value[value_size++] = (left >> 0) & 0xFF;
                crc                 = id + command;
                for (int i = 0; i < value_size; i++) {
                    crc = value[i++];
                }
                value[value_size++] = (crc >> 8) & 0xFF;
                value[value_size++] = (crc >> 0) & 0xFF;
                // ==========================================================
                break;
            case 'w':
                left += SPEED_SPAN;
                right += SPEED_SPAN;
                id           = 0x80;
                command      = 0x25;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                // ==========================================================
                value[value_size++] = (right >> 24) & 0xFF;
                value[value_size++] = (right >> 16) & 0xFF;
                value[value_size++] = (right >> 8) & 0xFF;
                value[value_size++] = (right >> 0) & 0xFF;
                value[value_size++] = (left >> 24) & 0xFF;
                value[value_size++] = (left >> 16) & 0xFF;
                value[value_size++] = (left >> 8) & 0xFF;
                value[value_size++] = (left >> 0) & 0xFF;
                crc                 = id + command;
                for (int i = 0; i < value_size; i++) {
                    crc = value[i++];
                }
                value[value_size++] = (crc >> 8) & 0xFF;
                value[value_size++] = (crc >> 0) & 0xFF;
                // ==========================================================
                break;
            case 's':
                left -= SPEED_SPAN;
                right -= SPEED_SPAN;
                id           = 0x80;
                command      = 0x25;
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                // ==========================================================
                value[value_size++] = (right >> 24) & 0xFF;
                value[value_size++] = (right >> 16) & 0xFF;
                value[value_size++] = (right >> 8) & 0xFF;
                value[value_size++] = (right >> 0) & 0xFF;
                value[value_size++] = (left >> 24) & 0xFF;
                value[value_size++] = (left >> 16) & 0xFF;
                value[value_size++] = (left >> 8) & 0xFF;
                value[value_size++] = (left >> 0) & 0xFF;
                crc                 = id + command;
                for (int i = 0; i < value_size; i++) {
                    crc = value[i++];
                }
                value[value_size++] = (crc >> 8) & 0xFF;
                value[value_size++] = (crc >> 0) & 0xFF;
                // ==========================================================
                break;
                /////////////////////////////
            case 'q':
                left += SPEED_SPAN;
                id = 0x80;
                if (0 <= left) {
                    command = 0x00;
                } else {
                    command = 0x01;
                }
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                // ==========================================================
                value[value_size++] = abs(left) & 0xFF;
                crc                 = id + command;
                for (int i = 0; i < value_size; i++) {
                    crc = value[i++];
                }
                value[value_size++] = (crc >> 8) & 0xFF;
                value[value_size++] = (crc >> 0) & 0xFF;
                // ==========================================================
                break;
            case 'a':
                left -= SPEED_SPAN;
                id = 0x80;
                if (0 <= left) {
                    command = 0x00;
                } else {
                    command = 0x01;
                }
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                // ==========================================================
                value[value_size++] = abs(left) & 0xFF;
                crc                 = id + command;
                for (int i = 0; i < value_size; i++) {
                    crc = value[i++];
                }
                value[value_size++] = (crc >> 8) & 0xFF;
                value[value_size++] = (crc >> 0) & 0xFF;
                // ==========================================================
                break;
                /////////////////////////////
            case 'e':
                right += SPEED_SPAN;
                id = 0x80;
                if (0 <= left) {
                    command = 0x04;
                } else {
                    command = 0x05;
                }
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                // ==========================================================
                value[value_size++] = abs(right) & 0xFF;
                crc                 = id + command;
                for (int i = 0; i < value_size; i++) {
                    crc = value[i++];
                }
                value[value_size++] = (crc >> 8) & 0xFF;
                value[value_size++] = (crc >> 0) & 0xFF;
                // ==========================================================
                break;
            case 'd':
                right -= SPEED_SPAN;
                id = 0x80;
                if (0 <= left) {
                    command = 0x04;
                } else {
                    command = 0x05;
                }
                command_size = CmdRoboclawDataList[command].Size_send - 2;
                // ==========================================================
                value[value_size++] = abs(right) & 0xFF;
                crc                 = id + command;
                for (int i = 0; i < value_size; i++) {
                    crc = value[i++];
                }
                value[value_size++] = (crc >> 8) & 0xFF;
                value[value_size++] = (crc >> 0) & 0xFF;
                // ==========================================================
                break;
                /////////////////////////////
            case 'l':
                id                  = 0x80;
                command             = 0x14;
                command_size        = CmdRoboclawDataList[command].Size_send - 2;
                crc                 = id + command;
                value[value_size++] = (crc >> 8) & 0xFF;
                value[value_size++] = (crc >> 0) & 0xFF;
                break;
                /////////////////////////////
            case 'p':
                this->_zlac->cmd_get_all_status();
                //this->log_message("==============================");
                sprintf(mess, "startup_state  : %s / %s", this->_zlac->info.left.error.startup_state ? "on " : "off", this->_zlac->info.right.error.startup_state ? "on " : "off");
                this->log_message(mess);
                sprintf(mess, "stop_state     : %s / %s", this->_zlac->info.left.error.stop_state ? "on " : "off", this->_zlac->info.right.error.stop_state ? "on " : "off");
                this->log_message(mess);
                sprintf(mess, "over_current   : %s / %s", this->_zlac->info.left.error.over_current ? "on " : "off", this->_zlac->info.right.error.over_current ? "on " : "off");
                this->log_message(mess);
                sprintf(mess, "over_voltage   : %s / %s", this->_zlac->info.left.error.over_voltage ? "on " : "off", this->_zlac->info.right.error.over_voltage ? "on " : "off");
                this->log_message(mess);
                sprintf(mess, "encoder_error  : %s / %s", this->_zlac->info.left.error.encoder_error ? "on " : "off", this->_zlac->info.right.error.encoder_error ? "on " : "off");
                this->log_message(mess);
                sprintf(mess, "overheat       : %s / %s", this->_zlac->info.left.error.overheat ? "on " : "off", this->_zlac->info.right.error.overheat ? "on " : "off");
                this->log_message(mess);
                sprintf(mess, "under_voltage  : %s / %s", this->_zlac->info.left.error.under_voltage ? "on " : "off", this->_zlac->info.right.error.under_voltage ? "on " : "off");
                this->log_message(mess);
                sprintf(mess, "overload       : %s / %s", this->_zlac->info.left.error.overload ? "on " : "off", this->_zlac->info.right.error.overload ? "on " : "off");
                this->log_message(mess);
                sprintf(mess, "not_connection : %s / %s", this->_zlac->info.left.error.not_connection ? "on " : "off", this->_zlac->info.right.error.not_connection ? "on " : "off");
                this->log_message(mess);
                this->log_message("------------------------------");
                sprintf(mess, "voltage [%d V] [%d V]", this->_zlac->info.left.voltage, this->_zlac->info.right.voltage);
                this->log_message(mess);
                sprintf(mess, "current [%d mA] [%d mA]", this->_zlac->info.left.current, this->_zlac->info.right.current);
                this->log_message(mess);
                sprintf(mess, "speed [%d] [%d]", this->_zlac->info.left.speed_rpm, this->_zlac->info.right.speed_rpm);
                this->log_message(mess);
                sprintf(mess, "position_given [%d] [%d]", this->_zlac->info.left.position_given, this->_zlac->info.right.position_given);
                this->log_message(mess);
                sprintf(mess, "position_feedback [%d] [%d]", this->_zlac->info.left.position_feedback, this->_zlac->info.right.position_feedback);
                this->log_message(mess);
                this->log_message("==============================");
                break;
            default:
                break;
        }
#endif
        if (0 != id) {
            if (true == this->_check_id(id)) {
                this->_controller(command, command_size, value, value_size);
            }
        }
    }
#else
    uint8_t id         = 0;
    uint8_t command    = 0;
    int command_size   = 0;
    uint8_t value[100] = { 0 };
    size_t value_size  = 0;
    bool running       = false;
    int length         = 0;
    bool flag_start    = false;

    unsigned long current_ms = millis();
    if (2 <= this->_input_serial->available()) {
        this->call_time                   = current_ms + HEARTBEAT_INTERVAL;
        this->_zlac->info.flag.heart_beat = true;

        id = (uint8_t)this->_input_serial->read();
        if (true == this->_check_id(id)) {
            command      = (uint8_t)this->_input_serial->read();
            running      = true;
            command_size = CmdRoboclawDataList[command].Size_send - 2;
            if (0 < command_size) {
                if (true == this->_receive_wait(command_size)) {
                    delay(this->TIMEOUT_INPUT_SERIAL_MS);
                    value_size = this->_input_serial->readBytes(value, command_size);
                }
                running = this->_check_crc(id, command, value, command_size);
                if (false == running) {
                    delay(this->TIMEOUT_INPUT_SERIAL_MS);
                    command_size = this->_input_serial->available();
                    this->_input_serial->readBytes(value, command_size);
                }
            }
        }
    }

    if (true == running) {
        this->_controller(command, command_size, value, value_size);
    } else {
        if (current_ms > this->call_time) {
            if (true == this->_zlac->info.flag.heart_beat) {
                this->_set_speed(0, 0);
                this->_zlac->info.flag.heart_beat = false;
            }
        }
    }
#endif
}

void RoboClawForZlac::_controller(uint8_t command, size_t command_size, uint8_t value[100], size_t value_size)
{
    this->log_trace(__func__);
    uint8_t buffer[255];
    unsigned int crc = 0;
    crc              = this->_crc_update(crc, this->_id);
    crc              = this->_crc_update(crc, command);
    for (int i = 0; (i < value_size) && (i < 100); i++) {
        crc = this->_crc_update(crc, value[i]);
    }

    if (0 <= command_size) {
        switch (command) {
            case 0x15: // roboclaw.ReadVersion
                this->_zlac->info.direct.set(this->_id, command, value_size);
                this->_read_version(crc);
                break;
            case 0x25: // roboclaw.SpeedM1M2
                this->_zlac->info.direct.set(this->_id, command, value_size, value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7], value[8], value[9]);
                this->_speed_m1m2(crc, value, value_size);
                break;
            case 0x14: // roboclaw.ResetEncoders
                this->_zlac->info.direct.set(this->_id, command, value_size);
                this->_reset_encoders(crc, value, value_size);
                break;
            case 0x00: // roboclaw.ForwardM1
                this->_zlac->info.direct.set(this->_id, command, value_size, value[0], value[1], value[2], value[3]);
                this->_forward_m1(crc, value, value_size);
                break;
            case 0x01: // roboclaw.BackwardsM1
                this->_zlac->info.direct.set(this->_id, command, value_size, value[0], value[1], value[2], value[3]);
                this->_backwards_m1(crc, value, value_size);
                break;
            case 0x04: // roboclaw.ForwardM2
                this->_zlac->info.direct.set(this->_id, command, value_size, value[0], value[1], value[2], value[3]);
                this->_forward_m2(crc, value, value_size);
                break;
            case 0x05: // roboclaw.BackwardsM2
                this->_zlac->info.direct.set(this->_id, command, value_size, value[0], value[1], value[2], value[3]);
                this->_backwards_m2(crc, value, value_size);
                break;
            case 0x10: // roboclaw.ReadEncM1
                this->_zlac->info.direct.set(this->_id, command, value_size);
                this->_read_enc_m1(crc);
                break;
            case 0x11: // roboclaw.ReadEncM2
                this->_zlac->info.direct.set(this->_id, command, value_size);
                this->_read_enc_m2(crc);
                break;
            case 0x18: // roboclaw.ReadMainBatteryVoltage
                this->_zlac->info.direct.set(this->_id, command, value_size);
                this->_read_main_battery_voltage(crc);
                break;
            case 0x19: // roboclaw.ReadLogicBatteryVoltage
                this->_zlac->info.direct.set(this->_id, command, value_size);
                this->_read_logic_battery_voltage(crc);
                break;
            case 0x52: // roboclaw.ReadTemp
                this->_zlac->info.direct.set(this->_id, command, value_size);
                this->_read_temp(crc);
                break;
            case 0x53: // roboclaw.ReadTemp2
                this->_zlac->info.direct.set(this->_id, command, value_size);
                this->_read_temp2(crc);
                break;
            case 0x5A: // roboclaw.ReadError
                this->_zlac->info.direct.set(this->_id, command, value_size);
                this->_read_error(crc);
                break;
            default:
                this->_zlac->info.direct.set(0, command, value_size);
                break;
        }
    }
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// Setting
///////////////////////////////////////////////////////////////////
#pragma region setting

bool RoboClawForZlac::update_id(int id)
{
    bool result = false;
    if (0x80 <= id) {
        if (id <= 0x87) {
            this->_id = id;
            result    = true;
        }
    }
    return result;
}
void RoboClawForZlac::log_message_mode(bool enable)
{
    if (true == enable) {
        this->output_log_enable();
    } else {
        this->output_log_disable();
    }
#if DEBUG_ZLAC706_SERIAL
    this->output_log_level(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE);

    if (nullptr != this->_zlac) {
        this->_zlac->output_log_level(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE);
        if (true == enable) {
            this->_zlac->output_log_enable();
        } else {
            this->_zlac->output_log_disable();
        }
    }
#endif
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// public function
///////////////////////////////////////////////////////////////////
#pragma region public_function
bool RoboClawForZlac::reset()
{
    bool result = false;
    if (true == this->_flag_initialized) {
        this->_zlac->cmd_clear_fault();
        this->_zlac->cmd_modify_the_rated_current(ROBOCLAW_CURRENT_MODIFICATION);

        this->_zlac->cmd_position_set_absolute();
#if DEBUG_ZLAC706_SERIAL_MODE == DEBUG_ZLAC706_SERIAL_MODE_POSITION
        this->_zlac->cmd_position_set_relative();
        this->_zlac->cmd_position_mode();
#if ROBOCLAW_SETTING_SET_GAIN
        this->_zlac->cmd_setting_proportional_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT, this->_zlac->info.left.position_proportional_gain);
        this->_zlac->cmd_setting_differential_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT, this->_zlac->info.left.position_differential_gain);
        this->_zlac->cmd_setting_feed_forward_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT, this->_zlac->info.left.position_feed_forward_gain);

        this->_zlac->cmd_setting_proportional_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT, this->_zlac->info.right.position_proportional_gain);
        this->_zlac->cmd_setting_differential_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT, this->_zlac->info.right.position_differential_gain);
        this->_zlac->cmd_setting_feed_forward_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT, this->_zlac->info.right.position_feed_forward_gain);
#endif
#elif DEBUG_ZLAC706_SERIAL_MODE == DEBUG_ZLAC706_SERIAL_MODE_TORQUE
        this->_zlac->cmd_torque_mode();
#else
        this->_zlac->cmd_speed_mode();
#if ROBOCLAW_SETTING_SET_GAIN
        this->_zlac->cmd_setting_proportional_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT, this->_zlac->info.left.speed_proportional_gain);
        this->_zlac->cmd_setting_integral_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT, this->_zlac->info.left.speed_integral_gain);
        this->_zlac->cmd_setting_differential_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT, this->_zlac->info.left.speed_differential_gain);

        this->_zlac->cmd_setting_proportional_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT, this->_zlac->info.right.speed_proportional_gain);
        this->_zlac->cmd_setting_integral_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT, this->_zlac->info.right.speed_integral_gain);
        this->_zlac->cmd_setting_differential_gain(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT, this->_zlac->info.right.speed_differential_gain);
#endif
#endif
        // this->_zlac->cmd_mode_selection(ZLAC706Serial::DRIVER_MODE::SPEED_FROM_DIGITAL);

        this->_zlac->cmd_speed_set_acc_and_dec(this->_zlac->info.acceleration_ms, this->_zlac->info.deceleration_ms);
        this->_set_speed(0, 0);
        result = true;
    }
    this->_flag_setting = result;
    return result;
}
bool RoboClawForZlac::begin()
{
    this->_setting_load();

    this->reset();
    this->loop();

    return this->_flag_setting;
}
bool RoboClawForZlac::setup(HardwareSerial *input_serial, HardwareSerial *motor_driver_left, HardwareSerial *motor_driver_right, unsigned long input_baud)
{
    try {
        if (false == this->_flag_initialized) {
            this->_input_serial = input_serial;
            this->_input_serial->setTimeout(this->TIMEOUT_INPUT_SERIAL_MS);
            this->_input_serial->begin(input_baud);

            this->_zlac->setup_serial_driver(motor_driver_left, motor_driver_right);
            this->_zlac->begin();

            this->_set_log_level();

            this->_flag_initialized = true;
        }

    } catch (...) {
    }
    return this->_flag_initialized;
}

bool RoboClawForZlac::loop()
{
    static unsigned int TIME_INTERVAL_MS = (1000) * 5;
    static unsigned long next_time_ms    = 0;
    static int ignore_cnt                = 0;
    bool result                          = true;
    if (true == this->_flag_setting) {
        ///////////////////////////////////////
        this->_receive();
        result = this->_zlac->cmd_speed_heart_beat();
        ///////////////////////////////////////
        if (next_time_ms <= millis()) {
            next_time_ms = millis() + TIME_INTERVAL_MS;
            if (false == this->_zlac->is_connection()) {
                ignore_cnt++;
                if (2 < ignore_cnt) {
                    result = this->reset();
                    this->log_error("is not connection");
                }
            } else if (true == this->_zlac->is_error()) {
                this->_zlac->cmd_clear_fault();
                this->log_error("happened error");
            } else {
                ignore_cnt = 0;
            }
        }
    } else {
        if (next_time_ms <= millis()) {
            next_time_ms = millis() + TIME_INTERVAL_MS;
            result       = this->reset();
        } else {
            result = false;
        }
    }
    return !this->_zlac->is_error();
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// Callback
///////////////////////////////////////////////////////////////////
#pragma region callback
bool RoboClawForZlac::callback_message(MessageFunction callback)
{
    bool result = false;
#if DEBUG_ZLAC706_SERIAL
    if (nullptr != callback) {
        if (nullptr != this->_zlac) {
            (void)this->_zlac->set_callback_message(callback);
            result = true;
        }
    }
#else
    result = true;
#endif
    return result;
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// state
///////////////////////////////////////////////////////////////////
#pragma region state
RoboClawForZlac::roboclaw_state RoboClawForZlac::get_state()
{
    roboclaw_state state = STATE_RUNNING;
    if (true == this->_zlac->info.flag.emergency) {
        state = STATE_EMERGENCY;
    } else if (false == this->_flag_initialized) {
        state = STATE_NOT_INITIALIZED;
    } else if (false == this->_flag_setting) {
        state = STATE_NOT_SETTING;
    } else if (false == this->_zlac->is_connection()) {
        state = STATE_ERROR;
    } else if (true == this->_zlac->is_error_flag()) {
        state = STATE_ERROR;
    }

    return state;
}
ZLAC706Serial::zlac_info RoboClawForZlac::get_zlac_info()
{
    return this->_zlac->info;
}
void RoboClawForZlac::set_emergency(bool emergency)
{
    this->_zlac->info.flag.emergency = emergency;
    this->_zlac->info.system.set(ZLAC706Serial::SYSTEM_LOG::LOG_EMERGENCY, 0, ((true == emergency) ? 1 : 2));
    this->_zlac->cmd_motor_stop();
}
bool RoboClawForZlac::setting_proportional_gain(ZLAC706Serial::DRIVER_TARGET target, int value)
{
    return this->_zlac->cmd_setting_proportional_gain(target, value);
}
bool RoboClawForZlac::setting_integral_gain(ZLAC706Serial::DRIVER_TARGET target, int value)
{
    return this->_zlac->cmd_setting_integral_gain(target, value);
}
bool RoboClawForZlac::setting_differential_gain(ZLAC706Serial::DRIVER_TARGET target, int value)
{
    return this->_zlac->cmd_setting_differential_gain(target, value);
}
bool RoboClawForZlac::setting_feed_forward_gain(ZLAC706Serial::DRIVER_TARGET target, int value)
{
    return this->_zlac->cmd_setting_feed_forward_gain(target, value);
}
bool RoboClawForZlac::setting_inverted(ZLAC706Serial::DRIVER_TARGET target, bool value)
{
    return this->_zlac->cmd_setting_inverted(target, value);
}
bool RoboClawForZlac::setting_acc(ZLAC706Serial::DRIVER_TARGET target, int value)
{
    return this->_zlac->cmd_setting_acc(target, value);
}
bool RoboClawForZlac::setting_dcc(ZLAC706Serial::DRIVER_TARGET target, int value)
{
    return this->_zlac->cmd_setting_dcc(target, value);
}
bool RoboClawForZlac::setting_save()
{
    return this->_zlac->setting_save();
}
bool RoboClawForZlac::_setting_load()
{
    return this->_zlac->setting_load();
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// private function
///////////////////////////////////////////////////////////////////
#pragma region private_function
void RoboClawForZlac::_set_log_level()
{
#if DEBUG_ZLAC706_SERIAL
    OUTPUT_LOG_LEVEL level = OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_DEBUG;
    if (nullptr != this->_zlac) {
        this->_zlac->output_log_level(level);
    }
    this->output_log_level(level);
#else
    if (nullptr != this->_zlac) {
        this->_zlac->output_log_level(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_OFF);
        this->_zlac->output_log_disable();
    }
    this->output_log_level(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_OFF);
    this->output_log_disable();
#endif
}

bool RoboClawForZlac::_receive_wait(int size)
{
    bool result             = false;
    unsigned long loop_time = millis() + this->TIMEOUT_INPUT_SERIAL_MS;
    do {
        if (size <= this->_input_serial->available()) {
            result = true;
            break;
        } else {
            delay(1);
        }
    } while (millis() <= loop_time);
    return result;
}

bool RoboClawForZlac::_check_id(uint8_t id)
{
    bool result = false;
#if PASS_RANGE_ID
    if (0x80 <= id) {
        if (id <= 0x87) {
            result = true;
        }
    }
#else
    if (id == this->_id) {
        result = true;
    }
#endif
    if (false == result) {
        this->_zlac->cmd_clear_fault(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_ALL);
#if DEBUG_ZLAC706_SERIAL
        char buffer[255];
        sprintf(buffer, "Not id[0x%02X]", id);
        this->log_warn(buffer);
#else
        this->log_warn("Not id");
#endif
    }
    return result;
}
void RoboClawForZlac::_response(unsigned int crc, uint8_t data[100], int size, bool add_crc)
{
    if (true == add_crc) {
        uint8_t send_data[100] = { 0 };
        int length             = 0;

        for (int i = 0; (i < size) && (i < 100); i++) {
            send_data[i] = data[i];
            crc          = this->_crc_update(crc, data[i]);
            length++;
        }
        send_data[length++] = crc >> 8;
        send_data[length++] = crc & 0xFF;
#if DEBUG_ZLAC706_SERIAL
        std::string message = "response(CRC) :";
        char buffer[100];
        for (int i = 0; i < length; i++) {
            sprintf(buffer, " 0x%02X", send_data[i]);
            message.append(buffer);
        }
        this->log_trace(message);
#else
        this->_input_serial->write(send_data, length);
#endif
    } else {
#if DEBUG_ZLAC706_SERIAL
        std::string message = "response(---) :";
        char buffer[100];
        for (int i = 0; i < size; i++) {
            sprintf(buffer, " 0x%02X", data[i]);
            message.append(buffer);
        }
        this->log_trace(message);
#else
        this->_input_serial->write(data, size);
#endif
    }
}
unsigned int RoboClawForZlac::_crc_update(unsigned int crc, uint8_t data)
{
    crc ^= data << 8;
    for (int i = 0; i < 8; i++) {
        if ((crc & 0x8000) == 0x8000) {
            crc = ((crc << 1) ^ 0x1021);
        } else {
            crc <<= 1;
        }
    }

    return crc;
}

int RoboClawForZlac::_rpm_to_mps(int value)
{
    return (value * 60 * 1000) / (SETTING_SYSTEM_WHEEL_DIAMETER_MM_X_PI);
}

int RoboClawForZlac::_mps_to_rpm(int value)
{
    return (value * SETTING_SYSTEM_WHEEL_DIAMETER_MM_X_PI) / (60 * 1000);
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// Command
///////////////////////////////////////////////////////////////////
#pragma region command
#pragma region common function

int RoboClawForZlac::_get_sign(double num)
{
    return num >= 0 ? 1 : -1;
}

void RoboClawForZlac::_set_speed(int speed_left, int speed_right, bool enable_left, bool enable_right)
{
    log_trace(__func__);
    static int left  = 0;
    static int right = 0;

    if (true == this->_flag_initialized) {
        if (true == enable_left) {
            left = speed_left;
        }
        if (true == enable_right) {
            right = speed_right;
        }
#if DEBUG_ZLAC706_SERIAL
        char buffer[255];
        sprintf(buffer, "%s left[%d],right[%d]", __func__, left, right);
        log_debug(buffer);
#endif

        if ((1 >= abs(left)) && (1 >= abs(right))) {
            this->_zlac->cmd_speed_set(0, 0);
            this->_zlac->cmd_motor_stop();
        } else {
            this->_zlac->cmd_speed_set(this->_mps_to_rpm(left), this->_mps_to_rpm(right));
            this->_zlac->cmd_motor_start();
        }
    }
}
#pragma endregion

void RoboClawForZlac::_read_version(unsigned int crc)
{
    this->log_trace("ReadVersion");
    static String version_text = "RoboClaw for ZLAC";
    static const char *text    = version_text.c_str();
    uint8_t buffer[255];
    int length = 0;

    for (int j = 0; j < version_text.length(); j++) {
        buffer[length++] = (uint8_t)text[j];
    }
    buffer[length++] = 10;
    buffer[length++] = 0;

    this->_response(crc, buffer, length, true);
}
void RoboClawForZlac::_speed_m1m2(unsigned int crc, uint8_t value[100], size_t value_size)
{
    this->log_trace("SpeedM1M2");
    int left  = (int32_t)((value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3] << 0));
    int right = (int32_t)((value[4] << 24) | (value[5] << 16) | (value[6] << 8) | (value[7] << 0));

    this->_set_speed(left, right);
#if DEBUG_ZLAC706_SERIAL
#else
    this->_input_serial->write(0xFF);
#endif
}
void RoboClawForZlac::_forward_m1(unsigned int crc, uint8_t value[100], size_t value_size)
{
    this->log_trace("ForwardM1");
    int left = (int32_t)(value[0]);
    this->_set_speed(left, 0, true, false);
#if DEBUG_ZLAC706_SERIAL
#else
    this->_input_serial->write(0xFF);
#endif
}
void RoboClawForZlac::_forward_m2(unsigned int crc, uint8_t value[100], size_t value_size)
{
    this->log_trace("ForwardM2");
    int right = (int32_t)(value[0]);
    this->_set_speed(0, right, false, true);
#if DEBUG_ZLAC706_SERIAL
#else
    this->_input_serial->write(0xFF);
#endif
}
void RoboClawForZlac::_backwards_m1(unsigned int crc, uint8_t value[100], size_t value_size)
{
    this->log_trace("BackwardsM1");
    int left = (int32_t)(value[0]);
    this->_set_speed(-left, 0, true, false);
#if DEBUG_ZLAC706_SERIAL
#else
    this->_input_serial->write(0xFF);
#endif
}
void RoboClawForZlac::_backwards_m2(unsigned int crc, uint8_t value[100], size_t value_size)
{
    this->log_trace("BackwardsM2");
    int right = (int32_t)(value[0]);
    this->_set_speed(0, -right, false, true);
#if DEBUG_ZLAC706_SERIAL
#else
    this->_input_serial->write(0xFF);
#endif
}
void RoboClawForZlac::_read_enc_m1(unsigned int crc)
{
    this->log_trace("ReadEncM1");
    uint8_t buffer[255] = { 0 };
    int length          = 0;
    int state           = 0; // TODO
    this->_zlac->cmd_get_position_feedback(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT);

    long enc_count   = (this->_zlac->info.left.position_feedback - this->_enc_postion) * ROBOCLAW_FEED_BACK_TIMES;
    buffer[length++] = (enc_count >> 24) & 0xFF;
    buffer[length++] = (enc_count >> 16) & 0xFF;
    buffer[length++] = (enc_count >> 8) & 0xFF;
    buffer[length++] = (enc_count >> 0) & 0xFF;
    buffer[length++] = (state >> 0) & 0xFF;

    /*
    0 to 4,294,967,295
    Bit0 - Counter Underflow (1= Underflow Occurred, Clear After Reading)
    Bit1 - Direction (0 = Forward, 1 = Backwards)
    Bit2 - Counter Overflow (1= Underflow Occurred, Clear After Reading)
    Bit3 - Reserved
    Bit4 - Reserved
    Bit5 - Reserved
    Bit6 - Reserved
    Bit7 - Reserved
    */

    this->_response(crc, buffer, length, true);
}
void RoboClawForZlac::_read_enc_m2(unsigned int crc)
{
    this->log_trace("ReadEncM2");
    uint8_t buffer[255]  = { 0 };
    int length           = 0;
    static long previous = 0;
    int state            = 0; // TODO
    this->_zlac->cmd_get_position_feedback(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT);

    long enc_count = (this->_zlac->info.right.position_feedback + this->_enc_difference);
    enc_count *= ROBOCLAW_FEED_BACK_TIMES;

    buffer[length++] = (enc_count >> 24) & 0xFF;
    buffer[length++] = (enc_count >> 16) & 0xFF;
    buffer[length++] = (enc_count >> 8) & 0xFF;
    buffer[length++] = (enc_count >> 0) & 0xFF;
    buffer[length++] = (state >> 0) & 0xFF;

    /*
    0 to 4,294,967,295

    Bit0 - Counter Underflow (1= Underflow Occurred, Clear After Reading)
    Bit1 - Direction (0 = Forward, 1 = Backwards)
    Bit2 - Counter Overflow (1= Underflow Occurred, Clear After Reading)
    Bit3 - Reserved
    Bit4 - Reserved
    Bit5 - Reserved
    Bit6 - Reserved
    Bit7 - Reserved
*/
    this->_response(crc, buffer, length, true);
}
void RoboClawForZlac::_reset_encoders(unsigned int crc, uint8_t value[100], size_t value_size)
{
    this->log_trace("ResetEncoders");
    this->_zlac->cmd_get_position_feedback(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT);
    this->_zlac->cmd_get_position_feedback(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT);

    this->_enc_postion    = this->_zlac->info.left.position_feedback;
    this->_enc_difference = (this->_zlac->info.left.position_feedback - this->_zlac->info.right.position_feedback);
#if DEBUG_ZLAC706_SERIAL
#else
    this->_input_serial->write(0xFF);
#endif
}
void RoboClawForZlac::_read_main_battery_voltage(unsigned int crc)
{
    this->log_trace("ReadMainBatteryVoltage");
    uint8_t buffer[255] = { 0 };
    int length          = 0;
    int voltage         = 0;

    this->_zlac->cmd_get_bus_voltage(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_ALL);

    if (this->_zlac->info.left.voltage > this->_zlac->info.right.voltage) {
        voltage = this->_zlac->info.right.voltage;
    } else {
        voltage = this->_zlac->info.left.voltage;
    }

    buffer[length++] = (voltage >> 8) & 0xFF;
    buffer[length++] = (voltage >> 0) & 0xFF;

    this->_response(crc, buffer, length, true);
}
void RoboClawForZlac::_read_logic_battery_voltage(unsigned int crc)
{
    this->log_trace("ReadLogicBatteryVoltage");
    uint8_t buffer[255] = { 0 };
    int length          = 0;
    int voltage         = 0;

    this->_zlac->cmd_get_bus_voltage(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT);
    this->_zlac->cmd_get_bus_voltage(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT);

    if (this->_zlac->info.left.voltage > this->_zlac->info.right.voltage) {
        voltage = this->_zlac->info.right.voltage;
    } else {
        voltage = this->_zlac->info.left.voltage;
    }

    buffer[length++] = (voltage >> 8) & 0xFF;
    buffer[length++] = (voltage >> 0) & 0xFF;

    this->_response(crc, buffer, length, true);
}
void RoboClawForZlac::_read_temp(unsigned int crc)
{
    this->log_trace("ReadTemp");
    uint8_t buffer[255]       = { 0 };
    int length                = 0;
    static unsigned long temp = 20;

    buffer[length++] = (temp >> 8) & 0xFF;
    buffer[length++] = (temp >> 0) & 0xFF;

    this->_response(crc, buffer, length, true);
}
void RoboClawForZlac::_read_temp2(unsigned int crc)
{
    this->log_trace("ReadTemp2");
    uint8_t buffer[255]       = { 0 };
    int length                = 0;
    static unsigned long temp = 20;

    buffer[length++] = (temp >> 8) & 0xFF;
    buffer[length++] = (temp >> 0) & 0xFF;

    this->_response(crc, buffer, length, true);
}

void RoboClawForZlac::_read_error(unsigned int crc)
{
    this->log_trace("ReadError");
    uint8_t buffer[255] = { 0 };
    int length          = 0;
    this->_zlac->cmd_get_alarm_status();

    static unsigned long state = 0; //Normal : 0x000000
    if ((true == this->_zlac->info.left.error.stop_state) || (true == this->_zlac->info.right.error.stop_state)) {
        // E-Stop : 0x000001
        state |= 0x000001;
    }
    if ((true == this->_zlac->info.left.error.over_voltage) || (true == this->_zlac->info.right.error.over_voltage)) {
        // Main Voltage High Error : 0x000008
        state |= 0x000008;
    }
    if ((true == this->_zlac->info.left.error.over_current) || (true == this->_zlac->info.right.error.over_current)) {
        // Logic Voltage High Error : 0x000010
        state |= 0x000010;
    }
    // Main Voltage High Warning : 0x040000
    if ((true == this->_zlac->info.left.error.under_voltage) || (true == this->_zlac->info.right.error.under_voltage)) {
        // Logic Voltage Low Error : 0x000020
        state |= 0x000020;
    }
    // Main Voltage Low Warning : 0x080000
    if (true == this->_zlac->info.left.error.overload) {
        // M1 Speed Error : 0x000100
        state |= 0x000100;
    }
    if (true == this->_zlac->info.right.error.overload) {
        // M2 Speed Error : 0x000200
        state |= 0x000200;
    }

    // Temperature Error : 0x000002
    // Temperature 2 Error : 0x000004
    // M1 Driver Fault Error : 0x000040
    // M2 Driver Fault Error : 0x000080
    // M1 Position Error : 0x000400
    // M2 Position Error : 0x000800
    // M1 Current Error : 0x001000
    // M2 Current Error : 0x002000
    // M1 Over Current Warning : 0x010000
    // M2 Over Current Warning : 0x020000
    // Temperature Warning : 0x100000
    // Temperature 2 Warning : 0x200000
    // S4 Signal Triggered : 0x400000
    // S5 Signal Triggered : 0x800000
    // Speed Error Limit Warning : 0x01000000
    // Position Error Limit Warning : 0x02000000

    buffer[length++] = (state >> 24) & 0xFF;
    buffer[length++] = (state >> 16) & 0xFF;
    buffer[length++] = (state >> 8) & 0xFF;
    buffer[length++] = (state >> 0) & 0xFF;

    this->_response(crc, buffer, length, true);
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// constructor
///////////////////////////////////////////////////////////////////
#pragma region constructor
RoboClawForZlac::RoboClawForZlac()
{
    this->_zlac = new ZLAC706Serial();
}
#pragma endregion
