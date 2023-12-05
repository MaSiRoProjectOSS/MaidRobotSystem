/**
 * @file controller_can.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Analyze CAN data
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "lower_body_controller/controller/sub_controller/controller_can.hpp"

namespace lower_body_unit
{
namespace controller
{

ControllerCAN::ControllerCAN(uint8_t can_cs)
{
    this->_flag_initialized = false;
    this->_flag_running     = false;
    if (NUM_DIGITAL_PINS < can_cs) {
        this->_can = new MCP_CAN(can_cs);
    }
}

bool ControllerCAN::is_running()
{
    return this->_flag_running && this->_flag_initialized;
}

bool ControllerCAN::setup()
{
    if (false == this->_flag_initialized) {
        if (NULL != this->_can) {
            if (CAN_OK == this->_can->begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ)) {
                this->_can->setMode(MCP_NORMAL);

                while (CAN_MSGAVAIL == this->_can->checkReceive()) {
                    // clear buffer
                    long unsigned int rec_id;
                    unsigned char rec_dlen;
                    unsigned char rec_data[8];
                    if (CAN_OK == this->_can->readMsgBuf(&rec_id, &rec_dlen, rec_data)) {
                        // do nothing
                    }
                }

                this->_flag_initialized = true;
            }
        }
    }
    return this->_flag_initialized;
}

bool ControllerCAN::receive(ControlLegMotor *leg_motor, ControlWaist *waist, CoordinateEuler *gyro)
{
    static TimeCheck motor_power_timer;
    bool result = false;

    if (true == this->_flag_initialized) {
        long unsigned int rec_id;
        unsigned char rec_dlen;
        unsigned char rec_data[8];
        while (CAN_MSGAVAIL == this->_can->checkReceive()) {
            this->_can->readMsgBuf(&rec_id, &rec_dlen, rec_data);

            if (this->CAN_ID_RECEIVE_LEG_TARGET == rec_id) {
                int16_t get_target_pos = ((rec_data[0] << 8) & 0xff00) | (rec_data[1] & 0x00ff);
                leg_motor->set_data(false, get_target_pos, 0);

            } else if (this->CAN_ID_RECEIVE_LEG_SMART_MOVE == rec_id) {
                int16_t get_target_pos   = ((rec_data[0] << 8) & 0xff00) | (rec_data[1] & 0x00ff);
                int16_t get_target_speed = ((rec_data[2] << 8) & 0xff00) | (rec_data[3] & 0x00ff);
                leg_motor->set_data(true, get_target_pos, get_target_speed);

            } else if (this->CAN_ID_RECEIVE_WAIST == rec_id) {
                waist->set_data_from_CAN(rec_data[0] * 4);

            } else if (this->CAN_ID_RECEIVE_MOTOR_POWER == rec_id) {
                if (2 == rec_dlen) {
                    bool flag_monitoring_reset = false;

                    if (1 == rec_data[1]) {
                        flag_monitoring_reset = true;
                        if (true == leg_motor->is_alive()) {
                            leg_motor->enable();
                            waist->enable();
                        }
                    } else {
                        leg_motor->disable();
                        waist->disable();
                    }
                    if (true == flag_monitoring_reset) {
                        // TODO : enableの時だけ、延長？
                        motor_power_timer.update();
                        if (false == this->_flag_running) {
                            this->_flag_running = true;
                        }
                    }
                }
            }
        }
    }

    // TODO :
    float step_height = 0;
    leg_motor->set_data(step_height);

    // =======================================
    // Stop if you can't confirm receipt
    // =======================================
    if (true == motor_power_timer.check_passing(this->SYSTEM_TIMEOUT_MS)) {
        leg_motor->disable();
        waist->disable();
        this->_flag_running = false;
    }
    return result;
}

bool ControllerCAN::send(ControlLegMotor *leg_motor, ControlWaist *waist)
{
    static TimeCheck can_util_send_timer;
    bool result = false;

    if (true == this->_flag_initialized) {
        if (true == can_util_send_timer.check_passing(this->SEND_INTERVAL_MS)) {
            unsigned char send_data[8] = { 0 };
            int32_t send_id            = 0;
            int8_t send_len            = 0;

            send_id  = this->CAN_ID_SEND_LEG_POSTION;
            send_len = 6;

            int16_t leg_now         = leg_motor->get_height_percentage();
            int16_t waist_pitch_now = waist->get_target_pitch() * 100.0;

            send_data[0] = ((leg_now >> 8)) & 0x00ff;
            send_data[1] = leg_now & 0x00ff;
            send_data[2] = ((waist_pitch_now >> 8)) & 0x00ff;
            send_data[3] = waist_pitch_now & 0x00ff;

            this->_can->sendMsgBuf(send_id, send_len, send_data);
            result = true;
        }
    }
    return result;
}

} // namespace controller
} // namespace lower_body_unit
