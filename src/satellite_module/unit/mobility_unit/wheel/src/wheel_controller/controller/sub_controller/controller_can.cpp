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
#include "wheel_controller/controller/sub_controller/controller_can.hpp"

namespace mobility_unit
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

bool ControllerCAN::receive(ControlWheel *wheel)
{
    static TimeCheck motor_power_timer;
    bool result = false;

    if (true == this->_flag_initialized) {
        long unsigned int rec_id;
        unsigned char rec_dlen;
        unsigned char rec_data[8];
        while (CAN_MSGAVAIL == this->_can->checkReceive()) {
            this->_can->readMsgBuf(&rec_id, &rec_dlen, rec_data);

            if (this->CAN_ID_RECEIVE_WHEEL == rec_id) {
                if (5 == rec_dlen) {
                    wheel->set_data(rec_data);
                }

            } else if (this->CAN_ID_RECEIVE_MOTOR_POWER == rec_id) {
                if (2 == rec_dlen) {
                    bool flag_monitoring_reset = false;

                    if (1 == rec_data[0]) {
                        flag_monitoring_reset = true;
                        wheel->enable();
                    } else {
                        wheel->disable();
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

    // =======================================
    // Stop if you can't confirm receipt
    // =======================================
    if (true == motor_power_timer.check_passing(this->SYSTEM_TIMEOUT_MS)) {
        wheel->disable();
        this->_flag_running = false;
    }
    return result;
}

bool ControllerCAN::send(ControlWheel *wheel, CoordinateEuler gyro)
{
    static int can_send_number = 0;
    static TimeCheck can_util_send_timer;
    bool result = false;

    if (true == this->_flag_initialized) {
        if (true == can_util_send_timer.check_passing(this->SEND_INTERVAL_MS)) {
            unsigned char send_data[8] = { 0 };
            int32_t send_id            = 0;
            int8_t send_len            = 0;

            if (can_send_number == 0) {
                send_id  = this->CAN_ID_SEND_GYRO;
                send_len = 6;

                int16_t send_gyro_y = gyro.yaw * 10.0;
                int16_t send_gyro_p = gyro.pitch * 10.0;
                int16_t send_gyro_r = gyro.roll * 10.0;

                send_data[0] = ((send_gyro_y >> 8)) & 0x00ff;
                send_data[1] = send_gyro_y & 0x00ff;
                send_data[2] = ((send_gyro_p >> 8)) & 0x00ff;
                send_data[3] = send_gyro_p & 0x00ff;
                send_data[4] = ((send_gyro_r >> 8)) & 0x00ff;
                send_data[5] = send_gyro_r & 0x00ff;

                result = true;
            } else if (1 == can_send_number) {
                send_id  = this->CAN_ID_SEND_LEG_POSTION;
                send_len = 6;

                int16_t leg_now   = wheel->step_height;
                int16_t pitch_now = 0;
                int16_t step_now  = wheel->get_step_percentage();

                send_data[0] = ((leg_now >> 8)) & 0x00ff;
                send_data[1] = leg_now & 0x00ff;
                send_data[2] = ((pitch_now >> 8)) & 0x00ff;
                send_data[3] = pitch_now & 0x00ff;
                send_data[4] = ((step_now >> 8)) & 0x00ff;
                send_data[5] = step_now & 0x00ff;

                result = true;
            } else {
                can_send_number = -1;
            }
            if (true == result) {
                this->_can->sendMsgBuf(send_id, send_len, send_data);
            }

            can_send_number++;
            if (can_send_number <= 2) {
                can_send_number = 0;
            }
        }
    }
    return result;
}

} // namespace controller
} // namespace mobility_unit
