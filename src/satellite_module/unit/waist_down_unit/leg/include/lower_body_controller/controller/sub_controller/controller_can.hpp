/**
 * @file controller_can.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Analyze CAN data
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef LOWER_BODY_UNIT_SUB_CONTROLLER_CAN_HPP
#define LOWER_BODY_UNIT_SUB_CONTROLLER_CAN_HPP

#include "lower_body_controller/controller/control_leg_motor.hpp"
#include "lower_body_controller/controller/control_waist.hpp"

#include <mcp_can.h>

namespace lower_body_unit
{
namespace controller
{

/**
 * @brief Analyze CAN data
 *
 */
class ControllerCAN {
public:
    ControllerCAN(uint8_t can_cs = __UINT8_MAX__);

public:
    bool setup();
    bool send(ControlLegMotor *leg_motor, ControlWaist *waist);
    bool receive(ControlLegMotor *leg_motor, ControlWaist *waist, CoordinateEuler *gyro);
    bool is_running();

private:
    MCP_CAN *_can;
    bool _flag_initialized = false; /*!<  */
    bool _flag_running     = false; /*!<  */

private:
    const unsigned int CAN_ID_RECEIVE_MOTOR_POWER    = 0x310;
    const unsigned int CAN_ID_RECEIVE_LEG_TARGET     = 0x311;
    const unsigned int CAN_ID_RECEIVE_WAIST          = 0x312;
    const unsigned int CAN_ID_RECEIVE_WHEEL          = 0x315;
    const unsigned int CAN_ID_RECEIVE_LEG_SMART_MOVE = 0x333;

    const unsigned int CAN_ID_SEND_GYRO        = 0x321;
    const unsigned int CAN_ID_SEND_LEG_POSTION = 0x322;

private:
    const int SYSTEM_TIMEOUT_MS = 2500;

    // TODO : 一度に全部送っていたのを、等間隔で送るように変更
    /*
    データの送り方を10ms集中型から分散型に変更
    ____--________--________--________--___
    ____-____-____-____-____-____-____-____

    変更理由：CANの仕様上、ぶつかると遅延する。
            集中型は、こちらの送信機器、相手側のバッファサイズなどを設計に考慮しないといけないため、
    */
    const int SEND_INTERVAL_MS = 10;
};

} // namespace controller
} // namespace lower_body_unit

#endif
