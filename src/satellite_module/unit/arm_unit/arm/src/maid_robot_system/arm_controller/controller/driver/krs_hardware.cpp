/**
 * @file krs_hardware.cpp
 * @author Claude (claude.masiro@gmail.com)
 * @brief Set up KRS hardware serial class
 * @version 0.1
 * @date 2023-05-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/controller/driver/krs_hardware.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace arm_unit
{

KRSHardware::KRSHardware()
{
    static HardwareSerial _KRS_SERIAL(PIN_KRS_RX, PIN_KRS_TX); /* Serial1 RX/TX */
    this->_krs_serial = &_KRS_SERIAL;

    static IcsHardSerialClass krs(&_KRS_SERIAL, PIN_KRS_EN, this->_BAUDRATE, this->_TIMEOUT); /* create instance and set EN pin (number 2) and specify UART */
    this->_krs = &krs;
}

void KRSHardware::KRS_setup()
{
    this->_krs->begin(); /* initial set up of servo motor communication */
}

void KRSHardware::get_krs_address(IcsHardSerialClass *krs_pointer)
{
    krs_pointer = this->_krs;
}

} // namespace arm_unit
} // namespace maid_robot_system
