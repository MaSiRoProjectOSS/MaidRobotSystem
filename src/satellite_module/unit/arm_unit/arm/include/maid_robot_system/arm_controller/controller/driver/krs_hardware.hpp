/**
 * @file krs_hardware.hpp
 * @author Claude (claude.masiro@gmail.com)
 * @brief Set up KRS hardware serial class
 * @version 0.1
 * @date 2023-05-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_KRS_HARDWARE_HPP
#define ARM_CONTROLLER_KRS_HARDWARE_HPP

#include "maid_robot_system/arm_controller/config.hpp"
#include "maid_robot_system/common/move_average.hpp"
#include "maid_robot_system/common/time_check.hpp"

#include <Arduino.h>
#include <IcsHardSerialClass.h>

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Set up KRS hardware serial class
 *
 */
class KRSHardware {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    /**
     * @brief Construct a new KRSHardware object
     *
     */
    KRSHardware();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief Get the krs object address
     *
     * @param krs object address
     */
    void get_krs_address(IcsHardSerialClass *krs);

    /**
     * @brief set up KRS
     *
     */
    void KRS_setup();

private:
    HardwareSerial *_krs_serial; /*!<  */
    IcsHardSerialClass *_krs;    /*!<  */

    const int _MOVE_TIME = 130; /*!<  */

    const int _TIMEOUT = 10; /*!< For checking whether the communication is active or not. */

    const long _BAUDRATE = 1250000;
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
