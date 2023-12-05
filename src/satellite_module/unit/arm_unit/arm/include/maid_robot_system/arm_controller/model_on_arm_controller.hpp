/**
 * @file model_on_arm_controller.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Model arm controller.
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_MODEL_ON_ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_MODEL_ON_ARM_CONTROLLER_HPP

#include "maid_robot_system/arm_controller/communication_ros.hpp"
#include "maid_robot_system/arm_controller/config.hpp"
#include "maid_robot_system/arm_controller/controller/ctrl_hand.hpp"
#include "maid_robot_system/arm_controller/controller/movable_arm.hpp"
#include "maid_robot_system/arm_controller/controller/sensor_arm.hpp"
#include "maid_robot_system/arm_controller/controller/sensor_shoulder.hpp"
#include "maid_robot_system/arm_controller/manager/posture_manager.hpp"
#include "maid_robot_system/common/interface/model_if.hpp"
#include "maid_robot_system/common/time_check.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Model arm controller
 *
 */
class ModelOnArmController : public ModelIf {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    ModelOnArmController();

protected:
    /*********************************************************
     * Protected function
     *********************************************************/
    /**
     * @brief model setup
     */
    bool _setup() override;
    /**
     * @brief Receiving Data
     */
    bool _receive(ModeList mode) override;
    /**
     * @brief operate on data
     */
    bool _calculate(ModeList mode) override;
    /**
     * @brief Transmitting data
     */
    void _send(ModeList mode) override;
    /**
     * @brief error check
     */
    void _error_check(ModeList mode) override;
    /**
     * @brief to debug output
     */
    void _debug_output(ModeList mode) override;

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    MovableArm *_active_arm;  /*!< active arm */
    MovableArm *_passive_arm; /*!< passive arm */
    SensorArm _sensor_arm;    /*!< sensor arm */
    MovableArm _R_arm;        /*!< right arm */
    MovableArm _L_arm;        /*!< left arm */
    CtrlHand _new_hand;       /*!< new hand */

    PostureManager _posture;                  /*!< posture manager data object */
    SensorShoulder _sensor_shoulder_operator; /*!< for calculate sensor shoulder */

    CommunicationROS _com_ros; /*!< ROS communication */

    KRSHardware _krs_hardware; /*!< KRS hardware */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
