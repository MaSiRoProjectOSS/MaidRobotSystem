/**
 * @file model_on_arm_controller.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Model arm controller.
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/model_on_arm_controller.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

ModelOnArmController::ModelOnArmController()
{
}

bool ModelOnArmController::_setup()
{
    bool result = true;

#ifdef CIRO
    this->_active_arm  = &this->_L_arm;
    this->_passive_arm = &this->_R_arm;
#endif
#ifdef CIYA
    this->_active_arm  = &this->_R_arm;
    this->_passive_arm = &this->_L_arm;
#endif

    randomSeed(analogRead(6));
    this->_krs_hardware.KRS_setup();
    this->_com_ros.ROS_setup();
    this->_R_arm.setup(R_ARM_ID, R_ARM_INI_POS, R_ARM_SCALE);
    this->_L_arm.setup(L_ARM_ID, L_ARM_INI_POS, L_ARM_SCALE);

    this->_sensor_arm.begin();

    this->_new_hand.begin();

    this->_posture.begin();
    return result;
}

bool ModelOnArmController::_receive(ModeList mode)
{
    bool result = true;
    PostureManagerArguments *args;

    this->_posture.update_vital();

    args = this->_posture.get_posture_args_address();

    this->_sensor_shoulder_operator.sensor_shoulder(args);

    this->_com_ros.ROS_check(args);

    this->_sensor_arm.set_posture_address(args);
    this->_sensor_arm.calculate();

    float external_deg[JOINT_NUM] = { this->_sensor_arm.get_each_joint_angle(0),
                                      this->_sensor_arm.get_each_joint_angle(1),
                                      this->_sensor_arm.get_each_joint_angle(2),
                                      this->_sensor_arm.get_each_joint_angle(3),
                                      0,
                                      0,
                                      0,
                                      0 };
    this->_passive_arm->set_external_deg(external_deg);

    return result;
}

bool ModelOnArmController::_calculate(ModeList mode)
{
    bool result = true;
    PostureManagerArguments *args;

    this->_posture.calculate_first();

    args = this->_posture.get_posture_args_address();

    this->_R_arm.set_posture_address(args);
    this->_R_arm.calculate();

    this->_L_arm.set_posture_address(args);
    this->_L_arm.calculate();

    if (true == this->_posture.calculate()) {
        this->_posture.pose_manager(this->_active_arm, this->_passive_arm, &this->_R_arm, &this->_L_arm, &this->_new_hand);
    }
    return result;
}

void ModelOnArmController::_send(ModeList mode)
{
    PostureManagerArguments *args;
    args = this->_posture.get_posture_args_address();

    this->_R_arm.send(args);
    this->_L_arm.send(args);
    this->_com_ros.ROS_send(args);
}

void ModelOnArmController::_error_check(ModeList mode)
{
}

void ModelOnArmController::_debug_output(ModeList mode)
{
}

} // namespace arm_unit
} // namespace maid_robot_system
