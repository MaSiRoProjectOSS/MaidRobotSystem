/**
 * @file sensor_arm.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Control sensor arm.
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/controller/sensor_arm.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

SensorArm::SensorArm()
{
    this->_begin();
}

void SensorArm::get_deg()
{
    joint[0].read();
    joint[1].read();
    joint[2].read();
    joint[3].read();
}

void SensorArm::_kinetic_calc(PostureManagerArguments *args)
{
    /* Axial rotation torsion of the link [deg] */
    float link_twist[JOINT_NUM] = { -joint[0].get_degree() + (float)this->_SEMI_CIRCLE_DEGREE,
                                    0,
                                    -joint[2].get_degree() - (float)this->_QUARTER_CIRCLE_DEGREE,
                                    (float)this->_QUARTER_CIRCLE_DEGREE,
                                    joint[4].get_degree(),
                                    0,
                                    0,
                                    0 };
    /* Rotation of the link. */
    float link_sita[JOINT_NUM] = { (float)this->_QUARTER_CIRCLE_DEGREE, -joint[1].get_degree(), 0, joint[3].get_degree(), 0, 0, 0, 0 };

    Matrix<DIMENSION_OF_HOMO_MATRIX> T_m[JOINT_NUM];

    for (int c = 0; c < JOINT_NUM; c++) {
        T_m[c].trans_matrix(link_arm[c], link_twist[c], 0, link_sita[c]);
    }

    Matrix<DIMENSION_OF_HOMO_MATRIX> T_st;
    Matrix<DIMENSION_OF_HOMO_MATRIX> T_cal;
    Vector<DIMENSION_OF_HOMO_MATRIX> arm_root_point;

    float root_point[DIMENSION_OF_HOMO_MATRIX] = { 0, 0, 0, 1 };
    arm_root_point.set_elements(root_point);
    T_st = T_m[0];

    hand_point = this->tensor_operator.Mat_vec_product(arm_root_point, T_st);

    for (int c = 1; c < JOINT_NUM; c++) {
        T_cal.Matrix_product(T_st, T_m[c]);
        T_st       = T_cal;
        hand_point = this->tensor_operator.Mat_vec_product(arm_root_point, T_st);
    }

    hand_point                     = this->tensor_operator.Mat_vec_product(arm_root_point, T_st);
    hand_y                         = hand_point.data[0] * this->_HAND_SCALING_FACTOR;
    args->states.sensor_arm_hand_z = hand_point.data[1] * this->_HAND_SCALING_FACTOR;
    hand_x                         = hand_point.data[2] * this->_HAND_SCALING_FACTOR;
    hand_sita                      = atan2(hand_y, hand_x) / PI * this->_SEMI_CIRCLE_DEGREE;
    hand_r                         = sqrt(hand_x * hand_x + hand_y * hand_y);
}

void SensorArm::set_posture_address(PostureManagerArguments *args)
{
    this->_args = args;
}

bool SensorArm::_begin()
{
    joint[0].setup(SENSOR_ARM_INIT_POS[0], SENSOR_ARM_DIRECTION[0], PIN_SENSOR_ARM_0);
    joint[1].setup(SENSOR_ARM_INIT_POS[1], SENSOR_ARM_DIRECTION[1], PIN_SENSOR_ARM_1);
    joint[2].setup(SENSOR_ARM_INIT_POS[2], SENSOR_ARM_DIRECTION[2], PIN_SENSOR_ARM_2);
    joint[3].setup(SENSOR_ARM_INIT_POS[3], SENSOR_ARM_DIRECTION[3], PIN_SENSOR_ARM_3);

    return true;
}

bool SensorArm::_end()
{
    return true;
}

bool SensorArm::_calculate()
{
    this->get_deg();
    this->_kinetic_calc(this->_args);

    /* Mode management. */
    if (state == FREE) {
        if (this->_args->states.sensor_arm_hand_z > -250) {
            state = HANDSHAKE;
        }
    }
    if (state == HANDSHAKE) {
        if (this->_args->states.sensor_arm_hand_z < -200) {
            state = FREE;
        }
    }

    return true;
}

float SensorArm::get_each_joint_angle(int joint_num)
{
    return this->joint[joint_num].get_degree();
}

} // namespace arm_unit
} // namespace maid_robot_system
