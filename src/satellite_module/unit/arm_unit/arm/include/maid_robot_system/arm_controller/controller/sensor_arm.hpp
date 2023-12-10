/**
 * @file sensor_arm.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Control sensor arm.
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_SENSOR_ARM_HPP
#define ARM_CONTROLLER_SENSOR_ARM_HPP

#define SENSOR_ARM_DRIVER_POTENTIOMETER_NUM (5)

#include "maid_robot_system/arm_controller/config.hpp"
#include "maid_robot_system/arm_controller/controller/driver/driver_potentiometer.hpp"
#include "maid_robot_system/arm_controller/manager/posture_manager_arguments.hpp"
#include "maid_robot_system/common/interface/control_if.hpp"
#include "maid_robot_system/common/move_average.hpp"
#include "maid_robot_system/math/matrix.hpp"


namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Control sensor arm.
 *
 */
class SensorArm : public ControlIF {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    /**
     * @brief Construct a new Sensor Arm object
     *
     */
    SensorArm();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief read current joint angle
     *
     */
    void get_deg();

    /**
     * @brief Set the posture address object
     *
     * @param PostureManagerArguments address
     */
    void set_posture_address(PostureManagerArguments *args);

    /**
     * @brief Get the each joint angle object
     *
     * @param joint_num : number of joint
     * @return float : joint angle
     */
    float get_each_joint_angle(int joint_num);

protected:
    /*********************************************************
     * Inherited function
     *********************************************************/
    bool _begin() override;
    bool _end() override;
    bool _calculate() override;

private:
    /*********************************************************
     * Private function
     *********************************************************/
    /**
     * @brief calculate kinematic arm attitude and position
     *
     * @param args : PostureManagerArguments address
     */
    void _kinetic_calc(PostureManagerArguments *args);

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    int state = FREE;                                               /*!< state */
    DriverPotentiometer joint[SENSOR_ARM_DRIVER_POTENTIOMETER_NUM]; /*!< DriverPotentiometer of each joint */
    Vector<DIMENSION_OF_HOMO_MATRIX> hand_point;                    /*!< hand position */

    float hand_x    = 0; /*!< x position of hand */
    float hand_y    = 0; /*!< y position of hand */
    float hand_sita = 0; /*!< sita position of hand */
    float hand_r    = 0; /*!< r position of hand */

    /* Kinematic model calculation */
    const float link_arm[JOINT_NUM] = { 0.0, 0.0, 0.134, 0.0, 0.145, 0, 0, 0 }; /*!< link arm length [m] */

    Tensor<DIMENSION_OF_HOMO_MATRIX> tensor_operator; /*!< tensor operation object. */

    PostureManagerArguments *_args; /*!< PostureManagerArguments address */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _SEMI_CIRCLE_DEGREE    = 180; /*!<  */
    const int _QUARTER_CIRCLE_DEGREE = 90;  /*!<  */

    const float _HAND_SCALING_FACTOR = 1000.0; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
