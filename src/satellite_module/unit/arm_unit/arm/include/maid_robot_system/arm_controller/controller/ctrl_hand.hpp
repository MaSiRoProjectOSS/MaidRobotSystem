/**
 * @file ctrl_hand.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Control hand.
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_CTRL_HAND_HPP
#define ARM_CONTROLLER_CTRL_HAND_HPP

#include "maid_robot_system/arm_controller/controller/driver/speed_servo.hpp"
#include "maid_robot_system/common/interface/control_if.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Control hand
 *
 */
class CtrlHand : public ControlIF {
public:
    /**
     * @brief Construct a new Ctrl Hand object
     *
     */
    CtrlHand();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief test
     *
     * @param set : test angle
     */
    void test(int set);

    /**
     * @brief Run move_along_trajectory of SpeedServo
     *
     * @param tar_deg : target angle [deg]
     * @param tar_speed target speed
     */
    void move_along_trajectory(float tar_deg, int tar_speed);

protected:
    /*********************************************************
     * Inherited function
     *********************************************************/
    bool _begin() override;
    bool _end() override;
    bool _calculate() override;

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    SpeedServo _servo; /*!< SpeedServo object */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _SERVO_PORT        = PA15; /*!<  */
    const int _INITIAL_POSITION  = 0;    /*!<  */
    const int _INITIAL_DIRECTION = -1;   /*!<  */

    const int _TEST_DEG_X_MIN = 0;   /*!<  */
    const int _TEST_DEG_X_MAX = 100; /*!<  */
    const int _TEST_DEG_Y_MIN = 60;  /*!<  */
    const int _TEST_DEG_Y_MAX = 120; /*!<  */

    const int _TEST_SPEED = 10; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
