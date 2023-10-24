/**
 * @file posture_manager_leg.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief PostureManager_Arm
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_POSTURE_MANAGER_LEG_HPP
#define ARM_CONTROLLER_POSTURE_MANAGER_LEG_HPP

#include "maid_robot_system/common/interface/control_if.hpp"
#include "posture_manager_arguments.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Manage posture of leg
 *
 */
class PostureManager_Leg : public ControlIF {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    PostureManager_Leg();

    /*********************************************************
     * Destructor
     *********************************************************/
    ~PostureManager_Leg();

    /**
     * @brief Set the posture address object
     *
     * @param PostureManagerArguments address
     */
    void set_posture_address(PostureManagerArguments *args);

    /**
     * @brief Set the target pos object
     *
     * @param target_pos : target position
     */
    void set_target_pos(float target_pos);

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
     * @brief reset
     *
     * @param args : PostureManagerArguments object address
     */
    void _reset(PostureManagerArguments *args);

private:
    /*********************************************************
     * Private variables
     *********************************************************/
#ifdef CIRO
    const int _upper_speed         = 2500;  /*!< upper speed */
    const int _down_speed          = -2500; /*!< lower speed */
    const int _default_standup_pos = 950;   /*!< default standup position */
    const int _default_sit_pos     = 100;   /*!< default sitting position */
#endif
#ifdef CIYA
    const int _upper_speed         = 1500;  /*!< upper speed */
    const int _down_speed          = -2500; /*!< lower speed */
    const int _default_standup_pos = 950;   /*!< default standup position */
    const int _default_sit_pos     = 100;   /*!< default sitting position */
#endif

    float _center_pos = 0.0; /*!< center position */
    float _target_pos = 0.0; /*!< target position */

    int _target_speed = 0; /*!< target speed */
    int _now_speed    = 0; /*!< current speed */

    TimeCheck _timer_dt; /*!< TimeCheck object */

    PostureManagerArguments *_args; /*!< PostureManagerArguments address */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _LEG_STAND_LIMIT        = 300;  /*!<  */
    const float _SIN_CYCLE_POS_FACTOR = 18.0; /*!<  */

    const int _LEG_SEND_POS_MIN = 0;    /*!<  */
    const int _LEG_SEND_POS_MAX = 1000; /*!<  */

    const float _SHOULDER_LEFT_SPEED_FACTOR = 100.0; /*!<  */

    const int _SPEED_X_MIN = 0;   /*!<  */
    const int _SPEED_X_MAX = 100; /*!<  */

    const int _SIT_POS_OFFSET   = 150; /*!<  */
    const int _STAND_POS_OFFSET = 50;  /*!<  */

    const float _DOWN_TO_TARGET_SPEED_FACTOR = 3.0; /*!<  */

    const int _HAND_Z_RISE_KNEE_LIMIT = 100; /*!<  */

    const int _DEFAULT_STANDUP_OFFSET = 150; /*!<  */

    const int _ONE_STEP_SPEED_INCREMENT_FOR_POSITIVE    = 20; /*!<  */
    const int _ONE_STEP_SPEED_INCREMENT_FOR_NEGATIVE    = 30; /*!<  */
    const int _ONE_STEP_SPEED_DECREMENT_FOR_LARGE_SPEED = 50; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
