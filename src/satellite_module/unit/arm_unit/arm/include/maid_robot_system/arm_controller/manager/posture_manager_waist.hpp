/**
 * @file posture_manager_waist.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Manage posture of waist.
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_POSTURE_MANAGER_WAIST_HPP
#define ARM_CONTROLLER_POSTURE_MANAGER_WAIST_HPP

#include "maid_robot_system/common/interface/control_if.hpp"
#include "posture_manager_arguments.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Manage posture of waist.
 *
 */
class PostureManager_Waist : public ControlIF {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    PostureManager_Waist();

    /*********************************************************
     * Destructor
     *********************************************************/
    ~PostureManager_Waist();

public:
    /**
     * @brief Set the posture address object
     *
     * @param PostureManagerArguments address
     */
    void set_posture_address(PostureManagerArguments *args);

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
    PostureManagerArguments *_args; /*!< PostureManagerArguments address */

    unsigned long _resident_time;     /*!< resident time */
    int _limit_yaw[2]  = { -80, 80 }; /*!< range of yaw angle */
    int _limit_roll[2] = { -10, 10 }; /*!< range of roll angle */

    float _now_yaw   = 0; /*!< current yaw angle */
    float _now_pitch = 0; /*!< current pitch angle */
    float _now_roll  = 0; /*!< current roll angle */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const float _SEC_TO_MICRO_SEC_FACTOR = 1000000.0; /*!<  */

    const float _NECK_TO_LINK_NECK_YAW_FACTOR  = 2.0;  /*!<  */
    const float _NECK_TO_LINK_NECK_ROLL_FACTOR = -1.0; /*!<  */

    const float SIN_CYCLE_TO_BREATH_DELTA_YAW_FACTOR  = 5.0; /*!<  */
    const float SIN_CYCLE_TO_BREATH_DELTA_ROLL_FACTOR = 3.0; /*!<  */

    const int _BREATH_SPEED_FACTOR = 2; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
