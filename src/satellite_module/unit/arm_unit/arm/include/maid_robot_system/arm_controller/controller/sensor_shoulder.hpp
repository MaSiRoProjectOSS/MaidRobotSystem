/**
 * @file sensor_shoulder.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief calculate sensor shoulder.
 * @version 0.23.2
 * @date 2023-05-02
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_SENSOR_SHOULDER_HPP
#define ARM_CONTROLLER_SENSOR_SHOULDER_HPP

#include "maid_robot_system/arm_controller/manager/posture_manager_arguments.hpp"
#include "maid_robot_system/common/interface/control_if.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief calculate sensor shoulder.
 *
 */
class SensorShoulder : public ControlIF {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    SensorShoulder();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief Set the posture manager arguments address.
     *
     * @param args : PostureManagerArguments pointer
     */
    void set_posture_manager_address(PostureManagerArguments *args);

    /**
     * @brief Get the posture manager arguments address.
     *
     * @param args : PostureManagerArguments pointer
     */
    void get_posture_manager_address(PostureManagerArguments *args);

    /**
     * @brief calculate sensor shoulder.
     *
     * @param args : PostureManagerArguments pointer
     */
    void sensor_shoulder(PostureManagerArguments *args);

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
    PostureManagerArguments *_PostureManager_data;

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _X_MIN_OF_CIYA = 650; /*!<  */
    const int _X_MIN_OF_CIRO = 300; /*!<  */
    const int _X_MAX         = 0;   /*!<  */

    const int _Y_MIN = 0;   /*!<  */
    const int _Y_MAX = 100; /*!<  */

    double _PERCENT_TO_RATE = 100.0; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
