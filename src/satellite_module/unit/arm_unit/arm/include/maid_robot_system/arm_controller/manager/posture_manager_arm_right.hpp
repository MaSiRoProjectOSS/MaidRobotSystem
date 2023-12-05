/**
 * @file posture_manager_arm_right.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_POSTURE_MANAGER_ARM_RIGHT_HPP
#define ARM_CONTROLLER_POSTURE_MANAGER_ARM_RIGHT_HPP

#include "maid_robot_system/common/interface/control_if.hpp"
#include "posture_manager_arguments.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief posture manager for right arm.
 *
 */
class PostureManager_ArmRight : public ControlIF {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    PostureManager_ArmRight();
    /*********************************************************
     * Destructor
     *********************************************************/
    ~PostureManager_ArmRight();

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
    PostureManagerArguments *_args; /*!< PostureManagerArguments address */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
