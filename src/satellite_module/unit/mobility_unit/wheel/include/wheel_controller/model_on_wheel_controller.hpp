/**
 * @file model_on_wheel_controller.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Handling Wheel Controller
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MOBILITY_UNIT_MODEL_ON_WHEEL_CONTROLLER_HPP
#define MOBILITY_UNIT_MODEL_ON_WHEEL_CONTROLLER_HPP

#include "maid_robot_system/common/interface/model_if.hpp"

namespace mobility_unit
{

/**
 * @brief Handling Wheel Controller
 */
class ModelOnWheelController : public ModelIf {
    ///////////////////////////////////////////////////////////
    // Constructor
    ///////////////////////////////////////////////////////////
public:
    /**
     * @brief Construct a new Model On Wheel Controller object
     *
     */
    ModelOnWheelController();

    ///////////////////////////////////////////////////////////
    // protected function
    ///////////////////////////////////////////////////////////
protected:
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
     * @brief Check to see if an error has occurred
     */
    void _error_check(ModeList mode) override;
    /**
     * @brief to debug output
     */
    void _debug_output(ModeList mode) override;
};

} // namespace mobility_unit

#endif
