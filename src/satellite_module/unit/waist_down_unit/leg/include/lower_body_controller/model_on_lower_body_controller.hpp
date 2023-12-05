/**
 * @file model_on_lower_body_controller.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Handling Wheel Controller
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef LOWER_BODY_UNIT_MODEL_ON_LOWER_BODY_CONTROLLER_HPP
#define LOWER_BODY_UNIT_MODEL_ON_LOWER_BODY_CONTROLLER_HPP

#include "maid_robot_system/common/interface/model_if.hpp"

namespace lower_body_unit
{

/**
 * @brief Handling Wheel Controller
 */
class ModelOnLowerBodyController : public ModelIf {
    ///////////////////////////////////////////////////////////
    // Constructor
    ///////////////////////////////////////////////////////////
public:
    /**
     * @brief Construct a new Model On Wheel Controller object
     *
     */
    ModelOnLowerBodyController();

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
     *
     * @param mode current mode
     */
    bool _receive(ModeList mode) override;
    /**
     * @brief Operate on data
     *
     * @param mode current mode
     */
    bool _calculate(ModeList mode) override;
    /**
     * @brief Transmitting data
     *
     * @param mode current mode
     */
    void _send(ModeList mode) override;
    /**
     * @brief check error
     *
     * @param mode current mode
     */
    void _error_check(ModeList mode) override;
    /**
     * @brief to debug output
     *
     * @param mode current mode
     */
    void _debug_output(ModeList mode) override;
};

} // namespace lower_body_unit

#endif
