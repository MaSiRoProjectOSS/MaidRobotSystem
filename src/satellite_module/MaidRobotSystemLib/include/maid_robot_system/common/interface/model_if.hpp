/**
 * @file model_if.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_COMMON_INTERFACE_MODEL_IF_HPP
#define MAID_ROBOT_SYSTEM_LIB_COMMON_INTERFACE_MODEL_IF_HPP

#include "maid_robot_system/common/mode_list.hpp"
#include "maid_robot_system/common/time_check.hpp"

namespace maid_robot_system
{
namespace common
{

class ModelIf {
public:
    ///////////////////////////////////////////////////////////
    // callback function
    ///////////////////////////////////////////////////////////
    using CALLBACK_CHANGE_MODE = void (*)(ModeList mode); /*!< Setting the callback function */
public:
    /**
     * @brief Construct a new Model If object
     */
    ModelIf();

    ///////////////////////////////////////////////////////////
    // public function
    ///////////////////////////////////////////////////////////
public:
    /**
     * @brief Make initial settings
     *
     * @return true     Successfully
     * @return false    Failed
     */
    bool setup();
    /**
     * @brief Calls for periodic processing
     */
    void loop();

    ///////////////////////////////////////////////////////////
    // Getter / Setter
    ///////////////////////////////////////////////////////////
public:
    /**
     * @brief Get current mode
     *
     * @return ModeList
     */
    ModeList get_mode();

    /**
     * @brief Set the interval check error object
     *
     * @param interval_ms
     */
    void set_interval_check_error(int interval_ms);

    /**
     * @brief Set the callback change mode object
     *
     * @param callback
     */
    void set_callback_change_mode(CALLBACK_CHANGE_MODE callback);

    ///////////////////////////////////////////////////////////
    // protected function
    ///////////////////////////////////////////////////////////
protected:
    /**
     * @brief set mode
     *
     * @param mode
     */
    void _set_mode(ModeList mode);

    ///////////////////////////////////////////////////////////
    /**
     * @brief model setup
     */
    virtual bool _setup() = 0;
    /**
     * @brief Receiving Data
     *
     * @param mode current mode
     */
    virtual bool _receive(ModeList mode) = 0;
    /**
     * @brief Operate on data
     *
     * @param mode current mode
     */
    virtual bool _calculate(ModeList mode) = 0;
    /**
     * @brief Transmitting data
     *
     * @param mode current mode
     */
    virtual void _send(ModeList mode) = 0;
    /**
     * @brief check error
     *
     * @param mode current mode
     */
    virtual void _error_check(ModeList mode) = 0;
    /**
     * @brief to debug output
     *
     * @param mode current mode
     */
    virtual void _debug_output(ModeList mode);

    /**
     * @brief Change mode
     *
     * @param mode current mode
     */
    virtual void _changed_mode(ModeList mode);

    ///////////////////////////////////////////////////////////
    // variable
    ///////////////////////////////////////////////////////////
private:
    /**
     * @brief Call the callback function when the mode changes
     *
     * @param mode current mode
     */
    void _happened_change_mode(ModeList mode);

private:
    CALLBACK_CHANGE_MODE _callback_changed_mode = nullptr; /*!< Callbacks to be called */

    ModeList _mode = ModeList::MODE_NOT_INITIALIZED;   /*!< current mode */
    TimeCheck happened_error;                          /*!< Manage time to check for errors */
    unsigned int _check_error_interval = (2 * (1000)); /*!< Interval to determine error */
};

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
