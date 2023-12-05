/**
 * @file control_if.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Control Class Interface
 * @version 0.1
 * @date 2023-02-14
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef COMMON_INTERFACE_CONTROL_IF_HPP
#define COMMON_INTERFACE_CONTROL_IF_HPP

#include "common/interface/mode_list.hpp"
#include "common/time_check.hpp"

/**
 * @brief Control Class Interface
 */
class ControlIF {
public:
    /**
     * @brief Construct a new Control I F object
     *
     */
    ControlIF();

public:
    /**
     * @brief Start of processing
     * @public
     *
     * @return true : Successful
     * @return false : Failed
     */
    bool begin();
    /**
     * @brief Stop of processing
     * @public
     *
     * @return true : Successful
     * @return false : Failed
     */
    bool end();
    /**
     * @brief perform calculations
     * @public
     *
     * @return true : Successful
     * @return false : Failed
     */
    bool calculate();

    /**
     * @brief Allow action
     * @public
     *
     */
    void enable();
    /**
     * @brief Stop order
     * @public
     *
     */
    void disable();

    /**
     * @brief to reboot
     * @public
     *
     * @return true : Successful
     * @return false : Failed
     */
    bool restart();

    /**
     * @brief the system is working
     * @public
     *
     * @return true : no stop order
     * @return false : STOP order
     */
    bool is_enable();

    /**
     * @brief Check for error occurrence
     * @public
     *
     * @return true : ERROR occurred
     * @return false : no error
     */
    bool is_error();

    /**
     * @brief Get the mode
     * @public
     *
     * @return ModeList current mode
     */
    ModeList get_mode();

    /**
     * @brief Emergency stop
     * @public
     * @attention top-level locking function
     */
    void system_lock();
    /**
     * @brief There are no blocking.
     * @public
     * @attention top-level locking function
     */
    void system_unlock();

protected:
    /**
     * @brief Start of processing
     * @attention Called when the public function "end()" is called.
     * @protected
     *
     * @return true : Accepted
     * @return false : Rejected
     */
    virtual bool _begin() = 0;
    /**
     * @brief Stop of processing
     * @attention Called when the public function "end()" is called.
     * @protected
     *
     * @return true : Accepted
     * @return false : Rejected
     */
    virtual bool _end() = 0;
    /**
     * @brief perform calculations
     * @attention Called when the public function "calculate()" is called.
     * @attention
     * @protected
     *
     * @return true : Accepted
     * @return false : Rejected
     */
    virtual bool _calculate() = 0;

    /**
     * @brief Check for error occurrence
     * @attention Called when the public function "is_error()" is called.
     * @attention
     *
     * @return true : ERROR occurred
     * @return false : no error
     */
    virtual bool _is_error();
    /**
     * @brief Allow action
     * @attention Called when the public function "enable()" is called.
     *
     * @return true : Accepted
     * @return false : Rejected
     */
    virtual bool _enable();
    /**
     * @brief Stop order
     * @attention Called when the public function "disable()" is called.
     * @protected
     *
     * @return true : Accepted
     * @return false : Rejected
     */
    virtual bool _disable();

protected:
    /**
     * @brief change mode
     * @protected
     *
     * @param mode : request mode
     */
    void _set_mode(ModeList mode);

private:
    bool _flag_initialized       = false;                                  /*!< Initialization flag */
    bool _if_flag_enable         = false;                                  /*!< Operation permission flag */
    bool _if_flag_emergency_stop = false;                                  /*!< Emergency stop flag */
    ModeList _if_mode            = ModeList::MODE_ERROR_NOT_COMMUNICATION; /*!< current mode */
};

#endif
