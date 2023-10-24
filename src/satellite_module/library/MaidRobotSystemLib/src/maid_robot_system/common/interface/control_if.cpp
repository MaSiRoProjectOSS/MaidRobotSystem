/**
 * @file control_if.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Control Class Interface
 * @version 0.1
 * @date 2023-02-14
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */

#include "maid_robot_system/common/interface/control_if.hpp"

namespace maid_robot_system
{
namespace common
{

ControlIF::ControlIF()
{
    this->_if_flag_enable         = true;
    this->_if_flag_emergency_stop = false;
    this->_flag_initialized       = false;
    this->_if_mode                = ModeList::MODE_NOT_INITIALIZED;
}
///////////////////////////////////////////////////////
bool ControlIF::_enable()
{
    return true;
}
bool ControlIF::_disable()
{
    return false;
}
bool ControlIF::_is_error()
{
    return false;
}
///////////////////////////////////////////////////////
bool ControlIF::begin()
{
    bool result = true;
    if (false == this->_flag_initialized) {
        result = this->_begin();
        if (true == result) {
            this->_set_mode(ModeList::MODE_RUNNING);
            this->system_unlock();
        } else {
            this->system_lock();
        }
        this->_flag_initialized = result;
    }
    return result;
}
bool ControlIF::end()
{
    bool result = true;
    if (true == this->_flag_initialized) {
        result = this->_end();
        if (true == result) {
            this->_set_mode(ModeList::MODE_FINISHED);
        } else {
            this->system_lock();
            this->_set_mode(ModeList::MODE_ERROR_NOT_STOP);
        }
        this->_flag_initialized = !result;
    }
    return result;
}
bool ControlIF::restart()
{
    bool result = this->end();
    if (true == result) {
        result = this->begin();
    }
    return result;
}
bool ControlIF::calculate()
{
    bool result = false;
    if (true == this->_flag_initialized) {
        if (false == this->_if_flag_emergency_stop) {
            if (true == this->_if_flag_enable) {
                result = this->_calculate();
            }
        }
    }
    return result;
}

void ControlIF::system_lock()
{
    if (false == this->_if_flag_emergency_stop) {
        this->_if_flag_emergency_stop = true;
    }
}
void ControlIF::system_unlock()
{
    if (true == this->_if_flag_emergency_stop) {
        this->_if_flag_emergency_stop = false;
    }
}
void ControlIF::enable()
{
    bool result = this->_enable();
    if (true == result) {
        this->_if_flag_enable = true;
    }
}
void ControlIF::disable()
{
    bool result = this->_disable();
    if (true == result) {
        this->_if_flag_enable = false;
    }
}
bool ControlIF::is_enable()
{
    return this->_if_flag_enable && !this->_if_flag_emergency_stop;
}
bool ControlIF::is_error()
{
    return this->_is_error();
}

ModeList ControlIF::get_mode()
{
    return this->_if_mode;
}

void ControlIF::_set_mode(ModeList mode)
{
    this->_if_mode = mode;
}

} // namespace common
} // namespace maid_robot_system
