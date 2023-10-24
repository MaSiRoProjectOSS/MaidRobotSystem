/**
 * @file model_if.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/common/interface/model_if.hpp"

namespace maid_robot_system
{
namespace common
{

///////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////
ModelIf::ModelIf()
{
}

///////////////////////////////////////////////////////////
// Getter / Setter
///////////////////////////////////////////////////////////
ModeList ModelIf::get_mode()
{
    return this->_mode;
}

void ModelIf::set_interval_check_error(int interval_ms)
{
    this->_check_error_interval = interval_ms;
    this->happened_error.update();
}

///////////////////////////////////////////////////////////
// public function
///////////////////////////////////////////////////////////
bool ModelIf::setup()
{
    bool result = true;
    result      = this->_setup();
    //////////////////////////////////////////////
    if (false == result) {
        this->_mode = ModeList::MODE_NOT_INITIALIZED;
    } else {
        this->_mode = ModeList::MODE_RUNNING;
        this->happened_error.update();
    }
    (void)this->_happened_change_mode(this->_mode);
    //////////////////////////////////////////////
    return result;
}

void ModelIf::loop()
{
    if (ModeList::MODE_RUNNING == this->_mode) {
        bool result = this->_receive(this->_mode);
        if (true == result) {
            result = this->_calculate(this->_mode);
        }
        if (true == result) {
            (void)this->_send(this->_mode);
        }
    }
    if (true == this->happened_error.check_passing(this->_check_error_interval)) {
        (void)this->_error_check(this->_mode);
    }
    (void)this->_happened_change_mode(this->_mode);
    (void)this->_debug_output(this->_mode);
}

///////////////////////////////////////////////////////////
// protected function
///////////////////////////////////////////////////////////
void ModelIf::_debug_output(ModeList mode)
{
    // do nothing
}

void ModelIf::_set_mode(ModeList mode)
{
    this->_mode = mode;
    (void)this->_happened_change_mode(this->_mode);
}

void ModelIf::_changed_mode(ModeList mode)
{
    // do nothing
}

///////////////////////////////////////////////////////////
// callback function
///////////////////////////////////////////////////////////
void ModelIf::set_callback_change_mode(CALLBACK_CHANGE_MODE callback)
{
    this->_callback_changed_mode = callback;
}

void ModelIf::_happened_change_mode(ModeList mode)
{
    static ModeList previous = ModeList::MODE_NOT_INITIALIZED;
    if (previous != mode) {
        if (NULL != this->_callback_changed_mode) {
            this->_changed_mode(mode);
            (void)this->_callback_changed_mode(mode);
        }
        previous = mode;
    }
}

} // namespace common
} // namespace maid_robot_system
