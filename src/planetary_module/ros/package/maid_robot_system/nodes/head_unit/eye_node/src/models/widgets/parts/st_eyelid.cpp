/**
 * @file st_eyelid.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-10-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "models/widgets/parts/st_eyelid.hpp"

namespace maid_robot_system
{
// =============================
// Constructor
// =============================
StEyelid::StEyelid()
{
    this->_fin_wink();
}
StEyelid::~StEyelid()
{
}

// =============================
// PUBLIC : Function
// =============================
void StEyelid::open_eye(uint wink_time_millisecond)
{
    // open eye during one wink
    if (true == _wink) {
        _start_time            = (_start_time + _wink_time_millisecond) - wink_time_millisecond;
        _wink_time_millisecond = wink_time_millisecond;
    }
}

void StEyelid::wink(uint wink_time_millisecond)
{
    if (false == _wink) {
        if (0 == _elapsedIndex) {
            _start_time = _current_time;
        }

        _wink_time_millisecond = wink_time_millisecond;
        _wink                  = true;
    } else {
        open_eye(wink_time_millisecond);
    }
}
bool StEyelid::enable_motion()
{
    return !_wink;
}

// =============================
// PUBLIC : Setter
// =============================
void StEyelid::set_elapsed(uint current_time)
{
#if 1

#else
    int elapsedIndex = _elapsedIndex;
    _current_time    = current_time;

    if (_start_time < current_time) {
        elapsedIndex = (int)((current_time - _start_time) / _wink_time_millisecond);

        if (EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX <= elapsedIndex) {
            if ((EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX * 2) > elapsedIndex) {
                elapsedIndex = EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX - elapsedIndex;
            } else {
                // finished wink
                this->_fin_win();
            }
        }
    }

    _elapsedIndex = elapsedIndex;

    if (true == enable_motion()) {
        if (false == _flag_next_blink) {
            _eye_blink_time  = this->set_eye_blink_time(blink_type::BLINK_TYPE_LONG) + current_time;
            _flag_next_blink = true;
        }

        if (_eye_blink_time < current_time) {
            wink(default_wink_time_millisecond);
        }
    }

#endif
}

// =============================
// PUBLIC : Getter
// =============================
uint StEyelid::get_elapsed()
{
    return _elapsedIndex;
}

// =============================
// PRIVATE : Variable
// =============================
void StEyelid::_fin_wink()
{
    _start_time      = 0xFFFFFFFF;
    _wink            = false;
    _elapsedIndex    = 0;
    _flag_next_blink = false;
}

} // namespace maid_robot_system
