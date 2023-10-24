/**
 * @file candle_stick.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.3
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "maid_robot_system/common/chart/candle_stick.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace common
{

void CandleStick::CandleStickData::clear()
{
    unsigned long current        = 0;
    this->_initialized           = false;
    this->_closed                = false;
    this->is_marked              = false;
    this->is_saved               = false;
    this->data.start_ms          = current;
    this->data.end_ms            = current;
    this->data.open              = 0;
    this->data.close             = 0;
    this->data.number_of_samples = 0;
    this->data.high              = __FLT_MIN__;
    this->data.low               = __FLT_MAX__;
    this->data.average           = 0;
    this->data.increase          = false;
}

/**
 * @brief Construct a new Candle Stick Data:: Candle Stick Data object
 */
CandleStick::CandleStickData::CandleStickData()
{
    this->clear();
}
void CandleStick::CandleStickData::set(float in_value)
{
    this->set(in_value, millis());
}
/**
 * @brief Set the value of the candle stick.
 *
 * @param in_value  The value of the candle stick.
 * @param time      The time of the candle stick.
 */
void CandleStick::CandleStickData::set(float in_value, unsigned long in_time_ms)
{
    if (false == this->_closed) {
        this->data.end_ms = in_time_ms;
        this->data.close  = in_value;
        if (false == this->_initialized) {
            this->_initialized  = true;
            this->data.start_ms = in_time_ms;
            this->data.open     = in_value;
        }
        this->data.increase = (this->data.close < this->data.open) ? true : false;
        this->data.high     = max(this->data.high, in_value);
        this->data.low      = min(this->data.low, in_value);
        this->data.number_of_samples++;
        ////////////
        this->data.average = ((this->data.average * (this->data.number_of_samples - 1)) + in_value) / this->data.number_of_samples;
    }
}

void CandleStick::CandleStickData::fin()
{
    this->_closed = true;
}

bool CandleStick::CandleStickData::save()
{
    if (true == this->is_marked) {
        this->is_saved = true;
    }
    return this->is_saved;
}
bool CandleStick::CandleStickData::mark()
{
    if (true == this->_closed) {
        this->is_marked = true;
    }
    return this->is_marked;
}

/////////////////////////////////////////////////////////////////////////////////////////////

CandleStick::CandleStick(unsigned long resolution_ms)
{
    this->_resolution_ms = resolution_ms;
    this->_time_next_ms  = 0;
    this->_list.clear();
}

std::vector<CandleStick::CandleStickData> *CandleStick::get_list()
{
    return &this->_list;
}

size_t CandleStick::size()
{
    return this->_list.size();
}

bool CandleStick::set(float in_value)
{
    return this->set(in_value, millis());
}

bool CandleStick::set(float in_value, unsigned long in_time_ms)
{
    bool result = false;

    if (this->_time_next_ms <= in_time_ms) {
        this->_time_next_ms = in_time_ms + this->_resolution_ms;
        if (0 < this->_list.size()) {
            this->_list.back().fin();
        }
        this->_remove_list();
        CandleStickData data;
        this->_list.push_back(data);
        result = true;
    }
    if (0 < this->_list.size()) {
        this->_list.back().set(in_value, in_time_ms);
    }
    return result;
}

int CandleStick::_remove_list()
{
    int result = 0;
    if (0 < this->_list.size()) {
        while ((this->MAX_LIST_SIZE < this->_list.size()) || (true == this->_list.front().is_saved)) {
            this->_list.erase(this->_list.begin());
            result++;
            if (0 == this->_list.size()) {
                break;
            }
        }
    }
    return result;
}

} // namespace common
} // namespace maid_robot_system
