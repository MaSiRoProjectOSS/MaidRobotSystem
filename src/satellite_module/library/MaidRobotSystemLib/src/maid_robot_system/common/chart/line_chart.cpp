/**
 * @file line_chart.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.3
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "maid_robot_system/common/chart/line_chart.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace common
{

void LineChart::LineChartData::clear(float in_value, unsigned long in_time_ms)
{
    this->is_marked    = false;
    this->is_saved     = false;
    this->data.time_ms = in_time_ms;
    this->data.value   = in_value;
}

/**
 * @brief Construct a new Candle Stick Data:: Candle Stick Data object
 */
LineChart::LineChartData::LineChartData(float in_value, unsigned long in_time_ms)
{
    this->clear(in_value, in_time_ms);
}
void LineChart::LineChartData::set(float in_value)
{
    this->set(in_value, millis());
}
/**
 * @brief Set the value of the candle stick.
 *
 * @param value     The value of the candle stick.
 * @param time      The time of the candle stick.
 */
void LineChart::LineChartData::set(float in_value, unsigned long in_time_ms)
{
    if (false == this->is_saved) {
        this->data.time_ms = in_time_ms;
        this->data.value   = in_value;
    }
}

bool LineChart::LineChartData::mark()
{
    this->is_marked = true;
    return this->is_marked;
}
bool LineChart::LineChartData::save()
{
    if (true == this->is_marked) {
        this->is_saved = true;
    }
    return this->is_saved;
}

/////////////////////////////////////////////////////////////////////////////////////////////

LineChart::LineChart()
{
    this->_list.clear();
}

std::vector<LineChart::LineChartData> *LineChart::get_list()
{
    return &this->_list;
}

size_t LineChart::size()
{
    return this->_list.size();
}

bool LineChart::set(float in_value)
{
    return this->set(in_value, millis());
}

bool LineChart::set(float in_value, unsigned long in_time_ms)
{
    bool result = false;

    if (0 < this->_list.size()) {
        if (in_value == this->_list.back().data.value) {
            result = true;
        } else if (true != this->_list.back().is_saved) {
            if (in_time_ms == this->_list.back().data.time_ms) {
                this->_list.back().set(in_value, in_time_ms);
                result = true;
            }
        }
    }
    if (false == result) {
        this->_remove_list();
        LineChartData data(in_value, in_time_ms);
        this->_list.push_back(data);
        result = true;
    }

    return result;
}

int LineChart::_remove_list()
{
    int result = 0;
    if (0 < this->_list.size()) {
        while ((this->MAX_LIST_SIZE < this->_list.size()) || (true == this->_list.front().is_saved)) {
            if (this->RETAIN_DATA_SIZE >= this->_list.size()) {
                break;
            }
            this->_list.erase(this->_list.begin());
            result++;
        }
    }
    return result;
}

} // namespace common
} // namespace maid_robot_system
