/**
 * @file four_dimensional.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.3
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "maid_robot_system/common/chart/four_dimensional.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace common
{

void FourDimensionalChart::FourDimensionalChartData::clear(float in_x, float in_y, float in_z, float in_w, unsigned long in_time_ms)
{
    this->is_marked    = false;
    this->is_saved     = false;
    this->data.time_ms = in_time_ms;
    this->data.x       = in_x;
    this->data.y       = in_y;
    this->data.z       = in_z;
    this->data.w       = in_w;
}

/**
 * @brief Construct a new Candle Stick Data:: Candle Stick Data object
 */
FourDimensionalChart::FourDimensionalChartData::FourDimensionalChartData(float in_x, float in_y, float in_z, float in_w, unsigned long in_time_ms)
{
    this->clear(in_x, in_y, in_z, in_w, in_time_ms);
}
void FourDimensionalChart::FourDimensionalChartData::set(float in_x, float in_y, float in_z, float in_w)
{
    this->set(in_x, in_y, in_z, in_w, millis());
}
/**
 * @brief Set the value of the candle stick.
 *
 * @param value     The value of the candle stick.
 * @param time      The time of the candle stick.
 */
void FourDimensionalChart::FourDimensionalChartData::set(float in_x, float in_y, float in_z, float in_w, unsigned long in_time_ms)
{
    if (false == this->is_saved) {
        this->data.time_ms = in_time_ms;
        this->data.x       = in_x;
        this->data.y       = in_y;
        this->data.z       = in_z;
        this->data.w       = in_w;
    }
}

bool FourDimensionalChart::FourDimensionalChartData::mark()
{
    this->is_marked = true;
    return this->is_marked;
}
bool FourDimensionalChart::FourDimensionalChartData::save()
{
    if (true == this->is_marked) {
        this->is_saved = true;
    }
    return this->is_saved;
}

/////////////////////////////////////////////////////////////////////////////////////////////

FourDimensionalChart::FourDimensionalChart()
{
    this->_list.clear();
}

std::vector<FourDimensionalChart::FourDimensionalChartData> *FourDimensionalChart::get_list()
{
    return &this->_list;
}

size_t FourDimensionalChart::size()
{
    return this->_list.size();
}

bool FourDimensionalChart::set(float in_x, float in_y, float in_z, float in_w)
{
    return this->set(in_x, in_y, in_z, in_w, millis());
}

bool FourDimensionalChart::set(float in_x, float in_y, float in_z, float in_w, unsigned long in_time_ms)
{
    bool result = false;

    if (0 < this->_list.size()) {
        if (in_x == this->_list.back().data.x) {
            if (in_y == this->_list.back().data.y) {
                if (in_z == this->_list.back().data.z) {
                    if (in_w == this->_list.back().data.w) {
                        result = true;
                    }
                }
            }
        }
        if (false == result) {
            if (true != this->_list.back().is_saved) {
                if (in_time_ms == this->_list.back().data.time_ms) {
                    this->_list.back().set(in_x, in_y, in_z, in_w, in_time_ms);
                    result = true;
                }
            }
        }
    }
    if (false == result) {
        this->_remove_list();
        FourDimensionalChartData data(in_x, in_y, in_z, in_w, in_time_ms);
        this->_list.push_back(data);
        result = true;
    }

    return result;
}

int FourDimensionalChart::_remove_list()
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
