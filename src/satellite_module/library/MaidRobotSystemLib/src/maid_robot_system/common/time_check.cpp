/**
 * @file time_check.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Check elapsed time
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */

#include "maid_robot_system/common/time_check.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace common
{
#ifndef HALF_PI
#define HALF_PI PI / 2.0
#endif

///////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////
TimeCheck::TimeCheck(unsigned long period_time)
{
    this->_period_time = period_time;
    this->update();
}

///////////////////////////////////////////////////////////
// Control function
///////////////////////////////////////////////////////////
void TimeCheck::update()
{
    this->_lap_start_time = millis();
}

void TimeCheck::from_scratch()
{
    this->_lap_start_time = 0;
}

///////////////////////////////////////////////////////////
// Check function
///////////////////////////////////////////////////////////
bool TimeCheck::check_passing(unsigned long period_time)
{
    bool result = this->check_time_over(period_time);
    if (true == result) {
        this->update();
    }
    return result;
}

bool TimeCheck::check_time_over(unsigned long period_time)
{
    bool result = false;
    if (this->get_elapsed_time() > period_time) {
        result = true;
    }
    return result;
}
bool TimeCheck::check_passing()
{
    return this->check_passing(this->_period_time);
}

bool TimeCheck::check_time_over()
{
    return this->check_time_over(this->_period_time);
}

///////////////////////////////////////////////////////////
// Getter
///////////////////////////////////////////////////////////
unsigned long TimeCheck::get_elapsed_lap_time(unsigned long period_time)
{
    unsigned long dt = this->get_elapsed_time();
    if (dt > period_time) {
        this->update();
    }
    return dt;
}
unsigned long TimeCheck::get_elapsed_lap_time()
{
    return this->get_elapsed_lap_time(this->_period_time);
}

unsigned long TimeCheck::get_lap_time()
{
    unsigned long dt = this->get_elapsed_time();
    this->update();
    return dt;
}

unsigned long TimeCheck::get_elapsed_time()
{
    return millis() - this->_lap_start_time;
}

///////////////////////////////////////////////////////////
// Calculate function
///////////////////////////////////////////////////////////
double TimeCheck::get_s_curve_flag(unsigned long time_cycle)
{
    unsigned long dt = this->get_elapsed_time();
    double result    = 1.0;
    if (dt < time_cycle) {
        if (0 == dt) {
            result = 0;
        } else {
            double sin_data = sin(HALF_PI * ((float)(dt) / (float)(time_cycle)));
            result          = sin_data * sin_data;
        }
    }
    return result;
}

double TimeCheck::get_sin_cycle(unsigned long time_cycle)
{
    double value = 1.0;
    if (0 < time_cycle) {
        value = sin(HALF_PI * ((float)this->get_elapsed_time() / (float)time_cycle));
    }
    return value;
}

double TimeCheck::get_cos_cycle(unsigned long time_cycle)
{
    double value = 0.0;
    if (0 < time_cycle) {
        value = cos(HALF_PI * ((float)this->get_elapsed_time() / (float)time_cycle));
    }
    return value;
}

///////////////////////////////////////////////////////////

} // namespace common
} // namespace maid_robot_system
