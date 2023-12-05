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
#include "common/time_check.hpp"

#include <Arduino.h>

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
bool TimeCheck::check_passing(unsigned long checking_time)
{
    bool result = this->check_time_over(checking_time);
    if (true == result) {
        this->update();
    }
    return result;
}

bool TimeCheck::check_time_over(unsigned long checking_time)
{
    bool result = false;
    if (this->get_elapsed_time() > checking_time) {
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
unsigned long TimeCheck::get_elapsed_lap_time(unsigned long checking_time)
{
    unsigned long dt = this->get_elapsed_time();
    if (dt > checking_time) {
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
        double sin_data = sin(PI / (2.0 * ((float)(dt) / (float)(time_cycle))));
        result          = sin_data * sin_data;
    }
    return result;
}

double TimeCheck::get_sin_cycle(unsigned long time_cycle)
{
    return sin(PI * 2.0 * ((float)this->get_elapsed_time() / (float)time_cycle));
}

double TimeCheck::get_cos_cycle(unsigned long time_cycle)
{
    return cos(PI * 2.0 * ((float)this->get_elapsed_time() / (float)time_cycle));
}

///////////////////////////////////////////////////////////
