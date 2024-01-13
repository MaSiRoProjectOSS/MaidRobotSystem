/**
 * @file arduino_time.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "stab/arduino_time.hpp"

void delay(unsigned long ms)
{
}

void delayMicroseconds(unsigned int us)
{
}

unsigned long millis(unsigned long ms)
{
    static unsigned long counter = 0;
    if (0 != ms) {
        counter = ms;
    } else {
        counter++;
    }
    return counter;
}

unsigned long micros(unsigned long us)
{
    static unsigned long counter = 0;
    if (0 != us) {
        counter = us;
    } else {
        counter++;
    }
    return counter;
}
