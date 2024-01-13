/**
 * @file arduino_math.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "stab/arduino_math.hpp"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    const long run = in_max - in_min;
    if (run == 0) {
        return -1; // AVR returns -1, SAM returns 0
    }
    const long rise  = out_max - out_min;
    const long delta = x - in_min;
    return (delta * rise) / run + out_min;
}
