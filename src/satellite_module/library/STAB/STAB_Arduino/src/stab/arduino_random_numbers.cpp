/**
 * @file arduino_random_numbers.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "stab/arduino_random_numbers.hpp"

#include "stab/arduino_math.hpp"

unsigned long value_random_seed = 0;
long value_random_current       = 0;
long VALUE_RANDOM_CURRENT_MAX   = 100;

long random(long value)
{
    if (0 == value) {
        value_random_current++;
        if (value_random_current > VALUE_RANDOM_CURRENT_MAX) {
            value_random_current = 0;
        }
        return value_random_current;
    } else {
        return value;
    }
}
long random(long min, long max)
{
    long value = random(value_random_seed);
    value      = map(value, 0, 100, min, max);
    return 0;
}

void randomSeed(unsigned long seed)
{
    // if seed==0 then count up value that (0-100)
    value_random_seed = seed;
}
