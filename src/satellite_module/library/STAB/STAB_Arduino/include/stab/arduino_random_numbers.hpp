/**
 * @file arduino_random_numbers.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ARDUINO_FUNC_ARDUINO_RANDOM_NUMBERS_HPP
#define ARDUINO_FUNC_ARDUINO_RANDOM_NUMBERS_HPP

long random(long value);
long random(long min, long max);
void randomSeed(unsigned long seed);

#endif
