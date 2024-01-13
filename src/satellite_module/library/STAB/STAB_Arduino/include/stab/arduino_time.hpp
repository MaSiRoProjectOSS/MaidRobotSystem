/**
 * @file arduino_time.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ARDUINO_FUNC_ARDUINO_TIME_HPP
#define ARDUINO_FUNC_ARDUINO_TIME_HPP

void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long millis(unsigned long ms = 0);
unsigned long micros(unsigned long us = 0);

#endif
