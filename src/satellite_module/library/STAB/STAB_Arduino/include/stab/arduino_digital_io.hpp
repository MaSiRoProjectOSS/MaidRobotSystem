/**
 * @file arduino_digital_io.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ARDUINO_FUNC_ARDUINO_DIGITAL_IO_HPP
#define ARDUINO_FUNC_ARDUINO_DIGITAL_IO_HPP

#include "arduino_types.hpp"

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);

#endif
