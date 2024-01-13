/**
 * @file arduino_external_interrupts.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ARDUINO_FUNC_ARDUINO_EXTERNAL_INTERRUPTS_HPP
#define ARDUINO_FUNC_ARDUINO_EXTERNAL_INTERRUPTS_HPP

#include "arduino_types.hpp"

void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode);
void detachInterrupt(uint8_t interruptNum);

#endif
