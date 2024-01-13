/**
 * @file arduino_types.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#pragma once
#ifndef ARDUINO_FUNC_ARDUINO_TYPES_HPP
#define ARDUINO_FUNC_ARDUINO_TYPES_HPP

#include <stdint.h>

typedef bool boolean;
typedef uint8_t byte;
//typedef int size_t;
typedef unsigned int time_t;
typedef void *TaskHandle_t;

#define HIGH (0x1)
#define LOW  (0x0)

#define INPUT        (0x0)
#define OUTPUT       (0x1)
#define INPUT_PULLUP (0x2)

#define SERIAL  (0x0)
#define DISPLAY (0x1)

#define LSBFIRST (0)
#define MSBFIRST (1)

#define CHANGE  (1)
#define FALLING (2)
#define RISING  (3)

#endif
