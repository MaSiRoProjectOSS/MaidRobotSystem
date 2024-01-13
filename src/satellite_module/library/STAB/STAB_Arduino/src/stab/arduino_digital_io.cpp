/**
 * @file arduino_digital_io.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "stab/arduino_digital_io.hpp"

#define ARDUINO_DIGITAL_IO_VALUE_MAX 255
uint8_t arduino_digital_io_value[ARDUINO_DIGITAL_IO_VALUE_MAX] = { 0 };

void pinMode(uint8_t pin, uint8_t mode)
{
}

void digitalWrite(uint8_t pin, uint8_t val)
{
    if (ARDUINO_DIGITAL_IO_VALUE_MAX > pin) {
        arduino_digital_io_value[pin] = val;
    }
}
int digitalRead(uint8_t pin)
{
    int value = 0;
    if (ARDUINO_DIGITAL_IO_VALUE_MAX > pin) {
        value = arduino_digital_io_value[pin];
    }
    return value;
}
