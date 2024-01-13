/**
 * @file arduino_analog_io.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "stab/arduino_analog_io.hpp"

#define ARDUINO_ANALOG_IO_VALUE_MAX 255
int arduino_analog_io_value[ARDUINO_ANALOG_IO_VALUE_MAX] = { 0 };

int analogRead(uint8_t pin)
{
    int value = 0;
    if (ARDUINO_ANALOG_IO_VALUE_MAX > pin) {
        value = arduino_analog_io_value[pin];
    }
    return value;
}
void analogReference(uint8_t mode)
{
}
void analogWrite(uint8_t pin, int val)
{
    if (ARDUINO_ANALOG_IO_VALUE_MAX > pin) {
        arduino_analog_io_value[pin] = val;
    }
}
