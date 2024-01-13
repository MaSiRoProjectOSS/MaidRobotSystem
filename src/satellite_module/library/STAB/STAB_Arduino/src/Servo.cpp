/**
 * @file Servo.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.0.3
 * @date 2023-02-25
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "Servo.h"

Servo::Servo()
{
}
uint8_t Servo::attach(int pin, int value)
{
    this->_servoIndex = pin;
    this->write(value);
    return 0;
}
uint8_t Servo::attach(int pin, int min, int max, int value)
{
    this->_servoIndex = pin;
    this->_min        = min;
    this->_max        = min;
    this->write(value);
    return 0;
}
void Servo::detach()
{
}
void Servo::write(int value)
{
    this->_value = value;
}
void Servo::writeMicroseconds(int value)
{
    this->_value_microseconds = value;
}
int Servo::read()
{
    return this->_value;
}
int Servo::readMicroseconds()
{
    return this->_value_microseconds;
}
bool Servo::attached()
{
    return true;
}
