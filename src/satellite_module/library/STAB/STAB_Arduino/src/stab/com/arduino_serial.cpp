/**
 * @file arduino_serial.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "stab/com/arduino_serial.hpp"

Serial_STAB::Serial_STAB()
{
    peek_buffer = -1;
}
Serial_STAB::Serial_STAB(int uart_nr)
{
    peek_buffer = -1;
}
Serial_STAB::Serial_STAB(uint8_t rx, uint8_t tx)
{
}
Serial_STAB::~Serial_STAB()
{
}

void Serial_STAB::begin()
{
}

void Serial_STAB::begin(unsigned long baud)
{
}
void Serial_STAB::begin(unsigned long, uint8_t)
{
}
void Serial_STAB::end(void)
{
}

int Serial_STAB::available(void)
{
    return 0;
}
int Serial_STAB::peek(void)
{
    return 0;
}
int Serial_STAB::read(void)
{
    return 0;
}
int Serial_STAB::availableForWrite(void)
{
    return 0;
}
void Serial_STAB::flush(void)
{
}
size_t Serial_STAB::write(uint8_t)
{
    return 0;
}
size_t Serial_STAB::write(const uint8_t *, size_t)
{
    return 0;
}

int32_t Serial_STAB::readBreak()
{
    return 0;
}

size_t Serial_STAB::readBytes(uint8_t *buffer, size_t length)
{
    return 0;
}

uint32_t Serial_STAB::baud()
{
    return 0;
}
uint8_t Serial_STAB::stopbits()
{
    return 0;
}
uint8_t Serial_STAB::paritytype()
{
    return 0;
}
uint8_t Serial_STAB::numbits()
{
    return 0;
}
bool Serial_STAB::dtr()
{
    return true;
}
bool Serial_STAB::rts()
{
    return true;
}
void Serial_STAB::println(const char *format, ...)
{
}

void Serial_STAB::print(const char *format, ...)
{
}

void Serial_STAB::setTimeout(unsigned long timeout)
{
}

Serial_STAB Serial;
Serial_STAB Serial1;
Serial_STAB Serial2;
Serial_STAB Serial3;
