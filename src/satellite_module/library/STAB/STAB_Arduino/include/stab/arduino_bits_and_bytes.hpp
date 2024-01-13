/**
 * @file arduino_bits_and_bytes.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ARDUINO_FUNC_ARDUINO_BITS_AND_BYTES_HPP
#define ARDUINO_FUNC_ARDUINO_BITS_AND_BYTES_HPP

#define bit(b)                          (1UL << (b))
#define bitClear(value, bit)            ((value) &= ~(1UL << (bit)))
#define bitRead(value, bit)             (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)              ((value) |= (1UL << (bit)))
#define bitWrite(value, bit, bit_value) ((bit_value) ? bitSet(value, bit) : bitClear(value, bit))
#define highByte(w)                     ((uint8_t)((w) >> 8))
#define lowByte(w)                      ((uint8_t)((w)&0xff))

#endif
