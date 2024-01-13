/**
 * @file Arduino.h
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-22
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H

#ifndef _NOP
#define _NOP()                                                                                                                                                                     \
    do {                                                                                                                                                                           \
        __asm__ volatile("nop");                                                                                                                                                   \
    } while (0)
#endif

//////////////////////////////////////////////
// API
//////////////////////////////////////////////
#include "Servo.h"
#include "stab/api/String.h"

//////////////////////////////////////////////
// arduino fuction
//////////////////////////////////////////////
#include "stab/arduino_advanced_io.hpp"
#include "stab/arduino_analog_io.hpp"
#include "stab/arduino_bits_and_bytes.hpp"
#include "stab/arduino_characters.hpp"
#include "stab/arduino_digital_io.hpp"
#include "stab/arduino_external_interrupts.hpp"
#include "stab/arduino_interrupts.hpp"
#include "stab/arduino_math.hpp"
#include "stab/arduino_random_numbers.hpp"
#include "stab/arduino_time.hpp"
#include "stab/arduino_trigonometry.hpp"
#include "stab/arduino_usb.hpp"
#include "stab/arduino_zero_due_and_mkr_family.hpp"
#include "stab/com/arduino_serial.hpp"

//////////////////////////////////////////////
// pin setting
//////////////////////////////////////////////
#include "stab/pin_name.hpp"

//////////////////////////////////////////////
// DEFINE
//////////////////////////////////////////////
#define F(mess) (mess)

#endif
