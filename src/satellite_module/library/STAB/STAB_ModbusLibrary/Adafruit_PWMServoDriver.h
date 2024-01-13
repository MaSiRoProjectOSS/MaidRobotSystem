/**
 * @file Adafruit_PWMServoDriver.h
 * @brief
 * @version 0.23.12
 * @date 2024-01-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */

#ifndef STAB_ADAFRUIT_PWMSERVODRIVER_H
#define STAB_ADAFRUIT_PWMSERVODRIVER_H

#define PCA9685_I2C_ADDRESS 0x40
#include <Arduino.h>

class Adafruit_PWMServoDriver {
public:
    Adafruit_PWMServoDriver()
    {
    }
    Adafruit_PWMServoDriver(const uint8_t addr)
    {
    }
#if 0
    Adafruit_PWMServoDriver(const uint8_t addr, TwoWire &i2c)
    {
    }
#endif
    bool begin(uint8_t prescale = 0)
    {
        return true;
    }
    void reset()
    {
    }
    void sleep()
    {
    }
    void wakeup()
    {
    }
    void setExtClk(uint8_t prescale)
    {
    }
    void setPWMFreq(float freq)
    {
    }
    void setOutputMode(bool totempole)
    {
        this->_totempole = totempole;
    }
    uint16_t getPWM(uint8_t num, bool off = false)
    {
        return this->_num;
    }
    uint8_t setPWM(uint8_t num, uint16_t on, uint16_t off)
    {
        this->_num = num;
        return this->_num;
    }
    void setPin(uint8_t num, uint16_t val, bool invert = false)
    {
    }
    uint8_t readPrescale(void)
    {
        return 0;
    }
    void writeMicroseconds(uint8_t num, uint16_t Microseconds)
    {
    }

    void setOscillatorFrequency(uint32_t freq)
    {
        this->_oscillator_frequency = freq;
    }
    uint32_t getOscillatorFrequency(void)
    {
        return this->_oscillator_frequency;
    }

private:
    uint8_t _num                   = 0;
    bool _totempole                = false;
    uint32_t _oscillator_frequency = 0;
};

#endif
