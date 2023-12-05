/**
 * @file commutation_serial.hpp
 * @author  ()
 * @brief
 * @version 0.23.4
 * @date 2023-05-08
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef COMMUTATION_SERIAL_HPP
#define COMMUTATION_SERIAL_HPP

#include <Arduino.h>

template <int NUM>
class CommutationSerial {
public:
    CommutationSerial();

public:
    void setup(HardwareSerial *serial);
    void loop();

private:
    void _received();
    void _calculate();
    void _send();

private:
    HardwareSerial *_serial;
    const int SERIAL_DATA_MAX = NUM;
    char _data[NUM + 2];
    int index = 0;
};

template <int NUM>
CommutationSerial<NUM>::CommutationSerial()
{
}

template <int NUM>
void CommutationSerial<NUM>::setup(HardwareSerial *serial)
{
    this->_serial = serial;
}

template <int NUM>
void CommutationSerial<NUM>::loop()
{
    this->_received();
    this->_send();
}

template <int NUM>
void CommutationSerial<NUM>::_received()
{
    while (0 < this->_serial->available()) {
        char c = this->_serial->read();
        if ('\n' == c) {
            this->_data[this->index] = '\0';
            this->_calculate();
            this->index = 0;
            break;
        } else {
            this->_data[this->index++] = c;
            if (this->SERIAL_DATA_MAX <= this->index) {
                this->index = 0;
            }
        }
    }
}

template <int NUM>
void CommutationSerial<NUM>::_calculate()
{
    this->_serial->println(this->_data);
}

template <int NUM>
void CommutationSerial<NUM>::_send()
{
    static int count = 0;
    count++;
    if (1000 <= count) {
        //////////////////////////////////
        count = 0;
    }
}

#endif
