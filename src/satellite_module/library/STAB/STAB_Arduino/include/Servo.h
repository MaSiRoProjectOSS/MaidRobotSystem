/*
 * Defines for 16 bit timers used with  Servo library
 *
 * If _useTimerX is defined then TimerX is a 16 bit timer on the current board
 * timer16_Sequence_t enumerates the sequence that the timers should be allocated
 * _Nbr_16timers indicates how many 16 bit timers are available.
 */
/**
 * @file Servo.h
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.0.3
 * @date 2023-02-25
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef Servo_h
#define Servo_h

#include <inttypes.h>

#define Servo_VERSION (2) // software version of this library

#define MIN_PULSE_WIDTH     (544)   // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH     (2400)  // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH (1500)  // default pulse width when servo is attached
#define REFRESH_INTERVAL    (20000) // minumum time to refresh servos in microseconds

#define SERVOS_PER_TIMER (12) // the maximum number of servos controlled by one timer
#define MAX_SERVOS       (_Nbr_16timers * SERVOS_PER_TIMER)

#define INVALID_SERVO (255) // flag indicating an invalid servo index

#if !defined(ARDUINO_ARCH_STM32F4)

typedef struct {
    uint8_t nbr;      // a pin number from 0 to 255
    uint8_t isActive; // true if this channel is enabled, pin not pulsed if false
} ServoPin_t;

typedef struct {
    ServoPin_t Pin;
    volatile unsigned int ticks;
} servo_t;

class Servo {
public:
    Servo();
    uint8_t attach(int pin,
                   int value = DEFAULT_PULSE_WIDTH); // attach the given pin to the next free channel, sets pinMode, set angle value, returns channel number or 0 if failure
    uint8_t attach(int pin, int min, int max, int value = DEFAULT_PULSE_WIDTH); // as above but also sets min and max values for writes.
    void detach();
    void write(int value);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
    void writeMicroseconds(int value); // Write pulse width in microseconds
    int read();                        // returns current pulse width as an angle between 0 and 180 degrees
    int readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
    bool attached();                   // return true if this servo is attached, otherwise false
private:
    uint8_t _servoIndex; // index into the channel data for this servo
    int8_t _min;         // minimum is this value times 4 added to MIN_PULSE_WIDTH
    int8_t _max;         // maximum is this value times 4 added to MAX_PULSE_WIDTH
private:
    int _value              = 0;
    int _value_microseconds = 0;
};

#endif
#endif
