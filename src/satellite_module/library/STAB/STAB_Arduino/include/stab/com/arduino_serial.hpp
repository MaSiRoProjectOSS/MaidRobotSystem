/**
 * @file arduino_serial.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ARDUINO_FUNC_COMMUNICATION_ARDUINO_SERIAL_HPP
#define ARDUINO_FUNC_COMMUNICATION_ARDUINO_SERIAL_HPP

#include "../arduino_types.hpp"
#define SERIAL_BUFFER_SIZE (255)
#include <stdio.h>
#include <stdlib.h>

#define SERIAL_8N1 (0x06)
#define SERIAL_8N2 (0x0E)
#define SERIAL_7E1 (0x24)
#define SERIAL_8E1 (0x26)
#define SERIAL_7E2 (0x2C)
#define SERIAL_8E2 (0x2E)
#define SERIAL_7O1 (0x34)
#define SERIAL_8O1 (0x36)
#define SERIAL_7O2 (0x3C)
#define SERIAL_8O2 (0x3E)

class Serial_STAB {
public:
    enum
    {
        ONE_STOP_BIT          = 0,
        ONE_AND_HALF_STOP_BIT = 1,
        TWO_STOP_BITS         = 2,
    };
    enum
    {
        NO_PARITY    = 0,
        ODD_PARITY   = 1,
        EVEN_PARITY  = 2,
        MARK_PARITY  = 3,
        SPACE_PARITY = 4,
    };

private:
    int peek_buffer;

public:
    Serial_STAB();
    Serial_STAB(int uart_nr);
    Serial_STAB(uint8_t rx, uint8_t tx);
    ~Serial_STAB();

public:
    void begin();
    void begin(unsigned long);
    void begin(unsigned long, uint8_t);
    void end(void);

    int available(void);
    int peek(void);
    int read(void);
    int availableForWrite(void);
    void flush(void);
    size_t write(uint8_t);
    size_t write(const uint8_t *, size_t);
    void println(const char *format, ...);
    void print(const char *format, ...);

    int32_t readBreak();
    size_t readBytes(uint8_t *buffer, size_t length);

    uint32_t baud();
    uint8_t stopbits();
    uint8_t paritytype();
    uint8_t numbits();
    bool dtr();
    bool rts();

    void setTimeout(unsigned long timeout);

protected:
    TaskHandle_t _eventTask;

private:
    volatile uint8_t _rx_buffer_head             = 0;
    volatile uint8_t _rx_buffer_tail             = 0;
    unsigned char _rx_buffer[SERIAL_BUFFER_SIZE] = { 0 };
};

typedef Serial_STAB SoftwareSerial;
typedef Serial_STAB HardwareSerial;

extern Serial_STAB Serial;
extern Serial_STAB Serial1;
extern Serial_STAB Serial2;
extern Serial_STAB Serial3;

#endif
