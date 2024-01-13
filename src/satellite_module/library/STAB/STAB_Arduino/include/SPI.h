/**
 * @file SPI.h
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.0.1
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef SPI_H
#define SPI_H

#include "stab/arduino_types.hpp"

#define SPI_PERIPHERALS_USED (1)
#define DEBUG_MODE           (0)

typedef enum
{
    SPI_MODE0 = 0,
    SPI_MODE1 = 1,
    SPI_MODE2 = 2,
    SPI_MODE3 = 3,
} SPIMode;

class SPISettings {
public:
    SPISettings(uint32_t clock, int bitOrder, int dataMode);
};

class SPIStruct {
public:
    SPIStruct(uint8_t mosi = 0, uint8_t miso = 0, uint8_t sclk = 0, uint8_t ssel = 0);

    ~SPIStruct();

public:
    uint8_t transfer(uint8_t data);
    uint16_t transfer16(uint16_t data);
    void transfer(void *buf, int count);

    // Transaction Functions
    void usingInterrupt(int interruptNumber);
    void notUsingInterrupt(int interruptNumber);
    void beginTransaction(SPISettings settings);
    void endTransaction(void);

    // SPI Configuration methods
    void attachInterrupt();
    void detachInterrupt();

    void begin();
    void end();

    void format(int bits, int mode = 0);
    void frequency(int hz = 1000000);
    int write(int value);
    int write(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length);
    void lock(void);
    void unlock(void);
    void select(void);
    void deselect(void);
    void set_default_write_value(char data);
    void abort_transfer();
    void clear_transfer_buffer();
    void abort_all_transfers();

public:
    // Configuration.
    uint8_t _mosi;
    uint8_t _miso;
    uint8_t _sclk;
    uint8_t _hw_ssel;

    int _bits;
    int _mode;
    int _hz;
    char _write_fill;
    int8_t _select_count;
    void (*_init_func)(SPIStruct *);
};

extern SPIStruct SPI;

#endif
