/**
 * @file SPI.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.0.1
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "SPI.h"

SPISettings::SPISettings(uint32_t clock, int bitOrder, int dataMode)
{
}

SPIStruct::SPIStruct(uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t ssel)
{
}

SPIStruct::~SPIStruct()
{
}
uint8_t SPIStruct::transfer(uint8_t data)
{
    return 0;
}
uint16_t SPIStruct::transfer16(uint16_t data)
{
    return 0;
}
void SPIStruct::transfer(void *buf, int count)
{
}

// Transaction Functions
void SPIStruct::usingInterrupt(int interruptNumber)
{
}
void SPIStruct::notUsingInterrupt(int interruptNumber)
{
}
void SPIStruct::beginTransaction(SPISettings settings)
{
}
void SPIStruct::endTransaction(void)
{
}

void SPIStruct::attachInterrupt()
{
}
void SPIStruct::detachInterrupt()
{
}

void SPIStruct::begin()
{
}
void SPIStruct::end()
{
}

void SPIStruct::format(int bits, int mode)
{
}
void SPIStruct::frequency(int hz)
{
}
int SPIStruct::write(int value)
{
    return 0;
}
int SPIStruct::write(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length)
{
    return 0;
}
void SPIStruct::lock(void)
{
}
void SPIStruct::unlock(void)
{
}
void SPIStruct::select(void)
{
}
void SPIStruct::deselect(void)
{
}
void SPIStruct::set_default_write_value(char data)
{
}
void SPIStruct::abort_transfer()
{
}
void SPIStruct::clear_transfer_buffer()
{
}
void SPIStruct::abort_all_transfers()
{
}

SPIStruct SPI;
