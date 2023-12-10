/**
 * @file sensor_sht31.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief 温度センサー
 * @version 0.23.1
 * @date 2023-04-02
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "sensor_sht31.hpp"

Adafruit_SHT31 sht31_c = Adafruit_SHT31();
Adafruit_SHT31 sht31_d = Adafruit_SHT31();
#ifndef SHT31_BRIDGE_CLOSE_ID
// Set alternate i2c addr
#define SHT31_BRIDGE_CLOSE_ID 0x44
#endif
#ifndef SHT31_BRIDGE_OPEN_ID
// Set alternate i2c addr
#define SHT31_BRIDGE_OPEN_ID 0x45
#endif

SensorSht31::SensorSht31()
{
}

void SensorSht31::setup(unsigned long current)
{
    static unsigned long next_time = 0;
    static unsigned long interval  = 5000;
    if (current > next_time) {
        next_time = current + interval;
        if (false == this->enable_close) {
            if (true != sht31_c.begin(SHT31_BRIDGE_CLOSE_ID)) {
                log_v("could not find SHT31(bridge close)");
            } else {
                this->enable_close = true;
            }
        }
        if (false == this->enable_open) {
            if (true != sht31_d.begin(SHT31_BRIDGE_OPEN_ID)) {
                log_v("could not find SHT31(bridge open)");
            } else {
                this->enable_open = true;
            }
        }
        if (true == this->enable_close) {
            if (true == this->enable_open) {
                next_time = __LONG_MAX__;
            }
        }
#if 0
        next_time = __LONG_MAX__;
#endif
    }
}

void SensorSht31::loop(unsigned long current)
{
    if (true == this->enable_close) {
        float temperature_out_c;
        float humidity_out_c;
        sht31_c.readBoth(&temperature_out_c, &humidity_out_c);
        this->temperature_close->set(temperature_out_c, current);
        this->humidity_close->set(humidity_out_c, current);
    }
    if (true == this->enable_open) {
        float temperature_out;
        float humidity_out;
        sht31_d.readBoth(&temperature_out, &humidity_out);
        this->temperature_open->set(temperature_out, current);
        this->humidity_open->set(humidity_out, current);
    }
}
