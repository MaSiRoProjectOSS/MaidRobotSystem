/**
 * @file sensor_sht31.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-04-02
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef SENSOR_SHT31_HPP_
#define SENSOR_SHT31_HPP_

#include <Adafruit_SHT31.h>
#include <maid_robot_system/common/chart/candle_stick.hpp>

class SensorSht31 {
public:
    SensorSht31();

public:
    void setup(unsigned long current);
    void loop(unsigned long current);

public:
    bool enable_open  = false;
    bool enable_close = false;

    const int TEMPERATURE_INTERVAL = 1000 * 60;
    const int HUMIDITY_INTERVAL    = 1000 * 120;
    CandleStick *temperature_open  = new CandleStick(TEMPERATURE_INTERVAL);
    CandleStick *humidity_open     = new CandleStick(HUMIDITY_INTERVAL);
    CandleStick *temperature_close = new CandleStick(TEMPERATURE_INTERVAL);
    CandleStick *humidity_close    = new CandleStick(HUMIDITY_INTERVAL);
};

#endif /* SENSOR_SHT32_HPP_ */
