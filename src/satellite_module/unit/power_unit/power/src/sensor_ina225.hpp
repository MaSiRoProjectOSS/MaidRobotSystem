/**
 * @file sensor_ina225.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief 電流電圧センサー
 * @version 0.23.1
 * @date 2023-04-02
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef SENSOR_INA225_HPP_
#define SENSOR_INA225_HPP_

#include <INA226.h>
#include <maid_robot_system/common/chart/candle_stick.hpp>

class SensorIna225 {
public:
    SensorIna225();

public:
    void setup(unsigned long current);
    void loop(unsigned long current);

public:
    bool enable_mt = false;
    bool enable_ct = false;

    const int VOLTAGE_INTERVAL   = 1000 * 30;
    CandleStick *mt_busVoltage   = new CandleStick(VOLTAGE_INTERVAL);
    CandleStick *mt_shuntVoltage = new CandleStick(VOLTAGE_INTERVAL);
    CandleStick *mt_current      = new CandleStick(VOLTAGE_INTERVAL);

    CandleStick *ct_busVoltage   = new CandleStick(VOLTAGE_INTERVAL);
    CandleStick *ct_shuntVoltage = new CandleStick(VOLTAGE_INTERVAL);
    CandleStick *ct_current      = new CandleStick(VOLTAGE_INTERVAL);

    float send_ct_data = 0.0;
};

#endif /* SENSOR_INA225_HPP_ */
