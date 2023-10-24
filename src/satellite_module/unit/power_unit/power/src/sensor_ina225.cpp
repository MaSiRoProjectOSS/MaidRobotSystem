/**
 * @file sensor_ina225.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-04-02
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "sensor_ina225.hpp"

INA226 ina_mt(0x40); // 動力系
INA226 ina_ct(0x41); // 制御系

SensorIna225::SensorIna225()
{
}

void SensorIna225::setup(unsigned long current)
{
    static unsigned long next_time = 0;
    static unsigned long interval  = 5000;
    if (current > next_time) {
        next_time = current + interval;
        if (false == this->enable_mt) {
            if (true != ina_mt.begin()) {
                log_v("could not connect. Fix and Reboot");
            } else {
                this->enable_mt = true;
                ina_mt.setMaxCurrentShunt(1, 0.002);
            }
        }
        if (false == this->enable_ct) {
            if (true != ina_ct.begin()) {
                log_v("could not connect. Fix and Reboot");
            } else {
                this->enable_ct = true;
                ina_ct.setMaxCurrentShunt(1, 0.002);
            }
        }
        if (true == this->enable_mt) {
            if (true == this->enable_ct) {
                next_time = __LONG_MAX__;
            }
        }
#if 0
        next_time = __LONG_MAX__;
#endif
    }
}

void SensorIna225::loop(unsigned long current)
{
    if (true == this->enable_mt) {
        this->mt_busVoltage->set(ina_mt.getBusVoltage(), current);
        this->mt_shuntVoltage->set(ina_mt.getShuntVoltage_mV(), current);
        this->mt_current->set(ina_mt.getCurrent_mA(), current);
    }
    if (true == this->enable_ct) {
        this->send_ct_data = ina_ct.getBusVoltage();
        this->ct_busVoltage->set(this->send_ct_data, current);
        this->ct_shuntVoltage->set(ina_ct.getShuntVoltage_mV(), current);
        this->ct_current->set(ina_ct.getCurrent_mA(), current);
    }
}
