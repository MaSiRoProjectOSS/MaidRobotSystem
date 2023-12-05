/**
 * @file xiao_imu.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef XIAO_IMU_HPP
#define XIAO_IMU_HPP

#include "LSM6DS3.h"

#include <Arduino.h>

namespace XIAO
{
#ifndef XIAO_IMU_UNKNOWN_TEMP
#define XIAO_IMU_UNKNOWN_TEMP (-999)
#endif

class XIAOImu {
public:
    class Vector3 {
    public:
        Vector3(float x_axis = 0, float y_axis = 0, float z_axis = 0) : x(x_axis), y(y_axis), z(z_axis) {}
        void set(float x_axis = 0, float y_axis = 0, float z_axis = 0)
        {
            this->x = x_axis;
            this->y = y_axis;
            this->z = z_axis;
        }

        float x = 0;
        float y = 0;
        float z = 0;
    };

public:
    XIAOImu();

public:
    bool setup(uint8_t bus_type = I2C_MODE, uint8_t input_arg = 0x6A);
    bool update(void);

public:
    Vector3 get_accel();
    Vector3 get_gyro();
    float get_TempC();

private:
    Vector3 _accel;
    Vector3 _gyro;
    float _temp = XIAO_IMU_UNKNOWN_TEMP;
    bool _read_accel();
    bool _read_gyro();
    bool _read_TempC();

private:
    LSM6DS3 *_com_imu;
    const float _unknown_temp = XIAO_IMU_UNKNOWN_TEMP;
    bool _initialize          = false;
};

} // namespace XIAO

#endif
