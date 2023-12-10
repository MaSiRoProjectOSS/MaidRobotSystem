/**
 * @file driver_gyro.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Operate Gyro device
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "wheel_controller/controller/driver/driver_gyro.hpp"

namespace mobility_unit
{
namespace driver
{

DriverGyro::DriverGyro(int32_t sensorID, uint8_t address)
{
    this->_flag_initialized = false;
    this->_temperature      = 0;

    this->_bno = new Adafruit_BNO055(sensorID, address);
}

bool DriverGyro::setup()
{
    if (false == this->_flag_initialized) {
        if (NULL != this->_bno) {
            bool result = this->_bno->begin();
            if (true == result) {
                this->_bno->setExtCrystalUse(true);
                this->_intermittent_short.from_scratch();
                this->_intermittent_long.from_scratch();
                this->receive();
                this->_flag_initialized = true;
            }
        }
    }
    return this->_flag_initialized;
}

bool DriverGyro::restart()
{
    this->_flag_initialized = false;
    return this->setup();
}

void DriverGyro::receive()
{
    if (true == this->_flag_initialized) {
        if (true == this->_intermittent_short.check_passing(this->INTERMITTENT_SHORT_MS)) {
            imu::Vector<3> euler = this->_bno->getVector(Adafruit_BNO055::VECTOR_EULER);
#if 0
            static float previous_yaw = 0;
            float rotation;
            float d_yaw  = previous_yaw - euler.x();
            previous_yaw = euler.x();

            // TODO : 天地についてつかってる？
            if (d_yaw > 300) {
                rotation += 1;
            }
            if (d_yaw < -300) {
                rotation -= 1;
            }

            this->_current.yaw   = euler.x() + rotation * 360.0;
            this->_current.pitch = euler.z();
            this->_current.roll  = euler.y();
#else
            this->_current.yaw   = euler.x();
            this->_current.pitch = euler.z();
            this->_current.roll  = euler.y();
#endif
        }

        if (true == this->_intermittent_long.check_passing(this->INTERMITTENT_LONG_MS)) {
            // TODO: 温度取得いる？
            this->_temperature = this->_bno->getTemp();
        }
    }
}

CoordinateEuler DriverGyro::get()
{
    return this->_current;
}

} // namespace driver
} // namespace mobility_unit
