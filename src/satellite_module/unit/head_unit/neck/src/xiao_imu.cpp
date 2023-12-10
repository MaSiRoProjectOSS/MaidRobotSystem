/**
 * @file xiao_expansion_imu.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "xiao_imu.hpp"

namespace XIAO
{

XIAOImu::XIAOImu()
{
}

///////////////////////////////////////////////////////////////
// pubic
///////////////////////////////////////////////////////////////
bool XIAOImu::setup(uint8_t bus_type, uint8_t input_arg)
{
    bool result = false;
    if (NULL == this->_com_imu) {
        this->_com_imu = new LSM6DS3(bus_type, input_arg);
    }
    if (NULL != this->_com_imu) {
        if (0 == this->_com_imu->begin()) {
            result = true;
        }
    }
    this->_initialize = result;
    return this->_initialize;
}
bool XIAOImu::update(void)
{
    int result = 0;
    if (true == this->_initialize) {
        result += (true == this->_read_accel()) ? 1 : 0;
        result += (true == this->_read_gyro()) ? 1 : 0;
        result += (true == this->_read_TempC()) ? 1 : 0;
    }
    return (3 <= result) ? true : false;
}

///////////////////////////////////////////////////////////////
// getter
///////////////////////////////////////////////////////////////
XIAOImu::Vector3 XIAOImu::get_accel()
{
    return this->_accel;
}
XIAOImu::Vector3 XIAOImu::get_gyro()
{
    return this->_gyro;
}
float XIAOImu::get_TempC()
{
    return this->_temp;
}

///////////////////////////////////////////////////////////////
// private
///////////////////////////////////////////////////////////////
bool XIAOImu::_read_accel()
{
    bool result = false;
    if (true == this->_initialize) {
        this->_accel.set( //
                this->_com_imu->readFloatAccelX(),
                this->_com_imu->readFloatAccelY(),
                this->_com_imu->readFloatAccelZ());
        result = true;
    }
    return result;
}
bool XIAOImu::_read_gyro()
{
    bool result = false;
    if (true == this->_initialize) {
        this->_gyro.set( //
                this->_com_imu->readFloatGyroX(),
                this->_com_imu->readFloatGyroY(),
                this->_com_imu->readFloatGyroZ());
        result = true;
    }
    return result;
}
bool XIAOImu::_read_TempC()
{
    bool result = false;
    if (true == this->_initialize) {
        this->_temp = this->_com_imu->readTempC();
        result      = true;
    }
    return result;
}

} // namespace XIAO
