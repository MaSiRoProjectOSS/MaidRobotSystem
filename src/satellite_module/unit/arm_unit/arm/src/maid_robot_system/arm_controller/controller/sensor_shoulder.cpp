/**
 * @file sensor_shoulder.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-05-02
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/controller/sensor_shoulder.hpp"

#include <Arduino.h>

namespace maid_robot_system
{
namespace arm_unit
{

SensorShoulder::SensorShoulder()
{
}

void SensorShoulder::sensor_shoulder(PostureManagerArguments *args)
{
    this->set_posture_manager_address(args);
    this->_calculate();
    this->get_posture_manager_address(args);
}

void SensorShoulder::set_posture_manager_address(PostureManagerArguments *args)
{
    this->_PostureManager_data = args;
}

void SensorShoulder::get_posture_manager_address(PostureManagerArguments *args)
{
    args = this->_PostureManager_data;
}

bool SensorShoulder::_calculate()
{
#ifdef CIYA
    int in_x = this->_X_MIN_OF_CIYA;
#elif CIRO
    int in_x = this->_X_MIN_OF_CIRO;
#endif
    this->_PostureManager_data->input.shoulder_left_push
            = constrain(map(analogRead(PIN_SHOULDER_PUSH_SENSOR), in_x, this->_X_MAX, this->_Y_MIN, this->_Y_MAX), this->_Y_MIN, this->_Y_MAX) / this->_PERCENT_TO_RATE;

    return true;
}

bool SensorShoulder::_begin()
{
    return true;
}
bool SensorShoulder::_end()
{
    return true;
}

} // namespace arm_unit
} // namespace maid_robot_system
