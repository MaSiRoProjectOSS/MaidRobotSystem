/**
 * @file coordinate_euler.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief A structure that manages coordinates
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "common/types/coordinate_euler.hpp"

CoordinateEuler::CoordinateEuler(float pitch_angle, float roll_angle, float yaw_angle)
{
    this->set(pitch_angle, roll_angle, yaw_angle);
}

void CoordinateEuler::set(float pitch_angle, float roll_angle, float yaw_angle)
{
    this->pitch = pitch_angle;
    this->roll  = roll_angle;
    this->yaw   = yaw_angle;
}
