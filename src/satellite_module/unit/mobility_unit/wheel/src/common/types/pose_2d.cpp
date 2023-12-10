/**
 * @file pose_2d.cpp
 * @author Akari (masiro.to.akari@gmail.com)
* @brief A structure that manages Pose 2D
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "common/types/pose_2d.hpp"

Pose2D::Pose2D(float x_axis, float y_axis, float yaw_angle)
{
    this->set(x_axis, y_axis, yaw_angle);
}

void Pose2D::set(float x_axis, float y_axis, float yaw_angle)
{
    this->x   = x_axis;
    this->y   = y_axis;
    this->yaw = yaw_angle;
}
