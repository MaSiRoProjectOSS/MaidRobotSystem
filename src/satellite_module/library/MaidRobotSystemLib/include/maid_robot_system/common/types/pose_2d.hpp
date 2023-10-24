/**
 * @file pose_2d.hpp
 * @author Akari (masiro.to.akari@gmail.com)
* @brief A structure that manages Pose 2D
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_COMMON_TYPES_POSE_2D_HPP
#define MAID_ROBOT_SYSTEM_LIB_COMMON_TYPES_POSE_2D_HPP

namespace maid_robot_system
{
namespace common
{

/**
 * @brief A structure that manages Pose 2D
 *
 */
class Pose2D {
public:
    /**
     * @brief Construct a new Pose 2D object
     *
     * @param x_axis
     * @param y_axis
     * @param yaw_angle
     */
    Pose2D(float x_axis = 0, float y_axis = 0, float yaw_angle = 0);
    /**
     * @brief set value
     *
     * @param x_axis
     * @param y_axis
     * @param yaw_angle
     */
    void set(float x_axis, float y_axis, float yaw_angle);

public:
    float x   = 0; /*!< x axis */
    float y   = 0; /*!< y axis */
    float yaw = 0; /*!< Rotation around vertical axis */
};

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
