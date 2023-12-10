/**
 * @file coordinate_euler.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief A structure that manages coordinates
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_COMMON_TYPES_COORDINATE_EULER_HPP
#define MAID_ROBOT_SYSTEM_LIB_COMMON_TYPES_COORDINATE_EULER_HPP

namespace maid_robot_system
{
namespace common
{

/**
 * @brief A structure that manages coordinates
 *
 */
class CoordinateEuler {
public:
    /**
     * @brief Construct a new Coordinate Euler object
     *
     * @param pitch_axis
     * @param roll_axis
     * @param yaw_angle
     */
    CoordinateEuler(float pitch_axis = 0, float roll_axis = 0, float yaw_angle = 0);

    /**
     * @brief set value
     *
     * @param pitch_angle
     * @param roll_angle
     * @param yaw_angle
     */
    void set(float pitch_angle, float roll_angle, float yaw_angle);

public:
    float pitch = 0; /*!< Front-back tilt */
    float roll  = 0; /*!< Horizontal tilt */
    float yaw   = 0; /*!< Rotation around vertical axis */
};

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
