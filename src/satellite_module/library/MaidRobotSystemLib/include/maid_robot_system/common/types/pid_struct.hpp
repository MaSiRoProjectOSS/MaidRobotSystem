/**
 * @file pid_struct.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief A structure that manages PID
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_COMMON_TYPES_PID_STRUCT_HPP
#define MAID_ROBOT_SYSTEM_LIB_COMMON_TYPES_PID_STRUCT_HPP

namespace maid_robot_system
{
namespace common
{

/**
 * @brief A structure that manages PID
 *
 */
class PIDstruct {
public:
    /**
     * @brief Construct a new PIDstruct object
     *
     * @param proportional
     * @param integral
     * @param differential
     */
    PIDstruct(float proportional = 0, float integral = 0, float differential = 0);

    /**
     * @brief set value
     *
     * @param proportional
     * @param integral
     * @param differential
     */
    void set(float proportional, float integral, float differential);

public:
    float P = 0; /*!< proportional */
    float I = 0; /*!< integral */
    float D = 0; /*!< differential */

    float previous_P = 0; /*!< previous proportional */
};

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
