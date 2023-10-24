/**
 * @file control_leg_motor.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief control the leg
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MOBILITY_UNIT_CONTROL_LEG_MOTOR_HPP
#define MOBILITY_UNIT_CONTROL_LEG_MOTOR_HPP

#include "common/interface/control_if.hpp"
#include "wheel_controller/controller/driver/driver_stepper_motor.hpp"

namespace mobility_unit
{
namespace controller
{

/**
 * @brief control the leg
 *
 */
class ControlLegMotor : public ControlIF {
public:
    ControlLegMotor(int pin_left,
                    int pin_right,
                    int pin_sensor,
                    int pin_servo_plus,
                    int pin_servo_dir,
                    int pin_servo_enable,
                    int pin_servo_error,
                    int pwm_limit,
                    int position_max,
                    const int position_limit[2],
                    char fw_rw_sensor,
                    char fw_rw_motor);

protected:
    bool _begin() override;
    bool _end() override;
    bool _calculate() override;

    bool _is_error() override;

public:
    int center_pitch_pos = 7000; /*!<  */
    int leg_target_pos   = 0;    /*!<  */

public:
    void set_data(int flag_smart_move, int position, int speed);
    void set_data(float step_height);
    int get_height_percentage();

    bool is_alive();

    void debug_output();

private:
    driver::DriverStepperMotor *_mt; /*!<  */
    TimeCheck _heartbeat;            /*!<  */

    bool _leg_data_get_flag = false; /*!<  */
    float _step_height      = 0;     /*!<  */
    int _leg_position_limit[2];      /*!<  */
    int _leg_position_max = 190;     /*!<  */

private:
    const int TIMEOUT_MS = 2500; /*!<  */
};

} // namespace controller
} // namespace mobility_unit

#endif
