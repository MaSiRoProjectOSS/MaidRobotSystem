/**
 * @file control_wheel.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief control the wheel
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MOBILITY_UNIT_CONTROL_WHEEL_HPP
#define MOBILITY_UNIT_CONTROL_WHEEL_HPP

#include "maid_robot_system/common/interface/control_if.hpp"
#include "wheel_controller/controller/driver/driver_roboclaw.hpp"

namespace mobility_unit
{
namespace controller
{

/**
 * @brief control the wheel
 *
 */
class ControlWheel : public ControlIF {
public:
    ///////////////////////////////////////////////////////////
    // Constructor
    ///////////////////////////////////////////////////////////
    ControlWheel(HardwareSerial *h_serial, uint8_t pin_enable, uint8_t id, int revolution_left, int revolution_right);
#ifdef __AVR__
    ControlWheel(SoftwareSerial *s_serial, uint8_t pin_enable, uint8_t id, int revolution_left, int revolution_right);
#endif

protected:
    ///////////////////////////////////////////////////////////
    // Inherited function
    ///////////////////////////////////////////////////////////
    bool _begin() override;
    bool _end() override;
    bool _calculate() override;

    bool _is_error() override;

public:
    ///////////////////////////////////////////////////////////
    // Setter / Getter
    ///////////////////////////////////////////////////////////
    void set_data(unsigned char rec_data[8]);
    int get_step_percentage();

private:
    ///////////////////////////////////////////////////////////
    // private function
    ///////////////////////////////////////////////////////////
    void _resident();
    void _get_motor_status();

public:
    ///////////////////////////////////////////////////////////
    // Exposed variables
    ///////////////////////////////////////////////////////////

    int battery_voltage = 0; /*!<  */
    int32_t motor1_rps  = 0; /*!<  */
    int32_t motor2_rps  = 0; /*!<  */

    float step_height = 50;          /*!< 歩行高さ */
    float step_speed  = 0;           /*!< 歩速度 [mm/s] */
    float step_period = 2000;        /*!< 歩行周期 */
    float step_stride = 1250 * 0.25; /*!< 歩幅 [mm] */

private:
    ///////////////////////////////////////////////////////////
    // Private variables
    ///////////////////////////////////////////////////////////
    uint8_t _pin_enable; /*!<  */

    driver::DriverRoboclaw *_roboclaw; /*!<  */

    float _now_v         = 0; /*!<  */
    float _now_w         = 0; /*!<  */
    int _motor_mode      = 0; /*!<  */
    float _target_v      = 0; /*!<  */
    float _target_w      = 0; /*!<  */
    int _step_percentage = 0; /*!<  */

    int _revolution_left  = 1; /*!<  */
    int _revolution_right = 1; /*!<  */

private:
    ///////////////////////////////////////////////////////////
    // Constant
    ///////////////////////////////////////////////////////////
    const double STEP_HEIGHT_MAX = 130; /*!<  */

    const int MIN_STEP_HEIGHT   = 30;    /*!<  */
    const double WHEEL_TREAD    = 250.0; /*!< mm */
    const double WHEEL_DIAMETER = 200.0; /*!< mm */

    const int CONTROL_WHEEL_MODE_ROBOCLAW = 122; /*!<  */

    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////

public:
    void setup_pid(char m_fw_rw);
    float set_target_motor_rps(float rps);
    int set_motor_info(int16_t velo, int16_t position, int16_t temp, int16_t trq_current);

public:
    float target_rps;           /*!<  */
    int target_pwm         = 0; /*!<  */
    float target_motor_rps = 0; /*!<  */
    float now_rps;              /*!<  */

private:
    float gear_ratio = 64.0 * 70.0 * (54.0 / 25.0); /*!< POLORU */

    char motor_fw_rw;        /*!<  */
    double now_motor_rps;    /*!<  */
    int now_pos;             /*!<  */
    int now_temp;            /*!<  */
    float raw_rps;           /*!<  */
    int state = STATE_ERROR; /*!<  */

public:
    const int STATE_OK    = 10; /*!<  */
    const int STATE_ERROR = -1; /*!<  */
    const int STATE_STOP  = 0;  /*!<  */
};

} // namespace controller
} // namespace mobility_unit

#endif
