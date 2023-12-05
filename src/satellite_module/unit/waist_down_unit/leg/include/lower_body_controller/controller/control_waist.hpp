/**
 * @file control_waist.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief control the waist
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef LOWER_BODY_UNIT_CONTROL_WAIST_HPP
#define LOWER_BODY_UNIT_CONTROL_WAIST_HPP

#include "maid_robot_system/common/interface/control_if.hpp"
#include "maid_robot_system/common/types/coordinate_euler.hpp"

#include <Arduino.h>

namespace lower_body_unit
{
namespace controller
{

/**
 * @brief control the waist
 */
class ControlWaist : public ControlIF {
public:
    /**
     * @brief Construct a new Control Waist object
     *
     * @param pin_pitch
     */
    ControlWaist(uint8_t pin_pitch = __UINT8_MAX__);

protected:
    /**
     * @brief Start of processing
     * @attention Called when the public function "end()" is called.
     * @protected
     *
     * @return true : Accepted
     * @return false : Rejected
     */
    bool _begin() override;
    /**
     * @brief Stop of processing
     * @attention Called when the public function "end()" is called.
     * @protected
     *
     * @return true : Accepted
     * @return false : Rejected
     */
    bool _end() override;
    /**
     * @brief perform calculations
     * @attention Called when the public function "calculate()" is called.
     * @attention
     * @protected
     *
     * @return true : Accepted
     * @return false : Rejected
     */
    bool _calculate() override;

    /**
     * @brief Allow action
     * @attention Called when the public function "enable()" is called.
     *
     * @return true : Accepted
     * @return false : Rejected
     */
    bool _enable() override;

    bool _is_error() override;

public:
    /**
     * @brief Set the data from sensor
     *
     * @param center_pitch_pos : from leg motor
     */
    void set_data_from_sensor(int center_pitch_pos);
    /**
     * @brief Set the data from CAN
     *
     * @param request_pitch : from CAN
     */
    void set_data_from_CAN(int request_pitch);
    /**
     * @brief Set the data from gyro
     *
     * @param gyro : from gyro
     */
    void set_data_from_gyro(CoordinateEuler gyro);
    /**
     * @brief control the motor
     *
     */
    void send();

    /**
     * @brief Get the target pitch object
     *
     * @return int : return the indicated value
     */
    int get_target_pitch();

private:
    uint8_t _pin_pitch;      /*!< SETTING : pin No. */
    int _request_pitch;      /*!< request pitch from CAN */
    int _current_pitch_leg;  /*!< current pitch from Leg */
    int _current_pitch_gyro; /*!< current pitch from Gyro */
    int _correction_data;    /*!< Calculated result */

    TimeCheck _monitoring;                  /*!< receipt confirmation */
    const int MONITORING_INTERVAL_MS = 100; /*!< monitoring interval [ms] */

private:
    const int _PITCH_OFFSET = 1800; /*!< pitch offset */

    const long _SEND_IN_MIN  = 0;     /*!< send data rom range : low */
    const long _SEND_IN_MAX  = 18000; /*!< send data from range : High */
    const long _SEND_OUT_MIN = 80;    /*!< send data to range : low */
    const long _SEND_OUT_MAX = 245;   /*!< send data to range : High */

    const long _PITCH_IN_MIN  = 0;    /*!< request pitch from range : low */
    const long _PITCH_IN_MAX  = 1023; /*!< request pitch from range : High */
    const long _PITCH_OUT_MIN = 0;    /*!< request pitch to range : low */
    const long _PITCH_OUT_MAX = 6000; /*!< request pitch to range : High */
};

} // namespace controller
} // namespace lower_body_unit

#endif
