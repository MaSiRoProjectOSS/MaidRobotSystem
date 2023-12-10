/**
 * @file model_on_lower_body_controller.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Handling Wheel Controller
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "lower_body_controller/model_on_lower_body_controller.hpp"

#include "lower_body_controller/config.hpp"
#include "lower_body_controller/controller/control_communication.hpp"

#ifndef OUTPUT_LOG
#define OUTPUT_LOG 0
#endif

namespace lower_body_unit
{
controller::ControlCommunication ctrl_com(SETTING_PIN_COMMUNICATION_CAN_CS, //
                                          CONFIG_GYRO_SENSOR_ID,
                                          CONFIG_GYRO_ADDRESS);
controller::ControlWaist ctrl_waist(SETTING_PIN_WAIST_PITCH);
controller::ControlLegMotor ctrl_leg(SETTING_PIN_LEG_PWM_LEFT,
                                     SETTING_PIN_LEG_PWM_RIGHT,
                                     SETTING_PIN_LEG_POSITION,
                                     SETTING_PIN_STEPPER_SERVO_PLUS,
                                     SETTING_PIN_STEPPER_SERVO_DIR,
                                     SETTING_PIN_STEPPER_SERVO_ENABLE,
                                     SETTING_PIN_STEPPER_SERVO_ERROR,
                                     LEG_PWM_LIMIT,
                                     LEG_POSITION_MAX,
                                     LEG_POSITION_LIMIT,
                                     LEG_SENSOR_FW,
                                     LEG_MOTOR_FW);
CoordinateEuler gyro;

///////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////
ModelOnLowerBodyController::ModelOnLowerBodyController()
{
}

///////////////////////////////////////////////////////////
// protected function
///////////////////////////////////////////////////////////
bool ModelOnLowerBodyController::_setup()
{
    bool result = true;
    //////////////////////////////////////////////
    result = ctrl_leg.begin();
    if (true == result) {
        result = ctrl_waist.begin();
    }
    if (true == result) {
        result = ctrl_com.begin();
    }
    //////////////////////////////////////////////
    return result;
}

bool ModelOnLowerBodyController::_receive(ModeList mode)
{
    bool result = true;
    ctrl_com.receive(&ctrl_waist, &ctrl_leg, &gyro);
    return result;
}

bool ModelOnLowerBodyController::_calculate(ModeList mode)
{
    bool result = true;
    // -------------------------------------------
    // ctrl_leg
    // -------------------------------------------
    ctrl_leg.calculate();

    // -------------------------------------------
    // ctrl_waist
    // -------------------------------------------
    ctrl_waist.set_data_from_sensor(ctrl_leg.center_pitch_pos);
    ctrl_waist.calculate();
    return result;
}

void ModelOnLowerBodyController::_send(ModeList mode)
{
    ctrl_waist.send();
    ctrl_com.send(&ctrl_waist, &ctrl_leg);
}

void ModelOnLowerBodyController::_error_check(ModeList mode)
{
    bool flag_change = false;
    // ---------------------------------------------
    // check communication
    // ---------------------------------------------
    // if (false == flag_change)
    {
        if (true == ctrl_com.is_error()) {
            ctrl_com.restart();
            if (true == ctrl_com.is_error()) {
                this->_set_mode(ModeList::MODE_ERROR_NOT_COMMUNICATION);
                flag_change = true;
            }
        }
    }
    // ---------------------------------------------
    // check waist
    // ---------------------------------------------
    if (false == flag_change) {
        if (true == ctrl_waist.is_error()) {
            ctrl_waist.restart();
            if (true == ctrl_waist.is_error()) {
                this->_set_mode(ModeList::MODE_ERROR_NOT_COMMUNICATION);
                flag_change = true;
            }
        }
    }
    // ---------------------------------------------
    // check leg
    // ---------------------------------------------
    if (false == flag_change) {
        if (true == ctrl_leg.is_error()) {
            ctrl_leg.restart();
            if (true == ctrl_leg.is_error()) {
                this->_set_mode(ModeList::MODE_ERROR_NOT_COMMUNICATION);
                flag_change = true;
            }
        }
    }
    ///////////////////////////////////////////////////////
    if (false == flag_change) {
        this->_set_mode(ModeList::MODE_RUNNING);
    }
}

void ModelOnLowerBodyController::_debug_output(ModeList mode)
{
}

} // namespace lower_body_unit
