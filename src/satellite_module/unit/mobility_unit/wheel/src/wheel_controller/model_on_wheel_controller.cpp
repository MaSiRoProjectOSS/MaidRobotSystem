/**
 * @file model_on_wheel_controller.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Handling Wheel Controller
 * @version 0.1
 * @date 2023-02-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "wheel_controller/model_on_wheel_controller.hpp"

#include "wheel_controller/config.hpp"
#include "wheel_controller/controller/control_communication.hpp"

#ifndef OUTPUT_LOG
#define OUTPUT_LOG 0
#endif
#ifndef OUTPUT_DEBUG_MSG
#define OUTPUT_DEBUG_MSG 0
#endif

namespace mobility_unit
{
controller::ControlWheel ctrl_wheel(&Serial1, //
                                    SETTING_PIN_WHEEL_ENABLE,
                                    CONFIG_ROBOCLAW_ID,
                                    CONFIG_WHEEL_MOTOR_REVOLUTION_LEFT,
                                    CONFIG_WHEEL_MOTOR_REVOLUTION_RIGHT);
controller::ControlCommunication ctrl_com(SETTING_PIN_COMMUNICATION_CAN_CS, //
                                          CONFIG_GYRO_SENSOR_ID,
                                          CONFIG_GYRO_ADDRESS);

// TimeCheck loop_timer;
// TimeCheck cycle_timer;
// TimeCheck stepper_drive;

///////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////
ModelOnWheelController::ModelOnWheelController()
{
}

///////////////////////////////////////////////////////////
// protected function

///////////////////////////////////////////////////////////
bool ModelOnWheelController::_setup()
{
    bool result = true;
    //////////////////////////////////////////////
    // Serial.begin(230400);
    //////////////////////////////////////////////
    result = ctrl_wheel.begin();
    if (true == result) {
        result = ctrl_com.begin();
    }
    //////////////////////////////////////////////
    if (false == result) {
        LOG_ERROR("setup() : not initialized");
    }
    //////////////////////////////////////////////

    return result;
}

bool ModelOnWheelController::_receive(ModeList mode)
{
    bool result = true;
    ctrl_com.receive(&ctrl_wheel);
    return result;
}

bool ModelOnWheelController::_calculate(ModeList mode)
{
    bool result = true;
    // -------------------------------------------
    // ctrl_wheel
    // -------------------------------------------
    ctrl_wheel.calculate();

    return result;
}

void ModelOnWheelController::_send(ModeList mode)
{
    ctrl_com.send(&ctrl_wheel);
}

void ModelOnWheelController::_error_check(ModeList mode)
{
    bool flag_change = false;
    // ---------------------------------------------
    // check communication
    // ---------------------------------------------
    if (false == flag_change) {
        if (true == ctrl_com.is_error()) {
            ctrl_com.restart();
            if (true == ctrl_com.is_error()) {
                this->_set_mode(ModeList::MODE_ERROR_NOT_COMMUNICATION);
                flag_change = true;
            }
        }
    }
    // ---------------------------------------------
    // check wheel
    // ---------------------------------------------
    if (false == flag_change) {
        if (true == ctrl_wheel.is_error()) {
            ctrl_wheel.restart();
            if (true == ctrl_wheel.is_error()) {
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

void ModelOnWheelController::_debug_output(ModeList mode)
{
#if OUTPUT_LOG
    // debug print
    static TimeCheck serial_timer;
    if (true == serial_timer.check_passing(100)) {
        char buffer[255];
#if OUTPUT_DEBUG_MSG
        sprintf(buffer, //
                "[%d] %s %s"
                "\t leg_sen: %3d"
                "\t leg_now: %3d,"
                "\t leg_tar: %3d,"
                "\t leg_target_speed: %0.3f,"
                "\t speed: %3d\t freq: %0.3f",
                this->_mode,
                (true == ctrl_wheel.is_enable()) ? "<WHEEL ON >" : "[WHEEL OFF]",
                (true == ctrl_waist.is_enable()) ? "<LEG ON >" : "[LEG OFF]",
                ctrl_leg.sensor_data,
                ctrl_leg.now_pos,
                ctrl_leg.target_pos,
                ctrl_leg.leg_target_speed,
                ctrl_leg.set_speed,
                ctrl_leg.output_freq
                //
        );
        logger.log_message(buffer);
#elif 0
        sprintf(buffer, "%3d %3d", ctrl_leg.now_pos, ctrl_leg.leg_target_pos);
        logger.log_message(buffer);
#else
#endif
    }
#endif

#if OUTPUT_LOG
    // debug print
    static TimeCheck serial_timer;
    if (true == serial_timer.check_passing(this->INTERVAL_CHECK_ERROR)) {
        char buffer[255];
    }

    if (true == serial_timer.check_passing(100)) {
        //if(serial_timer.check_passing(100)){

        //  if(1){
#ifdef OUTPUT_DEBUG_MSG

        if (true == ctrl_wheel.is_enable()) {
            LOG_DEBUG("=WHEEL ON= ");
        } else {
            LOG_DEBUG("[WHEEL OFF] ");
        }
        if (true == ctrl_leg.is_enable()) {
            LOG_DEBUG("=LEG ON= ");
        } else {
            LOG_DEBUG("[LEG OFF] ");
        }
        /*
      LOG_DEBUG("\tx: ");      LOG_DEBUG(wheel.now_pos.x);
      LOG_DEBUG("\ty: ");      LOG_DEBUG(wheel.now_pos.y);
      LOG_DEBUG("\ts: ");      LOG_DEBUG(wheel.now_pos.yaw);

      LOG_DEBUG("\tgy_y: ");      LOG_DEBUG(wheel.gy_yaw);
      LOG_DEBUG("\tgy_p: ");      LOG_DEBUG(wheel.gy_pitch);
      LOG_DEBUG("\tgy_r: ");      LOG_DEBUG(wheel.gy_roll);




      LOG_DEBUG("\tnR: ");
      LOG_DEBUG(wheel.wheel_R_motor.now_pos);
      LOG_DEBUG("\tnL: ");
      LOG_DEBUG(wheel.wheel_L_motor.now_pos);

*/
        /*
      LOG_DEBUG("\t now_v: ");
      LOG_DEBUG(wheel.now_v);
      LOG_DEBUG("\t now_w: ");
      LOG_DEBUG(wheel.now_w);

      LOG_DEBUG("\t tar_v: ");
      LOG_DEBUG(wheel.target_v);
      LOG_DEBUG("\t tar_w: ");
      LOG_DEBUG(wheel.target_w);
*/
        /*
      LOG_DEBUG(wheel.now_battery_voltage);  LOG_DEBUG(" ,: ");
      LOG_DEBUG(wheel.wheel_R_motor.now_rps*100.0);  LOG_DEBUG(" ,: ");
      LOG_DEBUG(wheel.wheel_L_motor.now_rps*100.0);  LOG_DEBUG(" ,: ");
      LOG_DEBUG(wheel.wheel_R_motor.target_rps*100.0);  LOG_DEBUG(" ,: ");
      LOG_DEBUG(wheel.wheel_L_motor.target_rps*100.0);  LOG_DEBUG(" ,: ");
  */
        //    LOG_DEBUG(wheel.wheel_R_motor.final_output/100.0);  LOG_DEBUG(" ,: ");
        //    LOG_DEBUG(wheel.wheel_L_motor.final_output);  LOG_DEBUG(" ,: ");

        // LOG_DEBUG("\t leg_now_pos: ");
        //    LOG_DEBUG(leg_target_pos);  LOG_DEBUG(",");
        //   LOG_DEBUG("\t leg_now_pos: ");
        //   LOG_DEBUG(leg_motor.now_percentage);LOG_DEBUG(",");
        ctrl_leg.debug_output();

        //    LOG_DEBUG(send_pitch_pos);       LOG_DEBUG(" sen, ");
        //    LOG_DEBUG(ctrl_waist.waist_target_pitch_pos);       LOG_DEBUG(" tar, ");

        //     LOG_DEBUG(gyro_stabilize_pitch_pos);       LOG_DEBUG(" gy, ");

        /*

//      LOG_DEBUG("\t period_timer: ");
//      LOG_DEBUG(step_period_timer.get_elapsed_time());
      LOG_DEBUG("\t step_period: ");
      LOG_DEBUG(target_step_period);



      LOG_DEBUG("\t_speed: ");
      LOG_DEBUG(step_speed);
      LOG_DEBUG("\t _stride: ");
      LOG_DEBUG(step_stride);

      LOG_DEBUG("\tnow_step: ");
      LOG_DEBUG(now_step_height);





      LOG_DEBUG(leg_motor.now_pos);
*/
        //wheel.get_gyro();

#endif
#ifdef graph_debug
        LOG_DEBUG(leg_motor.now_pos);
        LOG_DEBUG(" , ");
        LOG_DEBUG(leg_motor.target_pos);
        /*
      LOG_DEBUG(wheel.now_v);
      LOG_DEBUG("\t , ");
      LOG_DEBUG(wheel.now_w);
      LOG_DEBUG("\t , ");
      LOG_DEBUG(wheel.target_v);
      LOG_DEBUG("\t , ");
      LOG_DEBUG(wheel.target_w);
*/
#endif
        LOG_DEBUG("\t\t\n");
    }
#endif
}

} // namespace mobility_unit
