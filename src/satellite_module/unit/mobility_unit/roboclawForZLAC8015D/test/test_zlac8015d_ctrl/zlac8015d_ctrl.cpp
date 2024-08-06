/**
 * @file native_test.cpp
 * @author Akari-mobility (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-07
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
///////////////////////////////////////////////////////////////////
#ifndef COMMON_CONSTANT
#define COMMON_CONSTANT 1
#endif
#ifndef MOTOR_PARAMETER
#define MOTOR_PARAMETER 1
#endif
#ifndef CONTROL_PARAMETER
#define CONTROL_PARAMETER 1
#endif
#ifndef READ_ONLY_PARAMETER
#define READ_ONLY_PARAMETER 1
#endif

///////////////////////////////////////////////////////////////////
#ifndef ZLAC_DRIVER_VERSION
#define ZLAC_DRIVER_VERSION 23423
#endif
///////////////////////////////////////////////////////////////////
#include <M5Atom.h>
#include <unity.h>
////
#include "driver_zlac/config_zlac.hpp"
#include "driver_zlac/zlac8015d_modbus.hpp"

ZLAC8015DCtrl ctrl;
static CRGB request_color = CRGB::Black;

void m5_led(CRGB color)
{
    static CRGB current_color = CRGB::Black;
    if (color != current_color) {
        current_color = color;
        (void)M5.dis.fillpix(current_color);
    }
}
void m5_led_request(CRGB color)
{
    request_color = color;
}
void setup_m5()
{
    bool enable_serial  = true;
    bool enable_i2c     = false;
    bool enable_display = true;
    (void)M5.begin(enable_serial, enable_i2c, enable_display);
    (void)M5.dis.begin();
    m5_led(CRGB::White);
    delay(200);
    m5_led(CRGB::Black);
    delay(200);
    m5_led(CRGB::White);
    delay(200);
    m5_led(CRGB::Black);
    delay(200);
    m5_led(CRGB::White);
    log_d("========================================");
    log_d("M5Atom initialized.");
    log_d("  - Start Modbus. Address[%d]", MODBUS_ADDRESS);
    bool flag = ctrl.begin(&Serial1, MODBUS_ADDRESS, MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU);
    log_d("========================================");
    TEST_ASSERT_TRUE(flag);
}

///////////////////////////////////////////////////////////////////
String text_drive_mode(ZLAC::DRIVER_MODE mode)
{
    switch (mode) {
        case ZLAC::DRIVER_MODE::POSITION_RELATIVE:
            return "POSITION_RELATIVE";
        case ZLAC::DRIVER_MODE::POSITION_ABSOLUTE:
            return "POSITION_ABSOLUTE";
        case ZLAC::DRIVER_MODE::VELOCITY:
            return "VELOCITY";
        case ZLAC::DRIVER_MODE::TORQUE:
            return "TORQUE";
        default:
            return "UNKNOWN";
    }
}
String text_can_baud_rate(ZLAC8015DCtrl::CAN_BAUD_RATE baud)
{
    switch (baud) {
        case ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_1000K:
            return "1000K";
        case ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_500K:
            return "500K";
        case ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_250K:
            return "250K";
        case ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_125K:
            return "125K";
        case ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_100K:
            return "100K";
        case ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_50K:
            return " 50K";
        case ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_25K:
            return " 25K";
        default:
            return "INVALID";
    }
}

///////////////////////////////////////////////////////////////////
void setUp(void)
{
}

void tearDown(void)
{
    // clean stuff up here
    m5_led(CRGB::Black);
}

/////////////////////////////////////////////////
// Common constant for Left and Right motors
/////////////////////////////////////////////////
#if COMMON_CONSTANT
void Communication_offline_time(void)
{
    // bool set_communication_offline_time(int value_ms, bool check = false)
    // TODO
    int value   = 0;
    bool result = ctrl.get_communication_offline_time(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Communication_offline_time : %d", value);
}
void RS485_Node_ID(void)
{
    // bool set_rs485_node_id(int id, bool check = false)
    // TODO

    int value   = 0;
    bool result = ctrl.get_rs485_node_id(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* RS485_Node_ID : %d", value);

    TEST_ASSERT_EQUAL_INT(1, value);
}
void RS485_Baud_Rate(void)
{
    // bool set_rs485_baud_rate(RS485_BAUD_RATE baud, bool check = false)

    // TODO
    ZLAC8015DCtrl::RS485_BAUD_RATE value;
    bool result = ctrl.get_rs485_baud_rate(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* RS485_Baud_Rate : [%d]", value);
}
void Input_signal_status(void)
{
    // TODO
    bool x0     = false;
    bool x1     = false;
    bool result = ctrl.get_input_signal_status(&x0, &x1);
    TEST_ASSERT_TRUE(result);
    log_d("* Input_signal_status : x0[%s]x1[%s]", x0 ? "T" : "F", x1 ? "T" : "F");
}
void Out_signal_status(void)
{
    // TODO
    bool x0     = false;
    bool x1     = false;
    bool result = ctrl.get_out_signal_status(&x0, &x1);
    TEST_ASSERT_TRUE(result);
    log_d("* Out_signal_status : x0[%d]x1[%d]", x0 ? "T" : "F", x1 ? "T" : "F");
}
void Clear_feedback_position(void)
{
    // bool set_clear_feedback_position(ZLAC::target_motor target, bool check = false)

    // TODO
    ZLAC::target_motor value;
    bool result = ctrl.get_clear_feedback_position(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Clear_feedback_position : [%d]", value);
}
void In_absolute_position_control_reset_the_zero_point(void)
{
    // bool set_reset_the_zero_point_in_absolute_position_control(ZLAC::target_motor target, bool check = false)

    // TODO
    ZLAC::target_motor value;
    bool result = ctrl.get_reset_the_zero_point_in_absolute_position_control(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* In_absolute_position_control_reset_the_zero_point : [%d]", value);
}
void Shaft_state_after_power_on(void)
{
    // bool set_shaft_state_after_power_on(bool lock_shaft, bool check = false)

    // TODO
    bool flag   = false;
    bool result = ctrl.get_shaft_state_after_power_on(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Shaft_state_after_power_on : %s", flag ? "T:lock" : "F:unlock");
}
void Maximum_motor_speed(void)
{
    // bool set_maximum_motor_speed(int r_min, bool check = false)

    // TODO
    int value   = 0;
    bool result = ctrl.get_maximum_motor_speed(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Maximum_motor_speed : %d", value);
}
void Register_parameter_settings(void)
{
    // bool set_register_parameter_settings(bool restore_factory_settings, bool check = false)

    // TODO
    bool flag   = false;
    bool result = ctrl.get_register_parameter_settings(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Register_parameter_settings : %s", flag ? "T" : "F");
}
void CAN_Node_info(void)
{
    // bool set_can_node_info(int id, CAN_BAUD_RATE baud, bool check = false)

    // TODO
    int id = 0;
    ZLAC8015DCtrl::CAN_BAUD_RATE baud;
    bool result = ctrl.get_can_node_info(&id, &baud);
    TEST_ASSERT_TRUE(result);
    log_d("* CAN_Node_info : id[%d]baud[%d]", id, baud);

    bool result;
    int backup_id = 0;
    ZLAC8015DCtrl::CAN_BAUD_RATE backup_baud;
    //
    result = ctrl.get_can_node_info(&backup_id, &backup_baud);
    TEST_ASSERT_TRUE(result);
    log_d("* CAN_Node_info : id[%d]baud[%s]", backup_id, text_can_baud_rate(backup_baud));

    result = ctrl.set_can_node_info(backup_id, backup_baud);
    TEST_ASSERT_TRUE(result);
}
void Control_mode(void)
{
    bool result;
    ZLAC::DRIVER_MODE backup_mode;
    ZLAC::DRIVER_MODE mode;
    result = ctrl.get_control_mode(&backup_mode);
    TEST_ASSERT_TRUE(result);
    log_d("* Control_mode : NO.1 [%s]", text_drive_mode(backup_mode).c_str());

    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_RELATIVE, true);
    TEST_ASSERT_TRUE_MESSAGE(result, "set_control_mode : POSITION_RELATIVE");
    result = ctrl.get_control_mode(&mode);
    TEST_ASSERT_EQUAL_MESSAGE(ZLAC::DRIVER_MODE::POSITION_RELATIVE, mode, "get_control_mode : POSITION_RELATIVE");
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_ABSOLUTE, true);
    TEST_ASSERT_TRUE_MESSAGE(result, "set_control_mode : POSITION_ABSOLUTE");
    result = ctrl.get_control_mode(&mode);
    TEST_ASSERT_EQUAL_MESSAGE(ZLAC::DRIVER_MODE::POSITION_ABSOLUTE, mode, "get_control_mode : POSITION_ABSOLUTE");
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::VELOCITY, true);
    TEST_ASSERT_TRUE_MESSAGE(result, "set_control_mode : VELOCITY");
    result = ctrl.get_control_mode(&mode);
    TEST_ASSERT_EQUAL_MESSAGE(ZLAC::DRIVER_MODE::VELOCITY, mode, "get_control_mode : VELOCITY");
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::TORQUE, true);
    TEST_ASSERT_TRUE_MESSAGE(result, "set_control_mode : TORQUE");
    result = ctrl.get_control_mode(&mode);
    TEST_ASSERT_EQUAL_MESSAGE(ZLAC::DRIVER_MODE::TORQUE, mode, "get_control_mode : TORQUE");

    result = ctrl.set_control_mode(backup_mode);
    TEST_ASSERT_TRUE(result);
}
void Control_word(void)
{
    // bool set_control_word(ZLAC_CONTROL_WORD word, bool check = false)

    // TODO
    ZLAC8015DCtrl::ZLAC_CONTROL_WORD value;
    bool result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Control_word : [%d]", value);
}
void Synchronous_control_status(void)
{
    // bool set_synchronous_control_status(bool synchronous, bool check = false)

    // TODO
    bool flag   = false;
    bool result = ctrl.get_synchronous_control_status(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Synchronous_control_status : %s", flag ? "T:synchronous" : "F:ansynchronous");
}
void Store_RW_register_to_EEPROM(void)
{
    // [CATION] This function is not implemented.
    // bool store_rw_register_to_eperm()

    // TODO
    bool result = true;

    TEST_ASSERT_TRUE_MESSAGE(result, "NOT TEST");
}
void Quick_stop_control(void)
{
    // bool set_quick_stop_control(ZLAC_STOP_CONTROL ctrl, bool check = false)

    // TODO
    ZLAC8015DCtrl::ZLAC_STOP_CONTROL value;
    bool result = ctrl.get_quick_stop_control(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Quick_stop_control : [%d]", value);
}
void Close_operation_control(void)
{
    // bool set_close_operation_control(bool stop_normally, bool check = false)

    // TODO
    bool flag   = false;
    bool result = ctrl.get_close_operation_control(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Close_operation_control : %s", flag ? "T:stop_normally" : "F:--");
}
void Disable_control(void)
{
    // bool set_disable_control(bool stop, bool check = false)

    // TODO
    bool flag   = false;
    bool result = ctrl.get_disable_control(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Disable_control : %s", flag ? "T:stop" : "F:--");
}
void Halt_control(void)
{
    // bool set_halt_control(ZLAC_STOP_CONTROL ctrl, bool check = false)

    // TODO
    ZLAC8015DCtrl::ZLAC_STOP_CONTROL value;
    bool result = ctrl.get_halt_control(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Halt_control : [%d]", value);
}
void Input_effective_level(void)
{
    // bool set_input_effective_level(bool low_level, bool check = false)

    // TODO
    bool flag   = false;
    bool result = ctrl.get_input_effective_level(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Input_effective_level : %s", flag ? "T:low_level" : "F:hight_level");
}
void Input_terminal_function_selection(void)
{
    // bool set_input_terminal_terminal_function_selection(TERMINAL_FUNCTION x0, TERMINAL_FUNCTION x1, bool check = false)

    // TODO
    ZLAC8015DCtrl::TERMINAL_FUNCTION x0;
    ZLAC8015DCtrl::TERMINAL_FUNCTION x1;
    bool result = ctrl.get_input_terminal_terminal_function_selection(&x0, &x1);
    TEST_ASSERT_TRUE(result);
    log_d("* Input_terminal_function_selection : x0[%d]x1[%d]", x0, x1);
}
void Output_effective_level(void)
{
    // bool set_output_effective_low_level(bool y0, bool y1, bool b0, bool b1, bool check = false)

    // TODO
    bool b0;
    bool b1;
    bool y0;
    bool y1;
    bool result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    log_d("* Output_effective_level : b0[%s]b1[%s]y0[%s]y1[%s]", //
          b0 ? "T" : "F",
          b1 ? "T" : "F",
          y0 ? "T" : "F",
          y1 ? "T" : "F");
}
void Output_terminal_function_selection(void)
{
    // bool set_output_terminal_function_selection( ZLAC_TERMINAL_FUNCTION b0, ZLAC_TERMINAL_FUNCTION b1, ZLAC_TERMINAL_FUNCTION y0, ZLAC_TERMINAL_FUNCTION y1, bool check = false)

    // TODO
    ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION b0;
    ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION b1;
    ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION y0;
    ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION y1;
    bool result = ctrl.get_output_terminal_function_selection(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    log_d("* Halt_control : b0[%d]b1[%d]y0[%d]y1[%d]", b0, b1, y0, y1);
}
void Driver_temperature_protection_threshold(void)
{
    // bool set_driver_temperature_protection_threshold(double value, bool check = false)

    // TODO
    double value = false;
    bool result  = ctrl.get_driver_temperature_protection_threshold(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Driver_temperature_protection_threshold : %8.3f", value);
}
void Alarm_PWM_processing_method(void)
{
    // bool set_alarm_pwm_processing_method(bool open, bool check = false)

    // TODO
    bool flag   = false;
    bool result = ctrl.get_alarm_pwm_processing_method(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Alarm_PWM_processing_method : %s", flag ? "T:Open" : "F:Close");
}
void Overload_processing_method(void)
{
    // bool set_overload_processing_method(bool open, bool check = false)

    // TODO
    bool flag   = false;
    bool result = ctrl.get_overload_processing_method(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Overload_processing_method : %s", flag ? "T:Open" : "F:Close");
}
void IO_emergency_stop_processing_mode(void)
{
    // bool set_io_emergency_stop_processing_mode(bool lock_shaft, bool check = false)

    // TODO
    bool flag   = false;
    bool result = ctrl.get_io_emergency_stop_processing_mode(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* IO_emergency_stop_processing_mode : %s", flag ? "T:Lock" : "F:Free");
}
#endif
#if MOTOR_PARAMETER
/////////////////////////////////////////////////
// Motor parameter
/////////////////////////////////////////////////
void Encoder_line(void)
{
    // bool set_encoder_line_left(int value, bool check = false)
    // bool set_encoder_line_right(int value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_encoder_line_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Encoder_line : Left :");
    result = ctrl.get_encoder_line_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Encoder_line : Right :");
    log_d("* Encoder_line : L[%d]R[%d]", left, right);
}
void Hall_offset_angle(void)
{
    // bool set_hall_offset_angle_left(int value, bool check = false)
    // bool set_hall_offset_angle_right(int value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_hall_offset_angle_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Hall_offset_angle : Left :");
    result = ctrl.get_hall_offset_angle_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Hall_offset_angle : Right :");
    log_d("* Hall_offset_angle : L[%d]R[%d]", left, right);
}
void Overload_factor(void)
{
    // bool set_overload_factor_left(int value, bool check = false)
    // bool set_overload_factor_right(int value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_overload_factor_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Overload_factor : Left :");
    result = ctrl.get_overload_factor_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Overload_factor : Right :");
    log_d("* Overload_factor : L[%d]R[%d]", left, right);
}
void Rated_current(void)
{
    // bool set_rated_current_left(double value, bool check = false)
    // bool set_rated_current_right(double value, bool check = false)

    // TODO
    double rated_left    = 0;
    double maximum_left  = 0;
    double rated_right   = 0;
    double maximum_right = 0;
    bool result          = ctrl.get_current_left(&rated_left, &maximum_left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Rated_current : Left :");
    log_d("* Rated_current : Left : rated[%8.3f]/maximum[%8.3f]", rated_left, maximum_left);
    result = ctrl.get_current_right(&rated_right, &maximum_right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Rated_current : Right :");
    log_d("* Rated_current : Right : rated[%8.3f]/maximum[%8.3f]", rated_right, maximum_right);
}
void Maximum_current(void)
{
    // bool set_maximum_current_left(double value, bool check = false)
    // bool set_maximum_current_right(double value, bool check = false)

    // TODO
    double rated_left    = 0;
    double maximum_left  = 0;
    double rated_right   = 0;
    double maximum_right = 0;
    bool result          = ctrl.get_current_left(&rated_left, &maximum_left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Maximum_current : Left :");
    log_d("* Maximum_current : Left : rated[%8.3f]/maximum[%8.3f]", rated_left, maximum_left);
    result = ctrl.get_current_right(&rated_right, &maximum_right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Maximum_current : Right :");
    log_d("* Maximum_current : Right : rated[%8.3f]/maximum[%8.3f]", rated_right, maximum_right);
}
void Overload_protection_time(void)
{
    // bool set_overload_protection_time_left(int value, bool check = false)
    // bool set_overload_protection_time_right(int value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_overload_protection_time_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Overload_protection_time : Left :");
    result = ctrl.get_overload_protection_time_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Overload_protection_time : Right :");
    log_d("* Overload_protection_time : L[%d]R[%d]", left, right);
}
void Position_following_error_threshold(void)
{
    // bool set_position_following_error_threshold_left(double value, bool check = false)
    // bool set_position_following_error_threshold_right(double value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_position_following_error_threshold_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Position_following_error_threshold : Left :");
    result = ctrl.get_position_following_error_threshold_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Position_following_error_threshold : Right :");
    log_d("* Position_following_error_threshold : L[%d]R[%d]", left, right);
}
void Velocity_smoothing_factor(void)
{
    // bool set_velocity_smoothing_factor_left(int value, bool check = false)
    // bool set_velocity_smoothing_factor_right(int value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_velocity_smoothing_factor_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Velocity_smoothing_factor : Left :");
    result = ctrl.get_velocity_smoothing_factor_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Velocity_smoothing_factor : Right :");
    log_d("* Velocity_smoothing_factor : L[%d]R[%d]", left, right);
}
void Current_loop(void)
{
    // bool set_current_loop_left(int kp, int ki, bool check = false)
    // bool set_current_loop_right(int kp, int ki, bool check = false)

    // TODO
    int left_kp  = 0;
    int left_ki  = 0;
    int right_kp = 0;
    int right_ki = 0;
    bool result  = ctrl.get_current_loop_left(&left_kp, &left_ki);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Current_loop : Left :");
    result = ctrl.get_current_loop_right(&right_kp, &right_ki);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Current_loop : Right :");
    log_d("* Current_loop : Left : kp[%d]ki[%d]", left_kp, left_ki);
    log_d("* Current_loop : Right : kp[%d]ki[%d]", right_kp, right_ki);
}
void Feedforward_output_smoothing_factor(void)
{
    // bool set_feedforward_output_smoothing_factor_left(int value, bool check = false)
    // bool set_feedforward_output_smoothing_factor_right(int value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_feedforward_output_smoothing_factor_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Feedforward_output_smoothing_factor : Left :");
    result = ctrl.get_feedforward_output_smoothing_factor_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Feedforward_output_smoothing_factor : Right :");
    log_d("* Feedforward_output_smoothing_factor : L[%d]R[%d]", left, right);
}
void Torque_output_smoothing_factor(void)
{
    // bool set_torque_output_smoothing_factor_left(int value, bool check = false)
    // bool set_torque_output_smoothing_factor_right(int value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_torque_output_smoothing_factor_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Torque_output_smoothing_factor : Left :");
    result = ctrl.get_torque_output_smoothing_factor_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Torque_output_smoothing_factor : Right :");
    log_d("* Torque_output_smoothing_factor : L[%d]R[%d]", left, right);
}
void Velocity_Loop(void)
{
    // bool set_velocity_loop_left(int kp, int ki, int kf, bool check = false)
    // bool set_velocity_loop_right(int kp, int ki, int kf, bool check = false)

    // TODO
    int left_kp  = 0;
    int left_ki  = 0;
    int left_kf  = 0;
    int right_kp = 0;
    int right_ki = 0;
    int right_kf = 0;
    bool result  = ctrl.get_velocity_loop_left(&left_kp, &left_ki, &left_kf);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Position_Loop : Left :");
    result = ctrl.get_velocity_loop_right(&right_kp, &right_ki, &right_kf);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Position_Loop : Right :");
    log_d("* Position_Loop : Left : kp[%d]ki[%d]kf[%d]", left_kp, left_ki, left_kf);
    log_d("* Position_Loop : Right : kp[%d]ki[%d]kf[%d]", right_kp, right_ki, right_kf);
}
void Position_Loop(void)
{
    // bool set_position_loop_left(int kp, int kf, bool check = false)
    // bool set_position_loop_right(int kp, int kf, bool check = false)

    // TODO
    int left_kp  = 0;
    int left_kf  = 0;
    int right_kp = 0;
    int right_kf = 0;
    bool result  = ctrl.get_position_loop_left(&left_kp, &left_kf);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Position_Loop : Left :");
    result = ctrl.get_position_loop_right(&right_kp, &right_kf);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Position_Loop : Right :");
    log_d("* Position_Loop : Left : kp[%d]kf[%d]", left_kp, left_kf);
    log_d("* Position_Loop : Right : kp[%d]kf[%d]", right_kp, right_kf);
}
void Initial_velocity(void)
{
    // bool set_initial_velocity_left(int value, bool check = false)
    // bool set_initial_velocity_right(int value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_initial_velocity_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Initial_velocity : Left :");
    result = ctrl.get_initial_velocity_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Initial_velocity : Right :");
    log_d("* Initial_velocity : L[%d]R[%d]", left, right);
}
void Motor_poles(void)
{
    // bool set_motor_poles_left(int value, bool check = false)
    // bool set_motor_poles_right(int value, bool check = false)

    // TODO
    int left    = 0;
    int right   = 0;
    bool result = ctrl.get_motor_poles_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Motor_poles : Left :");
    result = ctrl.get_motor_poles_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Motor_poles : Right :");
    log_d("* Motor_poles : L[%d]R[%d]", left, right);
}
void Over_temperature_threshold(void)
{
    // bool set_over_temperature_threshold_left(double value, bool check = false)
    // bool set_over_temperature_threshold_right(double value, bool check = false)

    // TODO
    double left  = 0;
    double right = 0;
    bool result  = ctrl.get_over_temperature_threshold_left(&left);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Over_temperature_threshold : Left :");
    result = ctrl.get_over_temperature_threshold_right(&right);
    TEST_ASSERT_TRUE_MESSAGE(result, "* Over_temperature_threshold : Right :");
    log_d("* Over_temperature_threshold : L[%8.3f]R[%8.3f]", left, right);
}
void Velocity_observer_coefficient(void)
{
    // bool set_velocity_observer_coefficient_left(int index1, int index2, int index3, int index4, bool check = false)
    // bool set_velocity_observer_coefficient_right(int index1, int index2, int index3, int index4, bool check = false)

    // TODO
    int index1 = 0;
    int index2 = 0;
    int index3 = 0;
    int index4 = 0;

    bool result = ctrl.get_velocity_observer_coefficient_left(&index1, &index2, &index3, &index4);
    TEST_ASSERT_TRUE(result);
    log_d("* Velocity_observer_coefficient : Left : L[%d]R[%d]", index1, index2, index3, index4);

    result = ctrl.get_velocity_observer_coefficient_right(&index1, &index2, &index3, &index4);
    TEST_ASSERT_TRUE(result);
    log_d("* Velocity_observer_coefficient : Right : L[%d]R[%d]", index1, index2, index3, index4);
}
#endif
#if CONTROL_PARAMETER
/////////////////////////////////////////////////
// Control parameter
/////////////////////////////////////////////////
void S_shape_acceleration_time(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_s_shape_acceleration_time(456, 321, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_s_shape_acceleration_time(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(456, left);
    TEST_ASSERT_EQUAL(321, right);
    // Restore
    result = ctrl.set_s_shape_acceleration_time(500, 500);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_s_shape_acceleration_time(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* S_shape_acceleration_time : L[%d]R[%d]", left, right);
}
void S_shape_deceleration_time(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_s_shape_deceleration_time(789, 123, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_s_shape_deceleration_time(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(789, left);
    TEST_ASSERT_EQUAL(123, right);
    // Restore
    result = ctrl.set_s_shape_deceleration_time(500, 500);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_s_shape_acceleration_time(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* S_shape_deceleration_time : L[%d]R[%d]", left, right);
}
void Deceleration_time_of_quick_stop(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_deceleration_time_of_quick_stop(12, 21, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_deceleration_time_of_quick_stop(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(12, left);
    TEST_ASSERT_EQUAL(21, right);
    // Restore
    result = ctrl.set_deceleration_time_of_quick_stop(10, 10);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_deceleration_time_of_quick_stop(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Deceleration_time_of_quick_stop : L[%d]R[%d]", left, right);
}
void Torque_slope(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_torque_slope(345, 678, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_torque_slope(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(345, left);
    TEST_ASSERT_EQUAL(678, right);
    // Restore
    result = ctrl.set_torque_slope(300, 300);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_torque_slope(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Torque_slope : L[%d]R[%d]", left, right);
}
void Target_velocity(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    result    = ctrl.set_control_mode(ZLAC::DRIVER_MODE::TORQUE, true);
    TEST_ASSERT_TRUE(result);
    // Test Setter
    result = ctrl.set_target_velocity(-123, -456, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_target_velocity(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(-123, left);
    TEST_ASSERT_EQUAL(-456, right);

    result = ctrl.set_target_velocity(789, 234, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_target_velocity(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(789, left);
    TEST_ASSERT_EQUAL(234, right);
    // Restore
    result = ctrl.set_target_velocity(0, 0);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_target_velocity(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Target_velocity : L[%d]R[%d]", left, right);
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_RELATIVE, true);
    TEST_ASSERT_TRUE(result);
}
void Target_position(void)
{
    ////////////
    bool result;
    long left  = 0;
    long right = 0;
    // Test Setter
    result = ctrl.set_target_position(-467, -891, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_target_position(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(-467, left);
    TEST_ASSERT_EQUAL(-891, right);

    result = ctrl.set_target_position(234, 567, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_target_position(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(234, left);
    TEST_ASSERT_EQUAL(567, right);
    // Restore
    result = ctrl.set_target_position(0, 0);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_target_position(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Target_position : L[%d]R[%d]", left, right);
}
void Max_speed(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_max_speed(97, 86, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_max_speed(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(97, left);
    TEST_ASSERT_EQUAL(86, right);
    // Restore
    result = ctrl.set_max_speed(120, 120);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_max_speed(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Max_speed : L[%d]R[%d]", left, right);
}
void Target_torque(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    result    = ctrl.set_control_mode(ZLAC::DRIVER_MODE::TORQUE, true);
    TEST_ASSERT_TRUE(result);
    // Test Setter
    result = ctrl.set_target_torque(-467, -891, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_target_torque(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(-467, left);
    TEST_ASSERT_EQUAL(-891, right);

    result = ctrl.set_target_torque(234, 567, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_target_torque(&left, &right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(234, left);
    TEST_ASSERT_EQUAL(567, right);
    // Restore
    result = ctrl.set_target_torque(0, 0);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_target_torque(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Target_torque : L[%d]R[%d]", left, right);

    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_RELATIVE, true);
    TEST_ASSERT_TRUE(result);
}
#endif
#if READ_ONLY_PARAMETER
/////////////////////////////////////////////////
// Read only parameter
/////////////////////////////////////////////////
void Software_version(void)
{
    int value   = 0;
    bool result = ctrl.get_software_version(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Software_version : %d", value);
    TEST_ASSERT_EQUAL_INT(ZLAC_DRIVER_VERSION, value);
}
void Bus_voltage(void)
{
    double value = NAN;
    bool result  = ctrl.get_bus_voltage(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Bus_voltage : %8.3f", value);
    if (NAN == value) {
        TEST_ASSERT_TRUE_MESSAGE(false, "NOT updated this value");
    }
}
void Status_word(void)
{
    bool left_shaft_lock;
    bool left_emergency_stop;
    bool left_alarm;
    bool left_is_run;
    bool right_shaft_lock;
    bool right_emergency_stop;
    bool right_alarm;
    bool right_is_run;

    bool result = ctrl.get_status_word(&left_shaft_lock, //
                                       &left_emergency_stop,
                                       &left_alarm,
                                       &left_is_run,
                                       &right_shaft_lock,
                                       &right_emergency_stop,
                                       &right_alarm,
                                       &right_is_run);
    TEST_ASSERT_TRUE(result);
    log_d("* Status_word : "
          "left_shaft_lock[%s]"
          "left_emergency_stop[%s]"
          "left_alarm[%s]"
          "left_is_run[%s]"
          "right_shaft_lock[%s]"
          "right_emergency_stop[%s]"
          "right_alarm[%s]"
          "right_is_run[%s] ", //
          left_shaft_lock ? "T" : "F",
          left_emergency_stop ? "T" : "F",
          left_alarm ? "T" : "F",
          left_is_run ? "T" : "F",
          right_shaft_lock ? "T" : "F",
          right_emergency_stop ? "T" : "F",
          right_alarm ? "T" : "F",
          right_is_run ? "T" : "F");
}
void Hall_input_state(void)
{
    bool left;
    bool right;
    bool result = ctrl.get_hall_input_state(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Hall_input_state : L[%s]R[%s]", left ? "T" : "F", right ? "T" : "F");
}
void Motor_temperature(void)
{
    int left;
    int right;
    bool result = ctrl.get_motor_temperature(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Motor_temperature : L[%d]R[%d]", left, right);
    TEST_ASSERT_GREATER_OR_EQUAL(0, left);
    TEST_ASSERT_GREATER_OR_EQUAL(0, right);
}
void Error_code(void)
{
    ZLAC8015DCtrl::zlac_error left;
    ZLAC8015DCtrl::zlac_error right;
    bool result = ctrl.get_error_code(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Error_code : L : "
          "over_voltage[%s]"
          "under_voltage[%s]"
          "over_current[%s]"
          "over_load[%s]"
          "current_out_of_tolerance[%s]"
          "encoder_out_of_tolerance[%s]"
          "velocity_out_of_tolerance[%s]"
          "reference_voltage_error[%s]"
          "eeprom_error[%s]"
          "hall_error[%s]"
          "motor_temperature_over_temperature[%s]", //
          left.over_voltage ? "T" : "F",
          left.under_voltage ? "T" : "F",
          left.over_current ? "T" : "F",
          left.over_load ? "T" : "F",
          left.current_out_of_tolerance ? "T" : "F",
          left.encoder_out_of_tolerance ? "T" : "F",
          left.velocity_out_of_tolerance ? "T" : "F",
          left.reference_voltage_error ? "T" : "F",
          left.eeprom_error ? "T" : "F",
          left.hall_error ? "T" : "F",
          left.motor_temperature_over_temperature ? "T" : "F");
    log_d("* Error_code : R : "
          "over_voltage[%s]"
          "under_voltage[%s]"
          "over_current[%s]"
          "over_load[%s]"
          "current_out_of_tolerance[%s]"
          "encoder_out_of_tolerance[%s]"
          "velocity_out_of_tolerance[%s]"
          "reference_voltage_error[%s]"
          "eeprom_error[%s]"
          "hall_error[%s]"
          "motor_temperature_over_temperature[%s]", //
          right.over_voltage ? "T" : "F",
          right.under_voltage ? "T" : "F",
          right.over_current ? "T" : "F",
          right.over_load ? "T" : "F",
          right.current_out_of_tolerance ? "T" : "F",
          right.encoder_out_of_tolerance ? "T" : "F",
          right.velocity_out_of_tolerance ? "T" : "F",
          right.reference_voltage_error ? "T" : "F",
          right.eeprom_error ? "T" : "F",
          right.hall_error ? "T" : "F",
          right.motor_temperature_over_temperature ? "T" : "F");
}
void Actual_motor_position(void)
{
    long left;
    long right;
    bool result = ctrl.get_actual_motor_position(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Actual_motor_position : L[%d]R[%d]", left, right);
    // TEST_ASSERT_GREATER_OR_EQUAL(0, left);
    // TEST_ASSERT_GREATER_OR_EQUAL(0, right);
}
void Actual_velocity(void)
{
    double left  = NAN;
    double right = NAN;
    bool result  = ctrl.get_actual_velocity(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Actual_velocity : L[%8.3f]R[%8.3f]", left, right);
    if (NAN == left) {
        TEST_ASSERT_TRUE_MESSAGE(false, "NOT updated this value(left)");
    }
    if (NAN == right) {
        TEST_ASSERT_TRUE_MESSAGE(false, "NOT updated this value(right)");
    }
}
void Actual_torque(void)
{
    double left  = NAN;
    double right = NAN;
    bool result  = ctrl.get_actual_torque(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Actual_torque : L[%8.3f]R[%8.3f]", left, right);
    if (NAN == left) {
        TEST_ASSERT_TRUE_MESSAGE(false, "NOT updated this value(left)");
    }
    if (NAN == right) {
        TEST_ASSERT_TRUE_MESSAGE(false, "NOT updated this value(right)");
    }
}
void Software_connected_status(void)
{
    bool value;
    bool result = ctrl.get_software_connected_status(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Software_connected_status : %s", value ? "T" : "F");
    TEST_ASSERT_TRUE_MESSAGE(value, "NOT connected");
}
void Driver_temperature(void)
{
    double value = NAN;
    bool result  = ctrl.get_driver_temperature(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Driver_temperature : %8.3f", value);
    if (NAN == value) {
        TEST_ASSERT_TRUE_MESSAGE(false, "NOT updated this value");
    }
}
#endif
void NOT_CONNECTED_DEVICE(void)
{
    TEST_ASSERT_TRUE_MESSAGE(false, "[Not connected] Please check the connection.");
}

void TEST_ONCE(void)
{
    log_d("=== TEST_ONCE ===");
    long left;
    long right;
    bool result = ctrl.get_actual_motor_position(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Actual_motor_position : L[%d]R[%d]", left, right);
}

///////////////////////////////////////////////////////////////////

void RUN_UNITY_TESTS()
{
    (void)setup_m5();
    UNITY_BEGIN();
    bool value;
    bool result = ctrl.get_software_connected_status(&value);
    if (false == result) {
        RUN_TEST(NOT_CONNECTED_DEVICE);
    } else {
        RUN_TEST(TEST_ONCE);
        //////////////////////////////////
#if READ_ONLY_PARAMETER
        log_d("=== Read only parameter ===");
        // Read only parameter
        RUN_TEST(Software_version);
        RUN_TEST(Bus_voltage);
        RUN_TEST(Status_word);
        RUN_TEST(Hall_input_state);
        RUN_TEST(Motor_temperature);
        RUN_TEST(Error_code);
        RUN_TEST(Actual_motor_position);
        RUN_TEST(Actual_velocity);
        RUN_TEST(Actual_torque);
        RUN_TEST(Software_connected_status);
        RUN_TEST(Driver_temperature);
#endif
#if CONTROL_PARAMETER
        // Control parameter
        log_d("=== Control parameter ===");
        RUN_TEST(S_shape_acceleration_time);
        RUN_TEST(S_shape_deceleration_time);
        RUN_TEST(Deceleration_time_of_quick_stop);
        RUN_TEST(Torque_slope);
        RUN_TEST(Target_velocity);
        RUN_TEST(Target_position);
        RUN_TEST(Max_speed);
        RUN_TEST(Target_torque);
#endif
#if COMMON_CONSTANT
        // Common constant
        log_d("=== Common constant ===");
        RUN_TEST(Communication_offline_time);
        RUN_TEST(RS485_Node_ID);
        RUN_TEST(RS485_Baud_Rate);
        RUN_TEST(Input_signal_status);
        RUN_TEST(Out_signal_status);
        RUN_TEST(Clear_feedback_position);
        RUN_TEST(In_absolute_position_control_reset_the_zero_point);
        RUN_TEST(Shaft_state_after_power_on);
        RUN_TEST(Maximum_motor_speed);
        RUN_TEST(Register_parameter_settings);
        RUN_TEST(CAN_Node_info);
        RUN_TEST(Control_mode);
        RUN_TEST(Control_word);
        RUN_TEST(Synchronous_control_status);
        RUN_TEST(Store_RW_register_to_EEPROM);
        RUN_TEST(Quick_stop_control);
        RUN_TEST(Close_operation_control);
        RUN_TEST(Disable_control);
        RUN_TEST(Halt_control);
        RUN_TEST(Input_effective_level);
        RUN_TEST(Input_terminal_function_selection);
        RUN_TEST(Output_effective_level);
        RUN_TEST(Output_terminal_function_selection);
        RUN_TEST(Driver_temperature_protection_threshold);
        RUN_TEST(Alarm_PWM_processing_method);
        RUN_TEST(Overload_processing_method);
        RUN_TEST(IO_emergency_stop_processing_mode);
#endif
#if MOTOR_PARAMETER
        // Motor parameter
        log_d("=== Motor parameter ===");
        RUN_TEST(Encoder_line);
        RUN_TEST(Hall_offset_angle);
        RUN_TEST(Overload_factor);
        RUN_TEST(Rated_current);
        RUN_TEST(Maximum_current);
        RUN_TEST(Overload_protection_time);
        RUN_TEST(Position_following_error_threshold);
        RUN_TEST(Velocity_smoothing_factor);
        RUN_TEST(Current_loop);
        RUN_TEST(Feedforward_output_smoothing_factor);
        RUN_TEST(Torque_output_smoothing_factor);
        RUN_TEST(Velocity_Loop);
        RUN_TEST(Initial_velocity);
        RUN_TEST(Motor_poles);
        RUN_TEST(Over_temperature_threshold);
        RUN_TEST(Velocity_observer_coefficient);
#endif
        //////////////////////////////////
    }
    UNITY_END();
}

///////////////////////////////////////////////////////////////////

#ifdef ARDUINO
#include <Arduino.h>
void setup()
{
    RUN_UNITY_TESTS();
}
void loop()
{
}
#else
int main(int argc, char **argv)
{
    RUN_UNITY_TESTS();
    return 0;
}
#endif
