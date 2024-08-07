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
#ifndef MOTOR_PARAMETER_RUNNING
#define MOTOR_PARAMETER_RUNNING 0
#endif
#ifndef COMPILE_TEST_FUNCTION
#define COMPILE_TEST_FUNCTION 1
#endif
#ifndef RESTORE_FACTORY_SETTINGS
#define RESTORE_FACTORY_SETTINGS 0
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
String text_zlac_driver_mode(ZLAC::DRIVER_MODE mode)
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
        case ZLAC::DRIVER_MODE::UNDEFINED:
            return "UNDEFINED";
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
String text_rs485_baud_rate(ZLAC8015DCtrl::RS485_BAUD_RATE baud)
{
    switch (baud) {
        case ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_128000:
            return "128000";
        case ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_115200:
            return "115200";
        case ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_57600:
            return " 57600";
        case ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_38400:
            return " 38400";
        case ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_19200:
            return " 19200";
        case ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_9600:
            return " 9600";
        default:
            return "INVALID";
    }
}
String text_zlac_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD word)
{
    switch (word) {
        case ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_EMERGENCY_STOP:
            return "EMERGENCY_STOP";
        case ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_CLEAR_FAULT:
            return "CLEAR_FAULT";
        case ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_STOP:
            return "STOP";
        case ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE:
            return "ENABLE";
        case ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START:
            return "SYNCHRONOUS_START";
        case ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_START_LEFT:
            return "START_LEFT";
        case ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_START_RIGHT:
            return "START_RIGHT";
        case ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED:
            return "UNDEFINED";
        default:
            return "INVALID";
    }
}
String text_zlac_stop_control(ZLAC8015DCtrl::ZLAC_STOP_CONTROL ctrl)
{
    switch (ctrl) {
        case ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION:
            return "QUICK_WITH_DECELERATION";
        case ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION:
            return "_QUICK_WITHOUT_DECELERATION";
        case ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP:
            return "STOP";
        case ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_UNDEFINED:
            return "UNDEFINED";
        default:
            return "INVALID";
    }
}
String text_terminal_function(ZLAC8015DCtrl::TERMINAL_FUNCTION func)
{
    switch (func) {
        case ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE:
            return "NONE";
        case ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP:
            return "EMERGENCY_STOP";
        case ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_NC:
            return "NC";
        default:
            return "INVALID";
    }
}
String text_zlac_terminal_function(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION func)
{
    switch (func) {
        case ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE:
            return "OPEN_BRAKE";
        case ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE:
            return "CLOSE_BRAKE";
        case ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL:
            return "ALARM_SIGNAL";
        case ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL:
            return "DRIVE_STATUS_SIGNAL";
        case ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL:
            return "TARGET_POSITION_REACHED_SIGNAL";
        case ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED:
            return "UNDEFINED";
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
#if COMMON_CONSTANT | COMPILE_TEST_FUNCTION
void Communication_offline_time(void)
{
    ////////////
    bool result;
    int value = 0;
    // Test Setter
    result = ctrl.set_communication_offline_time(789, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_communication_offline_time(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(789, value);
    // Restore
    result = ctrl.set_communication_offline_time(1000);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_communication_offline_time(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Communication_offline_time [%d]", value);
}
void RS485_Node_ID(void)
{
    ////////////
    bool result;
    int value = 0;
#if 0
    // Test Setter
    //////////////////////////////////////////////
    // NOTE: Communication settings such as address/baud rate are not dynamically supported
    //////////////////////////////////////////////

    result = ctrl.set_rs485_node_id(23, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_rs485_node_id(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(23, value);
    // Restore
    result = ctrl.set_rs485_node_id(1);
    TEST_ASSERT_TRUE(result);
#endif
    result = ctrl.get_rs485_node_id(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* RS485_Node_ID [%d]", value);
}
void RS485_Baud_Rate(void)
{
    ////////////
    bool result;
    ZLAC8015DCtrl::RS485_BAUD_RATE value;
#if 0
    // Test Setter
    //////////////////////////////////////////////
    // NOTE: Communication settings such as address/baud rate are not dynamically supported
    //////////////////////////////////////////////
    ZLAC8015DCtrl::RS485_BAUD_RATE input_value;
    // ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_128000;
    input_value = ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_128000;
    result      = ctrl.set_rs485_baud_rate(input_value, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_rs485_baud_rate(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(input_value, value);
    // [Change baud]

    // ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_115200;
    input_value = ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_115200;
    result      = ctrl.set_rs485_baud_rate(input_value, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_rs485_baud_rate(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(input_value, value);
    // [Change baud]

    // ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_57600;
    input_value = ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_57600;
    result      = ctrl.set_rs485_baud_rate(input_value, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_rs485_baud_rate(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(input_value, value);
    // [Change baud]

    // ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_38400;
    input_value = ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_38400;
    result      = ctrl.set_rs485_baud_rate(input_value, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_rs485_baud_rate(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(input_value, value);
    // [Change baud]

    // ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_19200;
    input_value = ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_19200;
    result      = ctrl.set_rs485_baud_rate(input_value, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_rs485_baud_rate(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(input_value, value);
    // [Change baud]

    // ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_9600;
    input_value = ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_9600;
    result      = ctrl.set_rs485_baud_rate(input_value, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_rs485_baud_rate(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(input_value, value);
    // [Change baud]

    // Restore
    result = ctrl.set_rs485_baud_rate(ZLAC8015DCtrl::RS485_BAUD_RATE::RS485_BAUD_RATE_128000);
    TEST_ASSERT_TRUE(result);
#endif
    result = ctrl.get_rs485_baud_rate(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* RS485_Baud_Rate [%s]", text_rs485_baud_rate(value));
}
void Input_signal_status(void)
{
    // TODO
    // [NOT TEST] If you want to test it, enter the pin.
    bool x0     = false;
    bool x1     = false;
    bool result = ctrl.get_input_signal_status(&x0, &x1);
    TEST_ASSERT_TRUE(result);
    log_d("* Input_signal_status : x0[%s]x1[%s]", x0 ? "T" : "F", x1 ? "T" : "F");
}
void Out_signal_status(void)
{
    // TODO
    // [NOT TEST] If you test it, you need to set it so that the out signal changes.
    bool x0     = false;
    bool x1     = false;
    bool result = ctrl.get_out_signal_status(&x0, &x1);
    TEST_ASSERT_TRUE(result);
    log_d("* Out_signal_status : x0[%s]x1[%s]", x0 ? "T" : "F", x1 ? "T" : "F");
}
void Clear_feedback_position(void)
{
    // TODO
    // bool set_clear_feedback_position(ZLAC::target_motor target, bool check = false)

    ZLAC::target_motor value;
    bool result = ctrl.get_clear_feedback_position(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Clear_feedback_position : [%d]", value);
}
void In_absolute_position_control_reset_the_zero_point(void)
{
    // TODO
    // bool set_reset_the_zero_point_in_absolute_position_control(ZLAC::target_motor target, bool check = false)

    ZLAC::target_motor value;
    bool result = ctrl.get_reset_the_zero_point_in_absolute_position_control(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* In_absolute_position_control_reset_the_zero_point : [%d]", value);
}
void Shaft_state_after_power_on(void)
{
    ////////////
    bool result;
    bool flag = false;
    // Test Setter
    result = ctrl.set_lock_shaft_state_after_power_on(false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_lock_shaft_state_after_power_on(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(flag);

    result = ctrl.set_lock_shaft_state_after_power_on(true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_lock_shaft_state_after_power_on(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(flag);

    // Restore
    result = ctrl.set_lock_shaft_state_after_power_on(false);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_lock_shaft_state_after_power_on(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Shaft_state_after_power_on : %s", flag ? "T:lock" : "F:unlock");
}
void Maximum_motor_speed(void)
{
    ////////////
    bool result;
    int value = 0;
    // Test Setter
    result = ctrl.set_maximum_motor_speed(987, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_maximum_motor_speed(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(987, value);
    // Restore
    result = ctrl.set_maximum_motor_speed(1000);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_maximum_motor_speed(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Maximum_motor_speed [%d]", value);
}
void Register_parameter_settings(void)
{
    bool result = true;
#if 0
    // [NOT TEST]
    result = ctrl.restore_factory_settings();
#endif
    log_d("* Restore_factory_settings : %s", result ? "T" : "F");
    TEST_ASSERT_TRUE_MESSAGE(result, "NOT TEST");
}
void CAN_Node_info(void)
{
    ////////////
    bool result;
    int id = 0;
    ZLAC8015DCtrl::CAN_BAUD_RATE baud;
    // Test Setter
    result = ctrl.set_can_node_info(2, ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_1000K, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_can_node_info(&id, &baud);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(2, id);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_1000K, baud);

    result = ctrl.set_can_node_info(3, ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_500K, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_can_node_info(&id, &baud);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(3, id);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_500K, baud);

    result = ctrl.set_can_node_info(4, ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_250K, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_can_node_info(&id, &baud);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(4, id);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_250K, baud);

    result = ctrl.set_can_node_info(5, ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_125K, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_can_node_info(&id, &baud);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(5, id);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_125K, baud);

    result = ctrl.set_can_node_info(6, ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_100K, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_can_node_info(&id, &baud);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(6, id);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_100K, baud);

    result = ctrl.set_can_node_info(7, ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_50K, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_can_node_info(&id, &baud);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(7, id);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_50K, baud);

    result = ctrl.set_can_node_info(8, ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_25K, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_can_node_info(&id, &baud);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(8, id);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_25K, baud);

    // Restore
    result = ctrl.set_can_node_info(1, ZLAC8015DCtrl::CAN_BAUD_RATE::CAN_BAUD_RATE_500K);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_can_node_info(&id, &baud);
    TEST_ASSERT_TRUE(result);
    log_d("* CAN_Node_info : id[%d]baud[%s]", id, text_can_baud_rate(baud));
}
void Control_mode(void)
{
    ////////////
    bool result;
    ZLAC::DRIVER_MODE mode;
    // Test Setter
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_RELATIVE, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_mode(&mode);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC::DRIVER_MODE::POSITION_RELATIVE, mode);

    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_ABSOLUTE, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_mode(&mode);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC::DRIVER_MODE::POSITION_ABSOLUTE, mode);

    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::VELOCITY, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_mode(&mode);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC::DRIVER_MODE::VELOCITY, mode);

    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::TORQUE, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_mode(&mode);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC::DRIVER_MODE::TORQUE, mode);

    // Restore
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::UNDEFINED, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_mode(&mode);
    TEST_ASSERT_TRUE(result);
    log_d("* Control_mode : %s", text_zlac_driver_mode(mode).c_str());
}
void Control_word(void)
{
    ////////////
    bool result;
    ZLAC8015DCtrl::ZLAC_CONTROL_WORD value;
    // Test Setter
    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_EMERGENCY_STOP, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_EMERGENCY_STOP, value);

    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED, value);

    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_CLEAR_FAULT, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_CLEAR_FAULT, value);

    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_STOP, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_STOP, value);

    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE, value);

    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_RELATIVE);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START, value);

    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_ABSOLUTE);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START, value);

    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::TORQUE);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START, true);
    TEST_ASSERT_FALSE(result);

    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::VELOCITY);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_SYNCHRONOUS_START, true);
    TEST_ASSERT_FALSE(result);

    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_START_LEFT, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_START_LEFT, value);

    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_START_RIGHT, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_START_RIGHT, value);

    // Restore
    result = ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_control_word(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Control_word : %s", text_zlac_control_word(value));
}
void Synchronous_control_status(void)
{
    ////////////
    bool result;
    bool flag = false;
    // Test Setter
    result = ctrl.set_synchronous_control_status(true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_synchronous_control_status(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(flag);

    result = ctrl.set_synchronous_control_status(false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_synchronous_control_status(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(flag);

    // Restore
    result = ctrl.set_synchronous_control_status(true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_synchronous_control_status(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Synchronous_control_status : %s", flag ? "T:synchronous" : "F:ansynchronous");
}
void Store_RW_register_to_EEPROM(void)
{
    // [CATION] This function is not implemented.
    ////////////
    bool result = true;
    bool flag   = false;
    // Test Setter
    // result = ctrl.store_rw_register_to_eperm();
    // TEST_ASSERT_TRUE(result);

    // Restore
    TEST_ASSERT_TRUE_MESSAGE(result, "NOT TEST");
}
void Quick_stop_control(void)
{
    ////////////
    bool result;
    ZLAC8015DCtrl::ZLAC_STOP_CONTROL value;
    // Test Setter
    result = ctrl.set_quick_stop_control(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_quick_stop_control(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION, value);

    result = ctrl.set_quick_stop_control(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_quick_stop_control(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP, value);

    result = ctrl.set_quick_stop_control(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_quick_stop_control(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION, value);

    // Restore
    result = ctrl.set_quick_stop_control(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_quick_stop_control(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Quick_stop_control : %s", text_zlac_stop_control(value));
}
void Close_operation_control(void)
{
    ////////////
    bool result;
    bool flag = false;
    // Test Setter
    result = ctrl.set_close_operation_control(true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_close_operation_control(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(flag);

    result = ctrl.set_close_operation_control(false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_close_operation_control(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(flag);

    // Restore
    result = ctrl.set_close_operation_control(true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_close_operation_control(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Close_operation_control : %s", flag ? "T:stop_normally" : "F:--");
}
void Disable_control(void)
{
    ////////////
    bool result;
    bool flag = false;
    // Test Setter
    result = ctrl.set_disable_control(false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_disable_control(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(flag);

    result = ctrl.set_disable_control(true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_disable_control(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(flag);

    // Restore
    result = ctrl.set_disable_control(false);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_disable_control(&flag);
    TEST_ASSERT_TRUE(result);
    log_d("* Disable_control : %s", flag ? "T:stop" : "F:--");
}
void Halt_control(void)
{
    ////////////
    bool result;
    ZLAC8015DCtrl::ZLAC_STOP_CONTROL value;
    // Test Setter
    result = ctrl.set_halt_control(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_halt_control(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITH_DECELERATION, value);

    result = ctrl.set_halt_control(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_halt_control(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP, value);

    result = ctrl.set_halt_control(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_halt_control(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_QUICK_WITHOUT_DECELERATION, value);

    // Restore
    result = ctrl.set_halt_control(ZLAC8015DCtrl::ZLAC_STOP_CONTROL::ZLAC_STOP_CONTROL_STOP);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_halt_control(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Halt_control : %s", text_zlac_stop_control(value));
}
void Input_effective_level(void)
{
    ////////////
    bool result;
    bool x0 = false;
    bool x1 = false;
    // Test Setter
    TEST_ASSERT_TRUE(ctrl.set_input_effective_low_level(false, false, true));
    TEST_ASSERT_TRUE(ctrl.get_input_effective_low_level(&x0, &x1));
    TEST_ASSERT_FALSE(x0);
    TEST_ASSERT_FALSE(x1);

    TEST_ASSERT_TRUE(ctrl.set_input_effective_low_level(true, false, true));
    TEST_ASSERT_TRUE(ctrl.get_input_effective_low_level(&x0, &x1));
    TEST_ASSERT_TRUE(x0);
    TEST_ASSERT_FALSE(x1);

    TEST_ASSERT_TRUE(ctrl.set_input_effective_low_level(false, true, true));
    TEST_ASSERT_TRUE(ctrl.get_input_effective_low_level(&x0, &x1));
    TEST_ASSERT_FALSE(x0);
    TEST_ASSERT_TRUE(x1);

    TEST_ASSERT_TRUE(ctrl.set_input_effective_low_level(true, true, true));
    TEST_ASSERT_TRUE(ctrl.get_input_effective_low_level(&x0, &x1));
    TEST_ASSERT_TRUE(x0);
    TEST_ASSERT_TRUE(x1);

    // Restore
    TEST_ASSERT_TRUE(ctrl.set_input_effective_low_level(false, false));
    TEST_ASSERT_TRUE(ctrl.get_input_effective_low_level(&x0, &x1));
    log_d("* Input_effective_level : x0[%s]x1[%s]", x0 ? "T:low_level" : "F:hight_level", x1 ? "T:low_level" : "F:hight_level");
}
void Input_terminal_function_selection(void)
{
    ////////////
    bool result;
    ZLAC8015DCtrl::TERMINAL_FUNCTION x0;
    ZLAC8015DCtrl::TERMINAL_FUNCTION x1;
    // Test Setter
    result = ctrl.set_input_terminal_terminal_function_selection(ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE, //
                                                                 ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_input_terminal_terminal_function_selection(&x0, &x1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE, x0);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP, x1);

    result = ctrl.set_input_terminal_terminal_function_selection(ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP, //
                                                                 ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_input_terminal_terminal_function_selection(&x0, &x1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP, x0);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE, x1);

    result = ctrl.set_input_terminal_terminal_function_selection(ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_NC, //
                                                                 ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP);
    TEST_ASSERT_FALSE(result);

    // Restore
    result = ctrl.set_input_terminal_terminal_function_selection(ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_EMERGENCY_STOP, //
                                                                 ZLAC8015DCtrl::TERMINAL_FUNCTION::TERMINAL_FUNCTION_NONE);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_input_terminal_terminal_function_selection(&x0, &x1);
    TEST_ASSERT_TRUE(result);
    log_d("* Input_terminal_function_selection : x0[%s]x1[%s]", text_terminal_function(x0).c_str(), text_terminal_function(x1).c_str());
}
void Output_effective_level(void)
{
    ////////////
    bool result;
    bool b0;
    bool b1;
    bool y0;
    bool y1;
    // Test Setter
    result = ctrl.set_output_effective_low_level(true, false, false, false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(b0);
    TEST_ASSERT_FALSE(b1);
    TEST_ASSERT_FALSE(y0);
    TEST_ASSERT_FALSE(y1);

    result = ctrl.set_output_effective_low_level(false, true, false, false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(b0);
    TEST_ASSERT_TRUE(b1);
    TEST_ASSERT_FALSE(y0);
    TEST_ASSERT_FALSE(y1);

    result = ctrl.set_output_effective_low_level(false, false, true, false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(b0);
    TEST_ASSERT_FALSE(b1);
    TEST_ASSERT_TRUE(y0);
    TEST_ASSERT_FALSE(y1);

    result = ctrl.set_output_effective_low_level(false, false, false, true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(b0);
    TEST_ASSERT_FALSE(b1);
    TEST_ASSERT_FALSE(y0);
    TEST_ASSERT_TRUE(y1);

    result = ctrl.set_output_effective_low_level(true, true, false, false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(b0);
    TEST_ASSERT_TRUE(b1);
    TEST_ASSERT_FALSE(y0);
    TEST_ASSERT_FALSE(y1);

    result = ctrl.set_output_effective_low_level(true, false, true, false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(b0);
    TEST_ASSERT_FALSE(b1);
    TEST_ASSERT_TRUE(y0);
    TEST_ASSERT_FALSE(y1);

    result = ctrl.set_output_effective_low_level(true, false, false, true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(b0);
    TEST_ASSERT_FALSE(b1);
    TEST_ASSERT_FALSE(y0);
    TEST_ASSERT_TRUE(y1);

    result = ctrl.set_output_effective_low_level(false, true, true, false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(b0);
    TEST_ASSERT_TRUE(b1);
    TEST_ASSERT_TRUE(y0);
    TEST_ASSERT_FALSE(y1);

    result = ctrl.set_output_effective_low_level(false, true, false, true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(b0);
    TEST_ASSERT_TRUE(b1);
    TEST_ASSERT_FALSE(y0);
    TEST_ASSERT_TRUE(y1);

    result = ctrl.set_output_effective_low_level(false, false, true, true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(b0);
    TEST_ASSERT_FALSE(b1);
    TEST_ASSERT_TRUE(y0);
    TEST_ASSERT_TRUE(y1);

    result = ctrl.set_output_effective_low_level(false, true, true, true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(b0);
    TEST_ASSERT_TRUE(b1);
    TEST_ASSERT_TRUE(y0);
    TEST_ASSERT_TRUE(y1);

    result = ctrl.set_output_effective_low_level(true, false, true, true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(b0);
    TEST_ASSERT_FALSE(b1);
    TEST_ASSERT_TRUE(y0);
    TEST_ASSERT_TRUE(y1);

    result = ctrl.set_output_effective_low_level(true, true, false, true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(b0);
    TEST_ASSERT_TRUE(b1);
    TEST_ASSERT_FALSE(y0);
    TEST_ASSERT_TRUE(y1);

    result = ctrl.set_output_effective_low_level(true, true, true, false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(b0);
    TEST_ASSERT_TRUE(b1);
    TEST_ASSERT_TRUE(y0);
    TEST_ASSERT_FALSE(y1);

    // Restore
    result = ctrl.set_output_effective_low_level(false, false, false, false);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_effective_low_level(&b0, &b1, &y0, &y1);
    log_d("* Output_effective_level : b0[%s]b1[%s]y0[%s]y1[%s]", //
          b0 ? "T" : "F",
          b1 ? "T" : "F",
          y0 ? "T" : "F",
          y1 ? "T" : "F");
}
void Output_terminal_function_selection(void)
{
    ////////////
    bool result;
    ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION b0;
    ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION b1;
    ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION y0;
    ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION y1;
    // Test Setter
    result = ctrl.set_output_terminal_function_selection(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE, //
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE,
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL,
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL,
                                                         true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_terminal_function_selection(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE, b0);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE, b1);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL, y0);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL, y1);

    result = ctrl.set_output_terminal_function_selection(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE, //
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE,
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL,
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL,
                                                         true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_terminal_function_selection(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE, b0);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE, b1);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_DRIVE_STATUS_SIGNAL, y0);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL, y1);

    result = ctrl.set_output_terminal_function_selection(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE, //
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE,
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL,
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL,
                                                         true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_terminal_function_selection(&b0, &b1, &y0, &y1);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE, b0);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_CLOSE_BRAKE, b1);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_TARGET_POSITION_REACHED_SIGNAL, y0);
    TEST_ASSERT_EQUAL(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_ALARM_SIGNAL, y1);

    // Restore
    result = ctrl.set_output_terminal_function_selection(ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE, //
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_OPEN_BRAKE,
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED,
                                                         ZLAC8015DCtrl::ZLAC_TERMINAL_FUNCTION::ZLAC_TERMINAL_FUNCTION_UNDEFINED,
                                                         true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_output_terminal_function_selection(&b0, &b1, &y0, &y1);
    log_d("* Output_terminal_function_selection : b0[%s]b1[%s]", //
          text_zlac_terminal_function(b0).c_str(),
          text_zlac_terminal_function(b1).c_str());
    log_d("* Output_terminal_function_selection : y0[%s]y1[%s]", //
          text_zlac_terminal_function(y0).c_str(),
          text_zlac_terminal_function(y1).c_str());
}
void Driver_temperature_protection_threshold(void)
{
    ////////////
    bool result;
    double value = 0;
    // Test Setter
    result = ctrl.set_driver_temperature_protection_threshold(98.71, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_driver_temperature_protection_threshold(&value);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 98.71, value);

    // Restore
    result = ctrl.set_driver_temperature_protection_threshold(80.0);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_driver_temperature_protection_threshold(&value);
    TEST_ASSERT_TRUE(result);
    log_d("* Driver_temperature_protection_threshold : %8.3f", value);
}
void Alarm_PWM_processing_method(void)
{
    ////////////
    bool result;
    bool flag = false;
    // Test Setter
    result = ctrl.set_alarm_pwm_processing_method(false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_alarm_pwm_processing_method(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(flag);

    result = ctrl.set_alarm_pwm_processing_method(true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_alarm_pwm_processing_method(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(flag);

    // Restore
    result = ctrl.set_alarm_pwm_processing_method(false);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_alarm_pwm_processing_method(&flag);
    log_d("* Alarm_PWM_processing_method : %s", flag ? "T:Open" : "F:Close");
}
void Overload_processing_method(void)
{
    ////////////
    bool result;
    bool flag = false;
    // Test Setter
    result = ctrl.set_overload_processing_method(false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_overload_processing_method(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(flag);

    result = ctrl.set_overload_processing_method(true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_overload_processing_method(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(flag);

    // Restore
    result = ctrl.set_overload_processing_method(false);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_overload_processing_method(&flag);
    log_d("* Overload_processing_method : %s", flag ? "T:Open" : "F:Close");
}
void IO_emergency_stop_processing_mode(void)
{
    ////////////
    bool result;
    bool flag = false;
    // Test Setter
    result = ctrl.set_io_emergency_stop_processing_mode(false, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_io_emergency_stop_processing_mode(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FALSE(flag);

    result = ctrl.set_io_emergency_stop_processing_mode(true, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_io_emergency_stop_processing_mode(&flag);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(flag);

    // Restore
    result = ctrl.set_io_emergency_stop_processing_mode(false);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_io_emergency_stop_processing_mode(&flag);
    log_d("* IO_emergency_stop_processing_mode : %s", flag ? "T:Lock" : "F:Free");
}
#endif
#if MOTOR_PARAMETER | COMPILE_TEST_FUNCTION
/////////////////////////////////////////////////
// Motor parameter
/////////////////////////////////////////////////
void Encoder_line(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_encoder_line_left(1234, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_encoder_line_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(1234, left);

    result = ctrl.set_encoder_line_right(1765, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_encoder_line_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(1765, right);

    // Restore
    result = ctrl.set_encoder_line_left(1024, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_encoder_line_right(1024, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_encoder_line_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_encoder_line_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Encoder_line : L[%d]R[%d]", left, right);
}
void Hall_offset_angle(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_hall_offset_angle_left(-123, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_hall_offset_angle_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(-123, left);

    result = ctrl.set_hall_offset_angle_right(-234, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_hall_offset_angle_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(-234, right);

    result = ctrl.set_hall_offset_angle_left(257, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_hall_offset_angle_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(257, left);

    result = ctrl.set_hall_offset_angle_right(148, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_hall_offset_angle_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(148, right);

    // Restore
    result = ctrl.set_hall_offset_angle_left(0, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_hall_offset_angle_right(0, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_hall_offset_angle_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_hall_offset_angle_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Hall_offset_angle : L[%d]R[%d]", left, right);
}
void Overload_factor(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_overload_factor_left(123, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_overload_factor_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(123, left);

    result = ctrl.set_overload_factor_right(234, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_overload_factor_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(234, right);

    // Restore
    result = ctrl.set_overload_factor_left(200, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_overload_factor_right(200, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_overload_factor_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_overload_factor_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Overload_factor : L[%d]R[%d]", left, right);
}
void Current_left(void)
{
    ////////////
    bool result;
    double rated   = 0;
    double maximum = 0;
    // Test Setter
    result = ctrl.set_rated_current_left(10, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_maximum_current_left(29, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_current_left(&rated, &maximum);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(10, rated);
    TEST_ASSERT_EQUAL(29, maximum);

    result = ctrl.set_rated_current_left(16, true);
    TEST_ASSERT_FALSE(result);
    result = ctrl.set_maximum_current_left(31, true);
    TEST_ASSERT_FALSE(result);

    ///////////////////////////////////////////
    // Restore
    result = ctrl.set_rated_current_left(15, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_maximum_current_left(30, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_current_left(&rated, &maximum);
    TEST_ASSERT_TRUE(result);
    log_d("* Maximum_current : Left : rated[%8.3f]/maximum[%8.3f]", rated, maximum);
}
void Current_right(void)
{
    ////////////
    bool result;
    double rated   = 0;
    double maximum = 0;
    // Test Setter
    result = ctrl.set_rated_current_right(10, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_maximum_current_right(29, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_current_right(&rated, &maximum);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(10, rated);
    TEST_ASSERT_EQUAL(29, maximum);

    result = ctrl.set_rated_current_right(16, true);
    TEST_ASSERT_FALSE(result);
    result = ctrl.set_maximum_current_right(31, true);
    TEST_ASSERT_FALSE(result);

    ///////////////////////////////////////////
    // Restore
    result = ctrl.set_rated_current_right(15, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_maximum_current_right(30, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_current_right(&rated, &maximum);
    TEST_ASSERT_TRUE(result);
    log_d("* Maximum_current : Left : rated[%8.3f]/maximum[%8.3f]", rated, maximum);
}
void Overload_protection_time(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_overload_protection_time_left(217, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_overload_protection_time_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(210, left);

    result = ctrl.set_overload_protection_time_right(365, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_overload_protection_time_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(360, right);

    // Restore
    result = ctrl.set_overload_protection_time_left(300, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_overload_protection_time_right(300, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_overload_protection_time_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_overload_protection_time_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Overload_protection_time : L[%d]R[%d]", left, right);
}
void Position_following_error_threshold(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_position_following_error_threshold_left(317, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_position_following_error_threshold_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(310, left);

    result = ctrl.set_position_following_error_threshold_right(261, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_position_following_error_threshold_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(260, right);

    // Restore
    result = ctrl.set_position_following_error_threshold_left(409, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_position_following_error_threshold_right(409, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_position_following_error_threshold_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_position_following_error_threshold_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Position_following_error_threshold : L[%d]R[%d]", left, right);
}
void Velocity_smoothing_factor(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_velocity_smoothing_factor_left(853, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_velocity_smoothing_factor_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(853, left);

    result = ctrl.set_velocity_smoothing_factor_right(732, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_velocity_smoothing_factor_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(732, right);

    // Restore
    result = ctrl.set_velocity_smoothing_factor_left(1000, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_velocity_smoothing_factor_right(1000, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_velocity_smoothing_factor_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_velocity_smoothing_factor_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Velocity_smoothing_factor : L[%d]R[%d]", left, right);
}
void Current_loop(void)
{
    ////////////
    bool result;
    int left_kp  = 0;
    int left_ki  = 0;
    int right_kp = 0;
    int right_ki = 0;
    // Test Setter
    result = ctrl.set_current_loop_left(572, 287, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_current_loop_left(&left_kp, &left_ki);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(572, left_kp);
    TEST_ASSERT_EQUAL(287, left_ki);

    result = ctrl.set_current_loop_right(462, 143, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_current_loop_right(&right_kp, &right_ki);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(462, right_kp);
    TEST_ASSERT_EQUAL(143, right_ki);

    // Restore
    result = ctrl.set_current_loop_left(600, 300, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_current_loop_right(600, 300, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_current_loop_left(&left_kp, &left_ki);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_current_loop_right(&right_kp, &right_ki);
    TEST_ASSERT_TRUE(result);
    log_d("* Current_loop : Left : kp[%d]ki[%d]", left_kp, left_ki);
    log_d("* Current_loop : Right : kp[%d]ki[%d]", right_kp, right_ki);
}
void Feedforward_output_smoothing_factor(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_feedforward_output_smoothing_factor_left(2853, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_feedforward_output_smoothing_factor_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(2853, left);

    result = ctrl.set_feedforward_output_smoothing_factor_right(2732, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_feedforward_output_smoothing_factor_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(2732, right);

    // Restore
    result = ctrl.set_feedforward_output_smoothing_factor_left(100, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_feedforward_output_smoothing_factor_right(100, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_feedforward_output_smoothing_factor_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_feedforward_output_smoothing_factor_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Initial_velocity : L[%d]R[%d]", left, right);
}
void Torque_output_smoothing_factor(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_torque_output_smoothing_factor_left(1853, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_torque_output_smoothing_factor_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(1853, left);

    result = ctrl.set_torque_output_smoothing_factor_right(1732, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_torque_output_smoothing_factor_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(1732, right);

    // Restore
    result = ctrl.set_torque_output_smoothing_factor_left(100, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_torque_output_smoothing_factor_right(100, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_torque_output_smoothing_factor_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_torque_output_smoothing_factor_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Initial_velocity : L[%d]R[%d]", left, right);
}
void Velocity_Loop(void)
{
    ////////////
    bool result;
    int left_kp  = 0;
    int left_ki  = 0;
    int left_kf  = 0;
    int right_kp = 0;
    int right_ki = 0;
    int right_kf = 0;
    // Test Setter
    result = ctrl.set_velocity_loop_left(123, 456, 789, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_velocity_loop_left(&left_kp, &left_ki, &left_kf);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(123, left_kp);
    TEST_ASSERT_EQUAL(456, left_ki);
    TEST_ASSERT_EQUAL(789, left_kf);

    result = ctrl.set_velocity_loop_right(234, 567, 891, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_velocity_loop_right(&right_kp, &right_ki, &right_kf);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(234, right_kp);
    TEST_ASSERT_EQUAL(567, right_ki);
    TEST_ASSERT_EQUAL(891, right_kf);

    // Restore
    result = ctrl.set_velocity_loop_left(500, 100, 500, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_velocity_loop_right(500, 100, 500, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_velocity_loop_left(&left_kp, &left_ki, &left_kf);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_velocity_loop_right(&right_kp, &right_ki, &right_kf);
    TEST_ASSERT_TRUE(result);
    log_d("* Position_Loop : Left : kp[%d]ki[%d]kf[%d]", left_kp, left_ki, left_kf);
    log_d("* Position_Loop : Right : kp[%d]ki[%d]kf[%d]", right_kp, right_ki, right_kf);
}
void Position_Loop(void)
{
    ////////////
    bool result;
    int left_kp  = 0;
    int left_kf  = 0;
    int right_kp = 0;
    int right_kf = 0;
    // Test Setter
    result = ctrl.set_position_loop_left(113, 1123, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_position_loop_left(&left_kp, &left_kf);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(113, left_kp);
    TEST_ASSERT_EQUAL(1123, left_kf);

    result = ctrl.set_position_loop_right(191, 1231, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_position_loop_right(&left_kp, &left_kf);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(191, left_kp);
    TEST_ASSERT_EQUAL(1231, left_kf);

    // Restore
    result = ctrl.set_position_loop_left(100, 1000, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_position_loop_right(100, 1000, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_position_loop_left(&left_kp, &left_kf);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_position_loop_right(&right_kp, &right_kf);
    TEST_ASSERT_TRUE(result);
    log_d("* Position_Loop : Left : kp[%d]kf[%d]", left_kp, left_kf);
    log_d("* Position_Loop : Right : kp[%d]kf[%d]", right_kp, right_kf);
}
void Initial_velocity(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    // TORQUE
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::TORQUE);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_initial_velocity_left(11, true);
    TEST_ASSERT_FALSE(result);
    result = ctrl.set_initial_velocity_right(12, true);
    TEST_ASSERT_FALSE(result);

    // VELOCITY
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::VELOCITY);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_initial_velocity_left(32, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_initial_velocity_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(32, left);

    result = ctrl.set_initial_velocity_right(35, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_initial_velocity_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(35, right);

    // POSITION_RELATIVE
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_RELATIVE);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_initial_velocity_left(46, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_initial_velocity_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(46, left);

    result = ctrl.set_initial_velocity_right(47, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_initial_velocity_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(47, right);

    // POSITION_ABSOLUTE
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_ABSOLUTE);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_initial_velocity_left(52, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_initial_velocity_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(52, left);

    result = ctrl.set_initial_velocity_right(55, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_initial_velocity_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(55, right);

    // Restore
    result = ctrl.set_initial_velocity_left(1, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_initial_velocity_right(1, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_initial_velocity_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_initial_velocity_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Initial_velocity : L[%d]R[%d]", left, right);
}
void Motor_poles(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    // Test Setter
    result = ctrl.set_motor_poles_left(39, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_motor_poles_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(39, left);

    result = ctrl.set_motor_poles_right(41, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_motor_poles_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(41, right);

    // Restore
    result = ctrl.set_motor_poles_left(15, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_motor_poles_right(15, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_motor_poles_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_motor_poles_right(&right);
    TEST_ASSERT_TRUE(result);
    log_d("* Velocity_smoothing_factor : L[%d]R[%d]", left, right);
}
void Over_temperature_threshold(void)
{
    ////////////
    bool result;
    double left  = 0;
    double right = 0;
    // Test Setter
    result = ctrl.set_over_temperature_threshold_left(51.2, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_over_temperature_threshold_left(&left);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(51.2, left);

    result = ctrl.set_over_temperature_threshold_right(52.1, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_over_temperature_threshold_right(&right);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(52.1, right);

    // Restore
    result = ctrl.set_over_temperature_threshold_left(80, true);
    TEST_ASSERT_TRUE(result);
    result = ctrl.set_over_temperature_threshold_right(80, true);
    TEST_ASSERT_TRUE(result);

    result = ctrl.get_over_temperature_threshold_left(&left);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_over_temperature_threshold_right(&right);
    TEST_ASSERT_TRUE(result);
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
#if CONTROL_PARAMETER | COMPILE_TEST_FUNCTION
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
    result = ctrl.set_max_speed(60, 60);
    TEST_ASSERT_TRUE(result);
    result = ctrl.get_max_speed(&left, &right);
    TEST_ASSERT_TRUE(result);
    log_d("* Max_speed : L[%d]R[%d]", left, right);
}
void Target_velocity(void)
{
    ////////////
    bool result;
    int left  = 0;
    int right = 0;
    result    = ctrl.set_control_mode(ZLAC::DRIVER_MODE::VELOCITY);
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
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::UNDEFINED);
    TEST_ASSERT_TRUE(result);
}
void Target_position_absolute(void)
{
    ////////////
    bool result;
    long left  = 0;
    long right = 0;
    TEST_ASSERT_TRUE(ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED));
    TEST_ASSERT_TRUE(ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_ABSOLUTE));
#if MOTOR_PARAMETER_RUNNING
#else
#endif
}
void Target_position_relative_asynchronous(void)
{
}
void Target_position_relative_synchronous(void)
{
    ////////////
    bool result;
    long left      = 0;
    long right     = 0;
    double left_d  = 0;
    double right_d = 0;
    TEST_ASSERT_TRUE_MESSAGE(ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED), "Control word");
    log_d("* Control word : %s", text_zlac_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED).c_str());
    TEST_ASSERT_TRUE_MESSAGE(ctrl.set_control_mode(ZLAC::DRIVER_MODE::POSITION_RELATIVE), "MODE[POSITION_RELATIVE]");
    log_d("* Control mode : %s", text_zlac_driver_mode(ZLAC::DRIVER_MODE::POSITION_RELATIVE).c_str());
    TEST_ASSERT_TRUE_MESSAGE(ctrl.set_s_shape_acceleration_time(500, 500), "set acceleration time");
    TEST_ASSERT_TRUE_MESSAGE(ctrl.set_s_shape_deceleration_time(500, 500), "set deceleration time");

#if MOTOR_PARAMETER_RUNNING
    TEST_ASSERT_TRUE_MESSAGE(ctrl.set_clear_feedback_position(ZLAC::target_motor::TARGET_MOTOR_ALL), "clear feedback position");

    // Target velocity 2088h
    ctrl.set_target_velocity(100, 100);
    ctrl.set_target_position(100, 100);
    TEST_ASSERT_TRUE_MESSAGE(ctrl.set_synchronous_control_status(true), "synchronous");
    // Enable (0x200e)
    ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE);
    // Target velocity to    100RPM (0x2088)
    ctrl.set_target_velocity(10, 10);
    // Target velocity to    -100RPM
    // Stop (0x200e)
    delay(1000 * 2);
    ctrl.set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_STOP);
    // Actual velocity 20ABh
    ctrl.get_actual_velocity(&left_d, &right_d);
    log_d("* Target_position_relative_synchronous : L[%f]R[%f]", left_d, right_d);

#if 1
    TEST_ASSERT_TRUE(ctrl.set_target_position(0, 0, true));
    TEST_ASSERT_TRUE(ctrl.get_target_position(&left, &right));
    TEST_ASSERT_EQUAL(0, left);
    TEST_ASSERT_EQUAL(0, right);
#endif
#else
#if 0
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
#endif
#endif
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
#if READ_ONLY_PARAMETER | COMPILE_TEST_FUNCTION
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
    log_i("* Error_code : L : %s", left.no_error ? "No error" : "Error");
    if (true == left.over_voltage) {
        log_i("  * [%s]over_voltage", left.over_voltage ? "OVER" : "-");
    }
    if (true == left.under_voltage) {
        log_i("  * [%s]under_voltage", left.under_voltage ? "UNDER" : "-");
    }
    if (true == left.over_current) {
        log_i("  * [%s]over_current", left.over_current ? "T" : "F");
    }
    if (true == left.over_load) {
        log_i("  * [%s]over_load", left.over_load ? "T" : "F");
    }
    if (true == left.current_out_of_tolerance) {
        log_i("  * [%s]current_out_of_tolerance", left.current_out_of_tolerance ? "T" : "F");
    }
    if (true == left.encoder_out_of_tolerance) {
        log_i("  * [%s]encoder_out_of_tolerance", left.encoder_out_of_tolerance ? "T" : "F");
    }
    if (true == left.velocity_out_of_tolerance) {
        log_i("  * [%s]velocity_out_of_tolerance", left.velocity_out_of_tolerance ? "T" : "F");
    }
    if (true == left.reference_voltage_error) {
        log_i("  * [%s]reference_voltage_error", left.reference_voltage_error ? "T" : "F");
    }
    if (true == left.eeprom_error) {
        log_i("  * [%s]eeprom_error", left.eeprom_error ? "T" : "F");
    }
    if (true == left.hall_error) {
        log_i("  * [%s]hall_error", left.hall_error ? "T" : "F");
    }
    if (true == left.motor_temperature_over_temperature) {
        log_i("  * [%s]motor_temperature_over_temperature", left.motor_temperature_over_temperature ? "T" : "F");
    }

    log_i("* Error_code : R : %s", right.no_error ? "No error" : "Error");
    if (true == right.over_voltage) {
        log_i("  * [%s]over_voltage", right.over_voltage ? "OVER" : "-");
    }
    if (true == right.under_voltage) {
        log_i("  * [%s]under_voltage", right.under_voltage ? "UNDER" : "-");
    }
    if (true == right.over_current) {
        log_i("  * [%s]over_current", right.over_current ? "T" : "F");
    }
    if (true == right.over_load) {
        log_i("  * [%s]over_load", right.over_load ? "T" : "F");
    }
    if (true == right.current_out_of_tolerance) {
        log_i("  * [%s]current_out_of_tolerance", right.current_out_of_tolerance ? "T" : "F");
    }
    if (true == right.encoder_out_of_tolerance) {
        log_i("  * [%s]encoder_out_of_tolerance", right.encoder_out_of_tolerance ? "T" : "F");
    }
    if (true == right.velocity_out_of_tolerance) {
        log_i("  * [%s]velocity_out_of_tolerance", right.velocity_out_of_tolerance ? "T" : "F");
    }
    if (true == right.reference_voltage_error) {
        log_i("  * [%s]reference_voltage_error", right.reference_voltage_error ? "T" : "F");
    }
    if (true == right.eeprom_error) {
        log_i("  * [%s]eeprom_error", right.eeprom_error ? "T" : "F");
    }
    if (true == right.hall_error) {
        log_i("  * [%s]hall_error", right.hall_error ? "T" : "F");
    }
    if (true == right.motor_temperature_over_temperature) {
        log_i("  * [%s]motor_temperature_over_temperature", right.motor_temperature_over_temperature ? "T" : "F");
    }
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
    bool result;
    log_d("=== TEST_ONCE ===");
    result = ctrl.clear_fault();
    (void)Error_code();
}
void TEST_RUNNING(void)
{
    log_d("=== TEST_RUNNING ===");
}
void STOP_MOTOR(void)
{
    delay(1000);
    bool result;
    log_d("=== STOP_MOTOR ===");
    result = ctrl.set_control_mode(ZLAC::DRIVER_MODE::UNDEFINED);
    TEST_ASSERT_TRUE(result);
#if RESTORE_FACTORY_SETTINGS
    ctrl.restore_factory_settings();
#endif
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
        RUN_TEST(Max_speed);
        RUN_TEST(Target_velocity);
        RUN_TEST(Target_position_absolute);
        RUN_TEST(Target_position_relative_synchronous);
        RUN_TEST(Target_position_relative_asynchronous);
        RUN_TEST(Target_torque);
#endif
#if MOTOR_PARAMETER
        // Motor parameter
        log_d("=== Motor parameter ===");
        RUN_TEST(Encoder_line);
        RUN_TEST(Hall_offset_angle);
        RUN_TEST(Overload_factor);
        RUN_TEST(Current_left);
        RUN_TEST(Current_right);
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
        RUN_TEST(Control_mode);
        RUN_TEST(Control_word);
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
