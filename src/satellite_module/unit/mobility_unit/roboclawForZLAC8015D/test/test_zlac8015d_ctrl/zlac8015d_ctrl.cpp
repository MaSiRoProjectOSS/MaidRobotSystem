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
#define COMMON_CONSTANT     0
#define MOTOR_PARAMETER     0
#define CONTROL_PARAMETER   0
#define READ_ONLY_PARAMETER 0
////
//#include <HTTPClient.h>
#include <M5Atom.h>
//#include <Update.h>
//#include <WiFi.h>
////#include <Arduino.h>
#include <unity.h>
///////////////////////////////////////////////////////////////////
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
    log_i("========================================");
    log_i("M5Atom initialized.");
    log_i("  - Start Modbus. Address[%d]", MODBUS_ADDRESS);
    bool flag = ctrl.begin(&Serial1, MODBUS_ADDRESS, MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU);
    log_i("========================================");
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

///////////////////////////////////////////////////////////////////
void Communication_offline_time(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void RS485_Node_ID(void)
{
    int value = ctrl.get_rs485_node_id();
    log_i("* RS485 Node ID : %d", value);
    TEST_ASSERT_EQUAL_INT(1, value);
}
void RS485_Baud_Rate(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Input_signal_status(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Out_signal_status(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Clear_feedback_position(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void In_absolute_position_control_reset_the_zero_point(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Shaft_state_after_power_on(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Maximum_motor_speed(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Register_parameter_settings(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void CAN_Node_ID(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void CAN_Baud_rate(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Control_mode(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Control_word(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Synchronous_control_status(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Whether_store_RW_register_to_EEPROM(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Quick_stop_control(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Close_operation_control(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Disable_control(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Halt_control(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Input_effective_level(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Input_terminal_X0_terminal_function_selection(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Input_terminal_X1_terminal_function_selection(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Output_effective_level(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Output_terminal_B0_terminal_function_selection(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Output_terminal_B1_terminal_function_selection(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Output_terminal_Y0_terminal_function_selection(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Output_terminal_Y1_terminal_function_selection(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Driver_temperature_protection_threshold(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Alarm_PWM_processing_method(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Overload_processing_method(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void IO_emergency_stop_processing_mode(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Encoder_line(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Hall_offset_angle(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Overload_factor(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Rated_current(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Maximum_current(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Overload_protection_time(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Position_following_error_threshold(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Velocity_smoothing_factor(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Cl_Kp(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Cl_Ki(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Feedforward_output_smoothing_factor(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Torque_output_smoothing_factor(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Velocity_Loop_Kp(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Velocity_Loop_Ki(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Velocity_Loop_Kf(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Position_Loop_Kp(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Position_Loop_Kf(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Initial_velocity(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Motor_poles(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Over_temperature_threshold(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Velocity_observer_coefficient_1(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Velocity_observer_coefficient_2(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Velocity_observer_coefficient_3(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Velocity_observer_coefficient_4(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void S_shape_acceleration_time(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void S_shape_deceleration_time(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Deceleration_time_of_quick_stop(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Torque_slope(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Target_velocity(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Target_position(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Max_speed(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Target_torque(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Software_version(void)
{
    int value = ctrl.get_software_version();
    log_i("* Software version : %d", value);
    TEST_ASSERT_EQUAL_INT(23423, value);
}
void Bus_voltage(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Status_word(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Hall_input_state(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Motor_temperature(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Error_code(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Actual_motor_position(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Actual_velocity(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Actual_torque(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Software_connected_status(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}
void Driver_temperature(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}

///////////////////////////////////////////////////////////////////

void RUN_UNITY_TESTS()
{
    (void)setup_m5();
    UNITY_BEGIN();
    //////////////////////////////////
    // Common constant
#if COMMON_CONSTANT
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
    RUN_TEST(CAN_Node_ID);
    RUN_TEST(CAN_Baud_rate);
    RUN_TEST(Control_mode);
    RUN_TEST(Control_word);
    RUN_TEST(Synchronous_control_status);
    RUN_TEST(Whether_store_RW_register_to_EEPROM);
    RUN_TEST(Quick_stop_control);
    RUN_TEST(Close_operation_control);
    RUN_TEST(Disable_control);
    RUN_TEST(Halt_control);
    RUN_TEST(Input_effective_level);
    RUN_TEST(Input_terminal_X0_terminal_function_selection);
    RUN_TEST(Input_terminal_X1_terminal_function_selection);
    RUN_TEST(Output_effective_level);
    RUN_TEST(Output_terminal_B0_terminal_function_selection);
    RUN_TEST(Output_terminal_B1_terminal_function_selection);
    RUN_TEST(Output_terminal_Y0_terminal_function_selection);
    RUN_TEST(Output_terminal_Y1_terminal_function_selection);
    RUN_TEST(Driver_temperature_protection_threshold);
    RUN_TEST(Alarm_PWM_processing_method);
    RUN_TEST(Overload_processing_method);
    RUN_TEST(IO_emergency_stop_processing_mode);
#endif
#if MOTOR_PARAMETER
    // Motor parameter
    RUN_TEST(Encoder_line);
    RUN_TEST(Hall_offset_angle);
    RUN_TEST(Overload_factor);
    RUN_TEST(Rated_current);
    RUN_TEST(Maximum_current);
    RUN_TEST(Overload_protection_time);
    RUN_TEST(Position_following_error_threshold);
    RUN_TEST(Velocity_smoothing_factor);
    RUN_TEST(Cl_Kp);
    RUN_TEST(Cl_Ki);
    RUN_TEST(Feedforward_output_smoothing_factor);
    RUN_TEST(Torque_output_smoothing_factor);
    RUN_TEST(Velocity_Loop_Kp);
    RUN_TEST(Velocity_Loop_Ki);
    RUN_TEST(Velocity_Loop_Kf);
    RUN_TEST(Position_Loop_Kp);
    RUN_TEST(Position_Loop_Kf);
    RUN_TEST(Initial_velocity);
    RUN_TEST(Motor_poles);
    RUN_TEST(Over_temperature_threshold);
    RUN_TEST(Velocity_observer_coefficient_1);
    RUN_TEST(Velocity_observer_coefficient_2);
    RUN_TEST(Velocity_observer_coefficient_3);
    RUN_TEST(Velocity_observer_coefficient_4);
#endif
#if CONTROL_PARAMETER
    // Control parameter
    RUN_TEST(S_shape_acceleration_time);
    RUN_TEST(S_shape_deceleration_time);
    RUN_TEST(Deceleration_time_of_quick_stop);
    RUN_TEST(Torque_slope);
    RUN_TEST(Target_velocity);
    RUN_TEST(Target_position);
    RUN_TEST(Max_speed);
    RUN_TEST(Target_torque);
#endif
#if READ_ONLY_PARAMETER
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
    //////////////////////////////////
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

#if 0
#include "driver_zlac/config_zlac.hpp"
#include "driver_zlac/zlac8015d_modbus.hpp"

ZLAC8015DCtrl ctrl;

void setup()
{
    (void)setup_m5();
    log_i("========================================");
    log_i("M5Atom initialized.");
    log_i("  - Start Modbus. Address[%d]", MODBUS_ADDRESS);
    bool flag = ctrl.begin(&Serial1, MODBUS_ADDRESS, MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU);
    log_i("========================================");
    if (false == flag) {
        Serial.println("Failed to initialize RS485.");
        m5_led(CRGB::Red);
        while (1) {
            delay(1000);
        }
    }
}
void loop()
{
    static bool flag = false;
    (void)M5.update();
    if (M5.Btn.wasPressed()) {
        if (true == flag) {
            m5_led(CRGB::Green);
        } else {
            m5_led(CRGB::Blue);
        }
        flag = !flag;

        int value = ctrl.get_rs485_node_id();
        Serial.printf("RS485_Node_ID: %d\n", value);
        m5_led(CRGB::Green);
    }
}
#endif
