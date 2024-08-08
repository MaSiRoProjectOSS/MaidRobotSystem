/**
 * @file main.cpp
 * @brief
 * @version 0.24.06
 * @date 2024-08-08
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef PIO_UNIT_TESTING
#include "lib_zlac8015d_modbus.hpp"

#include <M5Atom.h>

class ImplZLAC8015DModbus : public LibZLAC8015DModbus {
    bool _reception(MessageFrame &frame) override
    {
        log_v("              ADR[0x%02X] Fun[0x%02X] Len[%d] CRC[0x%04X] Data[%02X %02X %02X %02X %02X %02X %02X %02X]",
              frame.address,
              frame.function,
              frame.data_length,
              frame.footer,
              frame.data[0],
              frame.data[1],
              frame.data[2],
              frame.data[3],
              frame.data[4],
              frame.data[5],
              frame.data[6],
              frame.data[7]);
        return true;
    }
};
ImplZLAC8015DModbus ctrl;

bool Check_no_error(void)
{
    bool result = false;
    LibZLAC8015DModbus::zlac_error left;
    LibZLAC8015DModbus::zlac_error right;
    if (true == ctrl.get_error_code(&left, &right)) {
        if (true == left.no_error) {
            if (true == right.no_error) {
                result = true;
            }
        }
    }
    if (false == result) {
        log_e("* Error : %s : L[%d/%s]R[%d/%s]",
              result ? "T" : "F", //
              left.err_value,
              left.no_error ? "T" : "F",
              right.err_value,
              right.no_error ? "T" : "F");
    }
    return result;
}
void Error_code(void)
{
    LibZLAC8015DModbus::zlac_error left;
    LibZLAC8015DModbus::zlac_error right;
    bool result = ctrl.get_error_code(&left, &right);
    log_i("* Error_code : L : %s[%d]", left.no_error ? "No error" : "Error", left.err_value);
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

    log_i("* Error_code : R : %s[%d]", right.no_error ? "No error" : "Error", right.err_value);
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
bool Wait_zero_velocity(int span_ms = 1000, int timeout_ms = (10 * 1000))
{
    ////////////
    bool result = false;

    double left_d  = 0;
    double right_d = 0;
    int time_out   = timeout_ms / span_ms;
    for (int i = 0; i < time_out; i++) {
        delay(span_ms);
        ctrl.get_actual_velocity(&left_d, &right_d);
        if (0.1 > std::abs(left_d)) {
            if (0.1 > std::abs(right_d)) {
                result = true;
                break;
            }
        }
    }
    return result;
}
void Target_velocity(void)
{
    ////////////
    bool result;
    int result_step = 0;
    long left       = 0;
    long right      = 0;
    double left_d   = 0;
    double right_d  = 0;

    ctrl.set_control_word(LibZLAC8015DModbus::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED);
    ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::VELOCITY);
    ctrl.set_s_shape_acceleration_time(500, 500);
    ctrl.set_s_shape_deceleration_time(500, 500);
    ctrl.set_target_velocity(0, 0);
    ctrl.get_actual_velocity(&left_d, &right_d);
    log_d("* Target_velocity : Get actual : L[%f]R[%f]", left_d, right_d);
    ctrl.set_control_word(LibZLAC8015DModbus::ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE);
    delay(1000);
    result_step += ctrl.set_target_velocity(60, 0) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_velocity(0, 60) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_velocity(0, 0) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_velocity(-60, 0) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_velocity(0, -60) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_velocity(0, 0) ? 1 : 0;
    delay(1000);
    result = ctrl.set_control_word(LibZLAC8015DModbus::ZLAC_CONTROL_WORD::CONTROL_WORD_STOP);
}
void Target_position_absolute_asynchronous(void)
{
    ////////////
    bool result;
    bool result_no_error;
    int cnt_step    = 0;
    int result_step = 0;
    long left       = 0;
    long right      = 0;
    double left_d   = 0;
    double right_d  = 0;

    ctrl.set_control_word(LibZLAC8015DModbus::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED);
    ctrl.set_synchronous_control_status(false);
    ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_ABSOLUTE);
    ctrl.set_s_shape_acceleration_time(500, 500);
    ctrl.set_s_shape_deceleration_time(500, 500);
    ctrl.set_target_position(0, 0);
    ctrl.set_max_speed(60, 60);
    ctrl.get_actual_velocity(&left_d, &right_d);
    log_d("* Target_position_absolute_asynchronous : Get actual : L[%f]R[%f]", left_d, right_d);
    ctrl.set_clear_feedback_position(LibZLAC8015DModbus::MODBUS_TARGET_MOTOR::TARGET_MOTOR_ALL);
    ctrl.control_word_enable();
    result_step += ctrl.set_target_position(10000, 10000) ? 1 : 0;
    result_step += ctrl.control_word_start_left() ? 1 : 0;
    result_step += ctrl.control_word_start_right() ? 1 : 0;
    Wait_zero_velocity();
    result_no_error = Check_no_error();
    cnt_step        = __LINE__;
    log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(1000, 10000) ? 1 : 0;
        result_step += ctrl.control_word_start_left() ? 1 : 0;
        result_step += ctrl.control_word_start_right() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(-10000, 0) ? 1 : 0;
        result_step += ctrl.control_word_start_left() ? 1 : 0;
        result_step += ctrl.control_word_start_right() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(0, 0) ? 1 : 0;
        result_step += ctrl.control_word_start_left() ? 1 : 0;
        result_step += ctrl.control_word_start_right() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(0, -10000) ? 1 : 0;
        result_step += ctrl.control_word_start_left() ? 1 : 0;
        result_step += ctrl.control_word_start_right() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(-10000, 0) ? 1 : 0;
        result_step += ctrl.control_word_start_left() ? 1 : 0;
        result_step += ctrl.control_word_start_right() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    result = ctrl.control_word_stop();
}
void Target_position_absolute_synchronous(void)
{
    ////////////
    bool result;
    bool result_no_error;
    int cnt_step    = 0;
    int result_step = 0;
    long left       = 0;
    long right      = 0;
    double left_d   = 0;
    double right_d  = 0;

    ctrl.control_word_none();
    ctrl.set_synchronous_control_status(true);
    ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_ABSOLUTE);
    ctrl.set_s_shape_acceleration_time(500, 500);
    ctrl.set_s_shape_deceleration_time(500, 500);
    ctrl.set_target_position(0, 0);
    ctrl.set_max_speed(60, 60);
    ctrl.get_actual_velocity(&left_d, &right_d);
    log_d("* Target_position_absolute_synchronous : Get actual : L[%f]R[%f]", left_d, right_d);
    ctrl.set_clear_feedback_position(LibZLAC8015DModbus::MODBUS_TARGET_MOTOR::TARGET_MOTOR_ALL);
    ctrl.control_word_enable();
    result_step += ctrl.set_target_position(10000, 10000) ? 1 : 0;
    result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
    Wait_zero_velocity();
    result_no_error = Check_no_error();
    cnt_step        = __LINE__;
    log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(1000, 10000) ? 1 : 0;
        result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(-10000, 0) ? 1 : 0;
        result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(0, 0) ? 1 : 0;
        result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(0, -10000) ? 1 : 0;
        result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    if (true == result_no_error) {
        result_step += ctrl.set_target_position(-10000, 0) ? 1 : 0;
        result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
        Wait_zero_velocity();
        result_no_error = Check_no_error();
        cnt_step        = __LINE__;
        log_i("LINE[%d] : %s", cnt_step, result_no_error ? "OK" : "NG");
    }
    result = ctrl.control_word_stop();
}

void Target_position_relative_asynchronous(void)
{
    ////////////
    bool result;
    int result_step = 0;
    long left       = 0;
    long right      = 0;
    double left_d   = 0;
    double right_d  = 0;

    ctrl.set_control_word(LibZLAC8015DModbus::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED);
    ctrl.set_synchronous_control_status(false);
    ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_RELATIVE);
    ctrl.set_s_shape_acceleration_time(500, 500);
    ctrl.set_s_shape_deceleration_time(500, 500);
    ctrl.set_target_position(0, 0);
    ctrl.set_max_speed(60, 60);
    ctrl.get_actual_velocity(&left_d, &right_d);
    log_d("* Target_position_relative_asynchronous : Get actual : L[%f]R[%f]", left_d, right_d);
    ctrl.set_clear_feedback_position(LibZLAC8015DModbus::MODBUS_TARGET_MOTOR::TARGET_MOTOR_ALL);
    ctrl.control_word_enable();
    result_step += ctrl.set_target_position(10000, 10000) ? 1 : 0;
    result_step += ctrl.control_word_start_left() ? 1 : 0;
    result_step += ctrl.control_word_start_right() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(10000, 10000) ? 1 : 0;
    result_step += ctrl.control_word_start_left() ? 1 : 0;
    result_step += ctrl.control_word_start_right() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(10000, -10000) ? 1 : 0;
    result_step += ctrl.control_word_start_left() ? 1 : 0;
    result_step += ctrl.control_word_start_right() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(-10000, -10000) ? 1 : 0;
    result_step += ctrl.control_word_start_left() ? 1 : 0;
    result_step += ctrl.control_word_start_right() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(-10000, 10000) ? 1 : 0;
    result_step += ctrl.control_word_start_left() ? 1 : 0;
    result_step += ctrl.control_word_start_right() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(0, 10000) ? 1 : 0;
    result_step += ctrl.control_word_start_left() ? 1 : 0;
    result_step += ctrl.control_word_start_right() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(10000, 0) ? 1 : 0;
    result_step += ctrl.control_word_start_left() ? 1 : 0;
    result_step += ctrl.control_word_start_right() ? 1 : 0;
    Wait_zero_velocity();
    result = ctrl.control_word_stop();
}
void Target_position_relative_synchronous(void)
{
    ////////////
    bool result;
    int result_step = 0;
    long left       = 0;
    long right      = 0;
    double left_d   = 0;
    double right_d  = 0;

    ctrl.control_word_none();
    ctrl.set_synchronous_control_status(true);
    ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_RELATIVE);
    ctrl.set_s_shape_acceleration_time(500, 500);
    ctrl.set_s_shape_deceleration_time(500, 500);
    ctrl.set_target_position(0, 0);
    ctrl.set_max_speed(60, 60);
    ctrl.get_actual_velocity(&left_d, &right_d);
    log_d("* Target_position_relative_synchronous : Get actual : L[%f]R[%f]", left_d, right_d);
    ctrl.set_clear_feedback_position(LibZLAC8015DModbus::MODBUS_TARGET_MOTOR::TARGET_MOTOR_ALL);
    ctrl.control_word_enable();
    result_step += ctrl.set_target_position(10000, 10000) ? 1 : 0;
    result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(10000, 10000) ? 1 : 0;
    result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(10000, -10000) ? 1 : 0;
    result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(-10000, -10000) ? 1 : 0;
    result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(-10000, 10000) ? 1 : 0;
    result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(0, 10000) ? 1 : 0;
    result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
    Wait_zero_velocity();
    result_step += ctrl.set_target_position(10000, 0) ? 1 : 0;
    result_step += ctrl.control_word_synchronous_start() ? 1 : 0;
    Wait_zero_velocity();
    result = ctrl.control_word_stop();
}
void Target_torque(void)
{
    ////////////
    bool result;
    int result_step = 0;
    long left       = 0;
    long right      = 0;
    double left_d   = 0;
    double right_d  = 0;
    ctrl.set_control_word(LibZLAC8015DModbus::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED);
    ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::TORQUE);
    ctrl.set_torque_slope(300, 300);
    ctrl.set_target_torque(0, 0);
    ctrl.get_actual_torque(&left_d, &right_d);
    log_d("* Target_torque : Get actual : L[%f]R[%f]", left_d, right_d);
    ctrl.set_control_word(LibZLAC8015DModbus::ZLAC_CONTROL_WORD::CONTROL_WORD_ENABLE);
    delay(1000);
    result_step += ctrl.set_target_torque(1000, 0) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_torque(0, 1000) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_torque(0, 0) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_torque(-1000, 0) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_torque(0, -1000) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_torque(-1000, -1000) ? 1 : 0;
    delay(1000);
    result_step += ctrl.set_target_torque(0, 0) ? 1 : 0;
    delay(1000);
    result = ctrl.set_control_word(LibZLAC8015DModbus::ZLAC_CONTROL_WORD::CONTROL_WORD_STOP);
}
///////////////
void setup()
{
    bool enable_serial  = true;
    bool enable_i2c     = false;
    bool enable_display = true;
    (void)M5.begin(enable_serial, enable_i2c, enable_display);
    (void)M5.dis.begin();
    (void)M5.dis.fillpix(CRGB::White);
    delay(200);
    (void)M5.dis.fillpix(CRGB::Black);
    delay(200);
    (void)M5.dis.fillpix(CRGB::White);
    log_d("========================================");
    log_d("M5Atom initialized.");
    log_d("  - Start Modbus. Address[%d]", MODBUS_ADDRESS);
    bool flag = ctrl.begin(&Serial1, MODBUS_ADDRESS, MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU);
    log_d("========================================");
    if (true == flag) {
        bool connected = false;
        bool result    = ctrl.get_software_connected_status(&connected);
        if (false == connected) {
            flag = false;
        }
    }
    if (false == flag) {
        log_e("Modbus initialize error.");
        (void)M5.dis.fillpix(CRGB::Red);
        while (1) {
            delay(1000);
        }
    }
    ctrl.clear_fault();
    (void)M5.dis.fillpix(CRGB::Green);
}
void loop()
{
    static int type = 0;
    (void)M5.update();
    if (M5.Btn.wasPressed()) {
        (void)M5.dis.fillpix(CRGB::Yellow);
        type++;
        switch (type) {
            case 1:
                Target_velocity();
                log_i("=== Target_velocity ===");
                break;
            case 2:
                Target_position_absolute_asynchronous();
                log_i("=== Target_position_absolute_asynchronous ===");
                break;
            case 3:
                Target_position_absolute_synchronous();
                log_i("=== Target_position_absolute_synchronous ===");
                break;
            case 4:
                Target_position_relative_synchronous();
                log_i("=== Target_position_relative_synchronous ===");
                break;
            case 5:
                Target_position_relative_asynchronous();
                log_i("=== Target_position_relative_asynchronous ===");
                break;

            default:
                Target_torque();
                log_i("=== Target_torque ===");
                type = 0;
                break;
        }
        (void)Error_code();
        ctrl.clear_fault();
        (void)M5.dis.fillpix(CRGB::Green);
    }
}
#endif
