/**
 * @file zlac8015d_modbus.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-05
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ZLAC8015D_MODBUS_HPP
#define ZLAC8015D_MODBUS_HPP

#include "lib_zlac8015d_modbus.hpp"
#include "zlac_driver.hpp"

#ifndef ZLAC_MODBUS_TYPE
#define ZLAC_MODBUS_TYPE MessageFrame::MODBUS_TYPE::MODBUS_TYPE_RTU
#endif
#ifndef MODBUS_TARGET_ADDRESS
#define MODBUS_TARGET_ADDRESS 0x01
#endif

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

class ZLAC8015DCtrl : public ZlacDriver {
private:
    ImplZLAC8015DModbus ctrl;

public:
    ZLAC8015DCtrl()
    {
        this->info.left.interval  = SETTING_INTERVAL_LEFT;
        this->info.right.interval = SETTING_INTERVAL_RIGHT;
    }
    ~ZLAC8015DCtrl()
    {
        if (nullptr != this->_serial_driver) {
            this->_serial_driver->end();
        }
    }

public:
    bool setup(HardwareSerial *serial_l, HardwareSerial *serial_r, unsigned long baud = 57600)
    {
        bool result = false;
        try {
            if (nullptr != serial_l) {
                this->_serial_driver = serial_l;
                this->_serial_driver->end();
                this->_serial_driver->begin(baud, SERIAL_8N1, -1, -1, this->_flag_invert, this->TIMEOUT_DRIVER_MS);
                result = ctrl.begin(this->_serial_driver, MODBUS_ADDRESS, ZLAC_MODBUS_TYPE);

                if (true == result) {
                    this->info.system.set(LOG_BEGIN, 0, 0);
                }
            }
            if (nullptr != serial_r) {
                // do nothing
            }
        } catch (...) {
        }
        return result;
    }

public:
    bool begin() override
    {
        bool result = true;
        log_v("%s", __func__);
        this->info.system.set(LOG_BEGIN, 0, 0);

        return result;
    }
    bool loop() override
    {
        bool result = true;
        log_v("%s", __func__);
        return result;
    }

public:
    bool cmd_setting_proportional_gain(ZLAC::TARGET_MOTOR target, int value) override
    {
        log_v("%s", __func__);
        bool result = true;
        int kp;
        int ki;
        int kf;
        LibZLAC8015DModbus::MODBUS_DRIVER_MODE mode = this->ctrl.current_mode();
        switch (mode) {
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_RELATIVE:
                if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                    result = this->ctrl.get_position_loop_right(&kp, &kf);
                    if (true == result) {
                        result = this->ctrl.set_position_loop_right(value, kf);
                    }
                    if (true == result) {
                        this->info.left.position_proportional_gain = value;
                        this->info.left.position_differential_gain = kf;
                    }
                }
                if (true == result) {
                    if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                        result = this->ctrl.get_position_loop_right(&kp, &kf);
                        if (true == result) {
                            result = this->ctrl.set_position_loop_right(value, kf);
                        }
                        if (true == result) {
                            this->info.right.position_proportional_gain = value;
                            this->info.right.position_differential_gain = kf;
                        }
                    }
                }
                break;
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::VELOCITY:
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::TORQUE:
                if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                    result = this->ctrl.get_velocity_loop_left(&kp, &ki, &kf);
                    if (true == result) {
                        result = this->ctrl.set_velocity_loop_left(value, ki, kf);
                    }
                    if (true == result) {
                        this->info.left.speed_proportional_gain = value;
                        this->info.left.speed_integral_gain     = ki;
                        this->info.left.speed_differential_gain = kf;
                    }
                }
                if (true == result) {
                    if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                        result = this->ctrl.get_velocity_loop_right(&kp, &ki, &kf);
                        if (true == result) {
                            result = this->ctrl.set_velocity_loop_right(value, ki, kf);
                        }
                        if (true == result) {
                            this->info.right.speed_proportional_gain = value;
                            this->info.right.speed_integral_gain     = ki;
                            this->info.right.speed_differential_gain = kf;
                        }
                    }
                }
                break;

            default:
                break;
        }

        return result;
    }

    bool cmd_setting_integral_gain(ZLAC::TARGET_MOTOR target, int value)
    {
        log_v("%s", __func__);
        bool result = true;
        int kp;
        int ki;
        int kf;
        LibZLAC8015DModbus::MODBUS_DRIVER_MODE mode = this->ctrl.current_mode();
        switch (mode) {
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_RELATIVE:
                break;
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::VELOCITY:
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::TORQUE:
                if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                    result = this->ctrl.get_velocity_loop_left(&kp, &ki, &kf);
                    if (true == result) {
                        result = this->ctrl.set_velocity_loop_left(kp, value, kf);
                    }
                    if (true == result) {
                        this->info.left.speed_proportional_gain = kp;
                        this->info.left.speed_integral_gain     = value;
                        this->info.left.speed_differential_gain = kf;
                    }
                }
                if (true == result) {
                    if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                        result = this->ctrl.get_velocity_loop_right(&kp, &ki, &kf);
                        if (true == result) {
                            result = this->ctrl.set_velocity_loop_right(kp, value, kf);
                        }
                        if (true == result) {
                            this->info.right.speed_proportional_gain = kp;
                            this->info.right.speed_integral_gain     = value;
                            this->info.right.speed_differential_gain = ki;
                        }
                    }
                }
                break;

            default:
                break;
        }

        return result;
    }

    bool cmd_setting_differential_gain(ZLAC::TARGET_MOTOR target, int value)
    {
        log_v("%s", __func__);
        bool result = true;
        int kp;
        int ki;
        int kf;
        LibZLAC8015DModbus::MODBUS_DRIVER_MODE mode = this->ctrl.current_mode();
        switch (mode) {
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_ABSOLUTE:
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_RELATIVE:
                if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                    result = this->ctrl.get_position_loop_right(&kp, &kf);
                    if (true == result) {
                        result = this->ctrl.set_position_loop_right(kp, value);
                    }
                    if (true == result) {
                        this->info.left.position_proportional_gain = kp;
                        this->info.left.position_differential_gain = value;
                    }
                }
                if (true == result) {
                    if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                        result = this->ctrl.get_position_loop_right(&kp, &kf);
                        if (true == result) {
                            result = this->ctrl.set_position_loop_right(kp, value);
                        }
                        if (true == result) {
                            this->info.right.position_proportional_gain = kp;
                            this->info.right.position_differential_gain = value;
                        }
                    }
                }
                break;
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::VELOCITY:
            case LibZLAC8015DModbus::MODBUS_DRIVER_MODE::TORQUE:
                if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                    result = this->ctrl.get_velocity_loop_left(&kp, &ki, &kf);
                    if (true == result) {
                        result = this->ctrl.set_velocity_loop_left(kp, ki, value);
                    }
                    if (true == result) {
                        this->info.left.speed_proportional_gain = kp;
                        this->info.left.speed_integral_gain     = ki;
                        this->info.left.speed_differential_gain = value;
                    }
                }
                if (true == result) {
                    if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                        result = this->ctrl.get_velocity_loop_right(&kp, &ki, &kf);
                        if (true == result) {
                            result = this->ctrl.set_velocity_loop_right(kp, ki, value);
                        }
                        if (true == result) {
                            this->info.right.speed_proportional_gain = kp;
                            this->info.right.speed_integral_gain     = ki;
                            this->info.right.speed_differential_gain = value;
                        }
                    }
                }
                break;

            default:
                break;
        }

        return result;
    }

    bool cmd_setting_feed_forward_gain(ZLAC::TARGET_MOTOR target, int value)
    {
        // TODO: implement
        bool result = true;

        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            result = this->ctrl.set_feedforward_output_smoothing_factor_left(value);
            if (true == result) {
                this->info.left.position_feed_forward_gain = value;
            }
        }
        if (true == result) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                result = this->ctrl.set_feedforward_output_smoothing_factor_right(value);
                if (true == result) {
                    this->info.right.position_feed_forward_gain = value;
                }
            }
        }
        return result;
    }

public:
    bool cmd_modify_the_rated_current(int value_mW, ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        ////////////
        bool result_01 = true;
        bool result_02 = true;
        double rated   = 0;
        double maximum = 0;

        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            // result_01 = this->ctrl.set_rated_current_left(value_mW);
            result_01 = this->ctrl.set_maximum_current_left(value_mW);
        }
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            // result_02 = this->ctrl.set_rated_current_right(value_mW);
            result_02 = this->ctrl.set_maximum_current_right(value_mW);
        }
        if (true == (result_01 && result_02)) {
            this->info.rated_current_mW = value_mW;
        }

        return result_01 && result_02;
    }

#if 0
    bool cmd_looking_for_z_signal(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        // TODO: implement
        log_v("%s", __func__);
        return false;
    }
#endif
    bool cmd_clear_fault(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        return ctrl.clear_fault();
    }

    bool cmd_get_alarm_status(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        bool result = false;
        LibZLAC8015DModbus::status_word left_status;
        LibZLAC8015DModbus::status_word right_status;
        LibZLAC8015DModbus::zlac_error left_er;
        LibZLAC8015DModbus::zlac_error right_er;
        result = ctrl.get_status_word(&left_status, &right_status);
        if (true == result) {
            result = ctrl.get_error_code(&left_er, &right_er);
            if (true == result) {
                unsigned long current_time = millis();
                // left
                this->info.left.error.not_connection = false;
                this->info.left.error.stop_state     = left_status.emergency_stop;
                this->info.left.error.startup_state  = left_status.is_run;

                this->info.left.error.over_current  = left_er.over_current;
                this->info.left.error.over_voltage  = left_er.over_voltage;
                this->info.left.error.encoder_error = left_er.encoder_out_of_tolerance;
                this->info.left.error.overheat      = left_er.motor_temperature_over_temperature;
                this->info.left.error.under_voltage = left_er.under_voltage;
                this->info.left.error.overload      = left_er.over_load;
                this->info.left.update_time         = current_time;
                // right
                this->info.right.error.not_connection = false;
                this->info.right.error.stop_state     = right_status.emergency_stop;
                this->info.right.error.startup_state  = right_status.is_run;

                this->info.right.error.over_current  = right_er.over_current;
                this->info.right.error.over_voltage  = right_er.over_voltage;
                this->info.right.error.encoder_error = right_er.encoder_out_of_tolerance;
                this->info.right.error.overheat      = right_er.motor_temperature_over_temperature;
                this->info.right.error.under_voltage = right_er.under_voltage;
                this->info.right.error.overload      = right_er.over_load;
                this->info.right.update_time         = current_time;
            }
        } else {
            this->info.left.error.not_connection  = true;
            this->info.right.error.not_connection = true;
        }

        return result;
    }
    bool cmd_get_bus_voltage(ZLAC::TARGET_MOTOR target)
    {
        bool result;
        double value = NAN;
        result       = ctrl.get_bus_voltage(&value);
        if (true == result) {
            this->info.left.voltage  = value;
            this->info.right.voltage = value;
        }
        return result;
    }

    bool cmd_get_output_current(ZLAC::TARGET_MOTOR target)
    {
        ////////////
        bool result_01 = true;
        bool result_02 = true;
        double rated   = 0;
        double maximum = 0;

        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            result_01 = ctrl.get_current_left(&rated, &maximum);
            if (true == result_01) {
                this->info.left.current = rated;
            }
        }
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            result_02 = ctrl.get_current_right(&rated, &maximum);
            if (true == result_02) {
                this->info.right.current = rated;
            }
        }

        return result_01 && result_02;
    }
    bool cmd_get_motor_speed(ZLAC::TARGET_MOTOR target)
    {
        // TODO: implement
        log_v("%s", __func__);
        static unsigned long next_time = 0;
        unsigned long now_time         = millis();
        bool result                    = true;
        double left_rpm;
        double right_rpm;
        int left_enc;
        int right_enc;
        if (now_time > next_time) {
            result = this->ctrl.get_actual_velocity(&left_rpm, &right_rpm);
            if (true == result) {
                result = this->ctrl.get_encoder_line_left(&left_enc);
            }
            if (true == result) {
                result = this->ctrl.get_encoder_line_right(&right_enc);
            }
            if (true == result) {
                this->info.left.speed_enc  = left_enc;
                this->info.left.speed_rpm  = left_rpm;
                this->info.right.speed_enc = right_enc;
                this->info.right.speed_rpm = right_rpm;
            }
        }
        return result;
    }
    bool cmd_get_position_given(ZLAC::TARGET_MOTOR target)
    {
        ////////////
        bool result = true;
        long left   = 0;
        long right  = 0;

        result = ctrl.get_target_position(&left, &right);
        if (true == result) {
            this->info.left.position_given  = left;
            this->info.right.position_given = right;
        }

        return result;
    }
    bool cmd_get_position_feedback(ZLAC::TARGET_MOTOR target)
    {
        ////////////
        bool result = true;
        long left   = 0;
        long right  = 0;

        result = ctrl.get_actual_motor_position(&left, &right);
        if (true == result) {
            this->info.left.position_feedback      = left;
            this->info.right.position_feedback     = right;
            this->info.left.position_feedback      = left * ((true != this->info.left.interval) ? 1 : -1);
            this->info.left.position_feedback_deg  = ((left % SETTING_SYSTEM_POSITION_AROUND) * 360.0) / SETTING_SYSTEM_POSITION_AROUND;
            this->info.right.position_feedback_deg = ((right % SETTING_SYSTEM_POSITION_AROUND) * 360.0) / SETTING_SYSTEM_POSITION_AROUND;
        }

        return result;
    }

    //////////////////////////////////////
    //////////////////////////////////////

    bool cmd_motor_start(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        return ctrl.control_word_enable();
    }
    bool cmd_motor_stop(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        return ctrl.control_word_stop();
    }
    //////////////////////////////////////

    bool cmd_position_mode()
    {
        bool result = ctrl.set_synchronous_control_status(true);
        if (true == result) {
            result = ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_RELATIVE);
        }
        if (true == result) {
            result = ctrl.set_s_shape_acceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_s_shape_deceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_max_speed(60, 60);
        }
        return result;
    }
    bool cmd_position_mode_pulse()
    {
        bool result = ctrl.set_synchronous_control_status(true);
        if (true == result) {
            result = ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_RELATIVE);
        }
        if (true == result) {
            result = ctrl.set_s_shape_acceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_s_shape_deceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_max_speed(60, 60);
        }
        return result;
    }
    bool cmd_position_set_absolute()
    {
        bool result = ctrl.set_synchronous_control_status(true);
        if (true == result) {
            result = ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_ABSOLUTE);
        }
        if (true == result) {
            result = ctrl.set_s_shape_acceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_s_shape_deceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_max_speed(60, 60);
        }
        return result;
    }
    bool cmd_position_set_relative()
    {
        bool result = ctrl.set_synchronous_control_status(true);
        if (true == result) {
            result = ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::POSITION_RELATIVE);
        }
        if (true == result) {
            result = ctrl.set_s_shape_acceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_s_shape_deceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_max_speed(60, 60);
        }
        return result;
    }
    bool cmd_position_set(long pos_l, int rpm_l, long pos_r, int rpm_r)
    {
        log_v("%s", __func__);
        // TODO :  rpmをステップに変換している。posの範囲はあっている？
        static int MAX  = 6000;
        static int STEP = 16384;
        bool result;

        if (this->info.POSITION_LIMIT < rpm_l) {
            rpm_l = this->info.POSITION_LIMIT;
        } else if (rpm_l < -this->info.POSITION_LIMIT) {
            rpm_l = -this->info.POSITION_LIMIT;
        }
        this->info.left.position_rpm = rpm_l;
        rpm_l                        = (rpm_l * STEP) / MAX;
        if (this->info.POSITION_LIMIT < rpm_r) {
            rpm_r = this->info.POSITION_LIMIT;
        } else if (rpm_r < -this->info.POSITION_LIMIT) {
            rpm_r = -this->info.POSITION_LIMIT;
        }
        this->info.right.position_rpm = rpm_r;
        rpm_r                         = (rpm_r * STEP) / MAX;

        this->info.left.position_request  = pos_l;
        this->info.right.position_request = pos_r;

        if (true == result) {
            result = ctrl.set_max_speed(rpm_l, rpm_r);
        }
        if (true == result) {
            result = ctrl.set_target_position(pos_l, pos_r);
        }
        ctrl.control_word_synchronous_start();
        return result;
    }
    //////////////////////////////////////
    bool cmd_torque_mode()
    {
        bool result = ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::TORQUE);
        if (true == result) {
            result = ctrl.set_torque_slope(300, 300);
        }
        if (true == result) {
            result = ctrl.set_target_torque(0, 0);
        }
        if (true == result) {
            result = ctrl.control_word_enable();
        }

        return result;
    }

    bool cmd_torque_set(int value_l_mA, int value_r_mA)
    {
        log_v("%s", __func__);
        static int flag_running_l = false;
        static int flag_running_r = false;

        if (this->info.TORQUE_LIMIT < value_l_mA) {
            value_l_mA = this->info.TORQUE_LIMIT;
        } else if (value_l_mA < -this->info.TORQUE_LIMIT) {
            value_l_mA = -this->info.TORQUE_LIMIT;
        }
        if (this->info.TORQUE_LIMIT < value_r_mA) {
            value_r_mA = this->info.TORQUE_LIMIT;
        } else if (value_r_mA < -this->info.TORQUE_LIMIT) {
            value_r_mA = -this->info.TORQUE_LIMIT;
        }
        this->info.left.torque_request_mA  = value_l_mA;
        this->info.right.torque_request_mA = value_r_mA;

        return ctrl.set_target_torque(value_l_mA, value_r_mA);
    }
    //////////////////////////////////////
    bool cmd_speed_mode()
    {
        bool result = ctrl.set_control_mode(LibZLAC8015DModbus::MODBUS_DRIVER_MODE::VELOCITY);
        if (true == result) {
            result = ctrl.set_s_shape_acceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_s_shape_deceleration_time(500, 500);
        }
        if (true == result) {
            result = ctrl.set_target_velocity(0, 0);
        }
        if (true == result) {
            result = ctrl.control_word_enable();
        }

        return result;
    }

    bool cmd_speed_set_acc_and_dec(int acceleration_ms, int deceleration_ms, ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        log_v("%s", __func__);
        bool result = true;
        int acc_l   = acceleration_ms;
        int acc_r   = acceleration_ms;
        int dec_l   = deceleration_ms;
        int dec_r   = deceleration_ms;
        if (target != target == ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL) {
            if (true == result) {
                result = ctrl.get_s_shape_acceleration_time(&acc_l, &acc_r);
            }
            if (true == result) {
                result = ctrl.get_s_shape_deceleration_time(&dec_l, &dec_r);
            }
            if (target != ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT) {
                acc_r = acceleration_ms;
                dec_r = deceleration_ms;
            }
            if (target != ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT) {
                acc_l = acceleration_ms;
                dec_l = deceleration_ms;
            }
        }
        if (true == result) {
            result = ctrl.set_s_shape_acceleration_time(acc_l, acc_r);
        }
        if (true == result) {
            result = ctrl.set_s_shape_deceleration_time(dec_l, dec_r);
        }

        return result;
    }
    bool cmd_speed_set(int milli_rpm_l, int milli_rpm_r)
    {
        // TODO: implement
        log_v("%s", __func__);
        bool result_01 = true;
        bool result_02 = true;
        if (true == this->info.left.interval) {
            milli_rpm_l = -milli_rpm_l;
        }
        if (true == this->info.right.interval) {
            milli_rpm_r = -milli_rpm_r;
        }

        if ((true == this->info.flag.emergency) || (false == this->info.flag.heart_beat)) {
            milli_rpm_l = 0;
            milli_rpm_r = 0;
        }
        this->info.left.speed_request_rpm  = milli_rpm_l;
        this->info.right.speed_request_rpm = milli_rpm_r;

        if (this->info.SPEED_LIMIT < (milli_rpm_l / 60)) {
            milli_rpm_l = this->info.SPEED_LIMIT * 60;
        } else if ((milli_rpm_l / 60) < -this->info.SPEED_LIMIT) {
            milli_rpm_l = -this->info.SPEED_LIMIT * 60;
        }
        if (this->info.SPEED_LIMIT < (milli_rpm_r / 60)) {
            milli_rpm_r = this->info.SPEED_LIMIT * 60;
        } else if ((milli_rpm_r / 60) < -this->info.SPEED_LIMIT) {
            milli_rpm_r = -this->info.SPEED_LIMIT * 60;
        }
        this->speed_mps_request.set(this->rpm_to_mps(milli_rpm_l / 60), this->rpm_to_mps(milli_rpm_r / 60), 0, 0);

        return ctrl.set_target_velocity(milli_rpm_l / 1000, milli_rpm_r / 1000);
    }

    bool cmd_speed_heart_beat()
    {
        // TODO: implement
        static MessageFrame _frame;
        static unsigned int TIME_LITTLE_HEART_BEAT_MS  = (10);
        static unsigned long next_little_heart_beat_ms = 0;

        static unsigned int TIME_HEART_BEAT_MS  = (500);
        static unsigned long next_heart_beat_ms = 0;
        static unsigned int TIME_GET_STATE_MS   = (1500);
        static unsigned long next_get_state_ms  = 0;
        bool result                             = false;
        bool result_01                          = false;
        bool result_02                          = false;
        bool flag_output                        = false;
#ifndef DEBUG_TRACE
        flag_output = true;
#endif
        unsigned long current_time = millis();
        if (false == this->_flag_monitoring_speed) {
            result_01 = true;
            result_02 = true;
            if (next_get_state_ms <= current_time) {
                next_get_state_ms = current_time + TIME_GET_STATE_MS;
                this->cmd_get_all_status();
            }
        } else {
            if (true != this->is_connection()) {
                // do nothing
            } else {
                if (next_little_heart_beat_ms <= current_time) {
                    next_little_heart_beat_ms = current_time + TIME_LITTLE_HEART_BEAT_MS;
                    this->cmd_get_position_feedback(ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL);
                }

                if (next_heart_beat_ms <= current_time) {
                    if (false == this->info.flag.running) {
                        if (next_get_state_ms <= current_time) {
                            next_get_state_ms = current_time + TIME_GET_STATE_MS;
                            this->cmd_get_all_status();
                        }
                    } else {
                        char buffer[4]     = { 0x80, 0x00, 0x80 };
                        char receive[100]  = { 0 };
                        int16_t buf        = 0;
                        long temp          = 0;
                        next_heart_beat_ms = current_time + TIME_HEART_BEAT_MS;
                        if (true != this->cmd_get_bus_voltage(ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)) {
                            result_01 = false;
                            result_02 = false;
                        }
                        if (true != this->cmd_get_output_current(ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)) {
                            result_01 = false;
                            result_02 = false;
                        }
                        if (true != this->cmd_get_motor_speed(ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)) {
                            result_01 = false;
                            result_02 = false;
                        }
                        if ((true == result_01) || (true == result_01)) {
                            this->speed_mps_feedback.set(this->rpm_to_mps(this->info.left.speed_rpm), this->rpm_to_mps(this->info.right.speed_rpm), 0, 0);
                            this->info.left.update_time  = current_time;
                            this->info.right.update_time = current_time;
                        }
                    }
                }
            }
        }
        return result_01 && result_02;
    }

#if 0
private:
    bool _confirm(const char *name, ZLAC::TARGET_MOTOR target, char cmd, bool output_log = true)
    {
        // TODO: implement
        bool result      = false;
        char buffer[100] = { 0 };
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            if (2 <= this->_receive(name, this->_serial_driver_1, 2, buffer, output_log)) {
                if (cmd == buffer[0]) {
                    if (cmd == buffer[1]) {
                        result = true;
                    }
                }
            }
        }
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            if (2 <= this->_receive(name, this->_serial_driver_2, 2, buffer, output_log)) {
                if (cmd == buffer[0]) {
                    if (cmd == buffer[1]) {
                        result = true;
                    }
                }
            }
        }
        return result;
    }
    bool _send_target(const char *name, ZLAC::TARGET_MOTOR target, char a1, char a2, char a3, bool confirm, bool output_log = false)
    {
        // TODO: implement
        bool result_01 = true;
        bool result_02 = true;
        bool result    = true;
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            result_01 = this->_send(name, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, this->_serial_driver_1, a1, a2, a3, confirm, output_log);
        }
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            result_02 = this->_send(name, ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT, this->_serial_driver_2, a1, a2, a3, confirm, output_log);
        }
        return result_01 && result_02;
    }
    bool _send(const char *name, ZLAC::TARGET_MOTOR target, HardwareSerial *serial, char a1, char a2, char a3, bool confirm, bool output_log = false)
    {
        // TODO: implement
        //log_v("%s", __func__);
        unsigned int cs = a1 + a2 + a3;
        bool result     = true;
        char buffer[4]  = { a1, a2, a3, (char)(cs & 0xFF) };
        if (true == output_log) {
#if DEBUG_TRACE
#if DEBUG_ZLAC706_SERIAL
            std::string debug_message = "send    :";
            char buf[100];
            for (int i = 0; i < 4; i++) {
                sprintf(buf, " 0x%02X", buffer[i]);
                debug_message.append(buf);
            }
            log_v("%s", debug_message.c_str());
#endif
#endif
        }
        if (nullptr != serial) {
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->_clear_receive(ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT);
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->_clear_receive(ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT);
            }
            serial->write(buffer, 4);
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->info.left.order.set(a1, a2, a3);
            }
            if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
                this->info.right.order.set(a1, a2, a3);
            }
            if (true == confirm) {
                result = this->_confirm(name, target, a1, output_log);
            }
        }
        return result;
    }
    int _receive(const char *name, HardwareSerial *serial, int size, char *buffer, bool output_log = false)
    {
        // TODO: implement
        //log_v("%s", __func__);
        char buf[100];
        int count               = 0;
        bool flag_timeout       = true;
        int index               = 0;
        unsigned long loop_time = 0;
        if (nullptr != serial) {
            loop_time    = millis() + this->TIMEOUT_DRIVER_MS;
            flag_timeout = true;
            index        = 0;
            do {
                count++;
                if (size <= index) {
                    flag_timeout = false;
                    break;
                }
                if (0 != serial->available()) {
                    buffer[index++] = serial->read();
                } else {
                    delay(this->INTERVAL_DRIVER_MS);
                }
            } while (millis() <= loop_time);
        }
        if (true == flag_timeout) {
#if DEBUG_ZLAC
            log_w("Timeout [%d] ms : size [%02d/%02d] :  (%s)", (count - 1) * this->INTERVAL_DRIVER_MS, index, size, name);
#endif
        } else {
            if (true == output_log) {
#if DEBUG_TRACE
#if DEBUG_ZLAC
                std::string debug_message_01 = "receive :";
                sprintf(buf, " count [%d] :", count - 1);
                debug_message_01.append(buf);
                for (int i = 0; i < size; i++) {
                    sprintf(buf, " 0x%02X", buffer[i]);
                    debug_message_01.append(buf);
                }
                log_v("%s", debug_message_01.c_str());
#endif
#endif
            }
        }
        return index;
    }
    bool _receive_wait(HardwareSerial *serial, int size)
    {
        // TODO: implement
        bool result             = false;
        unsigned long loop_time = millis() + this->TIMEOUT_DRIVER_MS;
        do {
            if (size <= serial->available()) {
                result = true;
                break;
            } else {
                delay(this->INTERVAL_DRIVER_MS);
            }
        } while (millis() <= loop_time);
        return result;
    }
    //////////////////////////////////////

    void _clear_receive(ZLAC::TARGET_MOTOR target = ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL)
    {
        // TODO: implement
        char buf[100];
#if DEBUG_TRACE
        bool result_01            = false;
        bool result_02            = false;
        std::string debug_message = "";
#else
        size_t size = 0;
#endif
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            if (nullptr != this->_serial_driver_1) {
#if DEBUG_TRACE
                debug_message = "clear data[1] ->";
                while (0 != this->_serial_driver_1->available()) {
                    sprintf(buf, " 0x%02X", this->_serial_driver_1->read());
                    debug_message.append(buf);
                    result_01 = true;
                }
                if (true == result_01) {
                    log_v("%s", debug_message.c_str());
                }
#else
                do {
                    size = this->_serial_driver_1->available();
                    if (0 == size) {
                        break;
                    } else {
                        (void)this->_serial_driver_1->readBytes(buf, size);
                        delay(this->INTERVAL_DRIVER_MS);
                    }
                } while (0 != size);
#endif
                //this->info.system.set(LOG_CLEAR_RECEIVE, ZLAC::TARGET_MOTOR::TARGET_MOTOR_LEFT, 0);
            }
        }
        if ((ZLAC::TARGET_MOTOR::TARGET_MOTOR_RIGHT == target) || (ZLAC::TARGET_MOTOR::TARGET_MOTOR_ALL == target)) {
            if (nullptr != this->_serial_driver_2) {
#if DEBUG_TRACE
                debug_message = "clear data[2] ->";
                while (0 != this->_serial_driver_2->available()) {
                    sprintf(buf, " 0x%02X", this->_serial_driver_2->read());
                    debug_message.append(buf);
                    result_02 = true;
                }
                if (true == result_02) {
                    log_v("%s", debug_message.c_str());
                }
#else
                do {
                    size = this->_serial_driver_2->available();
                    if (0 == size) {
                        break;
                    } else {
                        (void)this->_serial_driver_2->readBytes(buf, size);
                        delay(this->INTERVAL_DRIVER_MS);
                    }
                } while (0 != size);

#endif
                //this->info.system.set(LOG_CLEAR_RECEIVE, TARGET_MOTOR::TARGET_MOTOR_RIGHT, 0);
            }
        }
    }
#endif

private:
    HardwareSerial *_serial_driver = nullptr;
    bool _flag_invert              = SETTING_FLAG_INVERT;

    bool _flag_monitoring_speed = false;

private:
};

#endif
