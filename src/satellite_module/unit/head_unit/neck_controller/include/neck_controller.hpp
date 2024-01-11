/**
 * @file neck_controller.hpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-06
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef MODBUS_IMPL_HPP
#define MODBUS_IMPL_HPP

#include "modbus_lib_arduino.hpp"
#include "queue.hpp"

class NeckController : public ModbusLibArduino {
public:
    struct vector_t {
        union {
            int16_t value[3];
            struct {
                int16_t x;
                int16_t y;
                int16_t z;
            };
        };
    };

public:
    NeckController(HardwareSerial *serial, int pwm_size = 3);
    ~NeckController();

public:
    vector_t accel;
    vector_t gyro;
    int *pwm_servo;
    int pwm_servo_count;
    int *pwm_servo_request;

public:
    void set_accel(float x, float y, float z);
    void set_gyro(float x, float y, float z);
    bool set_pwm_servo(int index, int value);

protected:
    MessageFrame reception(MessageFrame frame) override;

private:
    void read_holding_registers(MessageFrame &frame);
    void write_single_register(MessageFrame &frame);
    void write_multiple_registers(MessageFrame &frame);

private:
    int _response_accel_and_gyro(MessageFrame &frame, unsigned int sub_address, unsigned int length, int start_index = 1);
    int _response_pwm_servo(MessageFrame &frame, unsigned int sub_address, unsigned int length, int start_index = 1);

private:
    Queue<MessageFrame::EXCEPTION_CODE> _exception_queue;
};

#endif
