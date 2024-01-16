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

#include <Adafruit_PWMServoDriver.h>
#include <SPIFFS.h>

#ifndef SERVO_ADDRESS
#define SERVO_ADDRESS PCA9685_I2C_ADDRESS
#endif
#ifndef NECK_CONTROLLER_SETTING_FILE_NAME
#define NECK_CONTROLLER_SETTING_FILE_NAME "/setting.ini"
#endif

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
    NeckController(HardwareSerial *serial, int pwm_size = 3, int pwm_address = PCA9685_I2C_ADDRESS);
    ~NeckController();

public:
    vector_t accel;
    vector_t gyro;
    int *pwm_servo;
    int pwm_servo_count;
    int *pwm_servo_request;
    unsigned int count_reception = 0;
    unsigned int count_process   = 0;

public:
    void set_accel(float x, float y, float z);
    void set_gyro(float x, float y, float z);
    bool set_pwm_servo(int index, int value);
    uint32_t get_oscillator_frequency(void);
    uint32_t get_pwm_freq(void);

protected:
    MessageFrame _reception(MessageFrame frame) override;
    bool _init() override;

private:
    bool _read_multiple_holding_registers(MessageFrame &frame);

    bool _write_single_register(MessageFrame &frame);
    bool _write_multiple_registers(MessageFrame &frame);

    int _response_accel_and_gyro(MessageFrame &frame, unsigned int sub_address, int length, int start_index = 2);
    int _response_pwm_servo_request(MessageFrame &frame, unsigned int sub_address, int length, int start_index = 2);
    int _response_pwm_servo_actual(MessageFrame &frame, unsigned int sub_address, int length, int start_index = 2);
    int _response_pwm_servo_setting(MessageFrame &frame, unsigned int sub_address, int length, int start_index = 2);

    bool _load_setting_setting();
    bool _save_setting_setting();
    bool _load_setting_setting(fs::FS &fs);
    bool _save_setting_setting(fs::FS &fs);

private:
    Queue<MessageFrame::EXCEPTION_CODE> _exception_queue;
    Adafruit_PWMServoDriver *_pwm;
    uint32_t _oscillator_frequency = (2.7 * 1000 * 1000);
    uint32_t _pwm_freq             = (1.6 * 1000);

private:
    const char *_setting_file_name = NECK_CONTROLLER_SETTING_FILE_NAME;
};

#endif
