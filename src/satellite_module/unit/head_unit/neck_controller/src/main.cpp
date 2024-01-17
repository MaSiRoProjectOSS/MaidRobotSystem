/**
 * @file main.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-10
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "neck_controller.hpp"

#include <M5AtomS3.h>

#define APP_NAME    "Neck controller"
#define APP_VERSION "0.23.12"
#define LOOP_SPAN   10
#ifndef CUSTOM_MODBUS_TYPE
#define CUSTOM_MODBUS_TYPE MessageFrame::MODBUS_TYPE::MODBUS_TYPE_ASCII
#endif

class HardwareSerialEX : public HardwareSerial {
public:
    HardwareSerialEX(uint8_t uart_nr) : HardwareSerial(uart_nr)
    {
    }
    UBaseType_t get_stack_high_water_mark()
    {
        return (ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE - uxTaskGetStackHighWaterMark(this->_eventTask));
    }
    UBaseType_t get_stack_size()
    {
        return ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE;
    }
};

HardwareSerialEX SerialEX(0);
NeckController modbus(&SerialEX);
void display_error(void)
{
    M5.Lcd.clear((uint16_t)BLACK);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setRotation(2);
    M5.Lcd.printf("%s\r\n version: %s\r\n\r\n", APP_NAME, APP_VERSION);
    M5.Lcd.printf("modbus begin failed\r\n");
}

void setup(void)
{
    m5::M5Unified::config_t cfg = M5.config();
    M5.begin(cfg);
    delay(1000);
    bool result = modbus.begin(MODBUS_ADDRESS, (MessageFrame::MODBUS_TYPE)CUSTOM_MODBUS_TYPE);
    if (false == result) {
        int count_down = (1000 * 10) / LOOP_SPAN;
        bool flag_wake = true;
        display_error();
        while (false == result) {
            M5.update();
            if (M5.BtnA.wasPressed()) {
                count_down = (1000 * 10) / LOOP_SPAN;
                M5.Lcd.wakeup();
                display_error();
                flag_wake = true;
            }
            if (true == flag_wake) {
                if (0 < count_down) {
                    count_down--;
                    if (0 == count_down) {
                        count_down = 0;
                        M5.Lcd.clear();
                        M5.Lcd.sleep();
                        flag_wake = false;
                    }
                }
            }
            delay(LOOP_SPAN);
        }
    }
    M5.Lcd.sleep();
}

void update_imu(void)
{
    static int count = 0;

    m5::IMU_Class::sensor_mask_t imu_update = M5.Imu.update();
    if (m5::IMU_Class::sensor_mask_t::sensor_mask_none < imu_update) {
        m5::IMU_Class::imu_data_t data = M5.Imu.getImuData();
        modbus.set_accel(data.accel.x, data.accel.y, data.accel.z);
        modbus.set_gyro(data.gyro.x, data.gyro.y, data.gyro.z);
    }
}

void loop(void)
{
    static int count      = 0;
    static int count_down = (1000 * 10) / LOOP_SPAN;
    static bool flag_wake = true;

    M5.update();

    count++;
    if ((250 / LOOP_SPAN) < count) {
        update_imu();
        count = 0;
    }
    if (M5.BtnA.wasPressed()) {
        count_down = (1000 * 10) / LOOP_SPAN;
        M5.Lcd.wakeup();
        flag_wake = true;
    }
    if (true == flag_wake) {
        if (0 < count_down) {
            count_down--;
            if (0 == (count_down % 10)) {
                UBaseType_t stack_cushy = SerialEX.get_stack_high_water_mark();
                UBaseType_t max_cushy   = SerialEX.get_stack_size();

                M5.Lcd.setCursor(0, 0);
                M5.Lcd.setRotation(2);
                M5.Lcd.printf("%s\r\n version: %s\r\n\r\n", APP_NAME, APP_VERSION);
                M5.Lcd.printf("Accel:\r\n %6d,%6d,%6d\r\n", (int)modbus.accel.x, (int)modbus.accel.y, (int)modbus.accel.z);
                M5.Lcd.printf("Gyro:\r\n %6d,%6d,%6d\r\n", (int)(modbus.gyro.x), (int)(modbus.gyro.y), (int)(modbus.gyro.z));

                M5.Lcd.printf("\r\nCount [%6d/%6d]\r\n", modbus.count_process, modbus.count_reception);
                M5.Lcd.printf("STACK [%6d/%6d]\r\n", (int)(stack_cushy), (int)max_cushy);
                M5.Lcd.printf("OSC-FREQ [%10d]\r\n", modbus.get_oscillator_frequency());
                M5.Lcd.printf("PWM-FREQ [%10d]\r\n", modbus.get_pwm_freq());
            }
            if (0 == count_down) {
                count_down = 0;
                M5.Lcd.clear();
                M5.Lcd.sleep();
                flag_wake = false;
            }
        }
    }
    delay(LOOP_SPAN);
}
