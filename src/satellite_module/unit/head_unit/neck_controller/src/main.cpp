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

#define NECK_CONTROLLER_VERSION "0.23.12"
#define LOOP_SPAN               10

NeckController modbus(&Serial1);
#ifndef CUSTOM_MODBUS_TYPE
#define CUSTOM_MODBUS_TYPE MessageFrame::MODBUS_TYPE::MODBUS_TYPE_ASCII
#endif

void setup(void)
{
    m5::M5Unified::config_t cfg = M5.config();
    M5.begin(cfg);
    delay(1000);
    bool result = modbus.begin(MODBUS_ADDRESS, (MessageFrame::MODBUS_TYPE)CUSTOM_MODBUS_TYPE);
    if (false == result) {
        M5.Lcd.clear((uint16_t)BLACK);
        M5.Lcd.setTextColor(RED, BLACK);
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.printf("modbus begin failed\r\n");
        while (false == result) {
            delay(1000);
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
    static int count_down = 0;

    M5.update();

    count++;
    if ((250 / LOOP_SPAN) < count) {
        update_imu();
        count = 0;
    }
    if (M5.BtnA.wasPressed()) {
        count_down = (1000 * 10) / LOOP_SPAN;
        M5.Lcd.wakeup();
    }

    if (0 < count_down) {
        count_down--;
        if (0 == (count_down % 10)) {
            M5.Lcd.setCursor(0, 0);
            M5.Lcd.printf("Neck controller\r\n version: %s\r\n\r\n", NECK_CONTROLLER_VERSION);
            M5.Lcd.printf("accel:\r\n %6d  \r\n %6d  \r\n %6d  \r\n", (int)modbus.accel.x, (int)modbus.accel.y, (int)modbus.accel.z);
            M5.Lcd.printf("gyro:\r\n %6d  \r\n %6d  \r\n %6d  \r\n", (int)(modbus.gyro.x), (int)(modbus.gyro.y), (int)(modbus.gyro.z));
        }
        if (0 == count_down) {
            count_down = 0;
            M5.Lcd.clear();
            M5.Lcd.sleep();
        }
    }

    delay(LOOP_SPAN);
}
