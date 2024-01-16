/**
 * @file test_modbus_controller.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-11
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "neck_controller.hpp"

#include <unity.h>
#define TEST_GROUP_SETTER                      (1)
#define TEST_GROUP_RECEPTION_EXCEPTION         (1)
#define TEST_GROUP_RECEPTION_PWM_MOTOR_VALUE   (1)
#define TEST_GROUP_RECEPTION_PWM_MOTOR_SETTING (1)
#define TEST_GROUP_RECEPTION_IMU               (1)

void debug_print(MessageFrame frame)
{
    printf("-------------\r\n");
    printf("  * function : %02X\r\n", frame.function);
    printf("  * length   : %02X\r\n", frame.data_length);
    printf("  * data     : ");
    for (int i = 0; i < frame.data_length; i++) {
        printf("%02X ", frame.data[i]);
    }
    printf("\r\n");
}

//////////////////////////////////////////////////////////////////////////////////////
void test_neck_controller_set_accel()
{
    NeckController impl(&Serial);

    impl.set_accel(1, 2, 3);
    TEST_ASSERT_EQUAL(1000, impl.accel.x);
    TEST_ASSERT_EQUAL(2000, impl.accel.y);
    TEST_ASSERT_EQUAL(3000, impl.accel.z);

    impl.set_accel(0.001, 0.002, 0.003);
    TEST_ASSERT_EQUAL(1, impl.accel.x);
    TEST_ASSERT_EQUAL(2, impl.accel.y);
    TEST_ASSERT_EQUAL(3, impl.accel.z);
}
void test_neck_controller_set_gyro()
{
    NeckController impl(&Serial);

    impl.set_gyro(1, 2, 3);
    TEST_ASSERT_EQUAL(1000, impl.gyro.x);
    TEST_ASSERT_EQUAL(2000, impl.gyro.y);
    TEST_ASSERT_EQUAL(3000, impl.gyro.z);

    impl.set_gyro(0.001, 0.002, 0.003);
    TEST_ASSERT_EQUAL(1, impl.gyro.x);
    TEST_ASSERT_EQUAL(2, impl.gyro.y);
    TEST_ASSERT_EQUAL(3, impl.gyro.z);
}

//////////////////////////////////////////////////////////////////////////////////////

void test_neck_controller_reception_0xXX_illegal()
{
    NeckController impl(&Serial);
    MessageFrame frame;
    frame.function    = 0x01;
    int data_length   = 0;
    frame.data_length = data_length;

    MessageFrame response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);
    TEST_ASSERT_EQUAL(0x81, response.function);

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);
    TEST_ASSERT_EQUAL(0x81, response.function);
    TEST_ASSERT_EQUAL(1, response.data_length);
    TEST_ASSERT_EQUAL(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION, response.data[0]);
}
void test_neck_controller_reception_0x07_read_exception_status()
{
    NeckController impl(&Serial);
    MessageFrame frame;
    frame.function    = 0x01;
    int data_length   = 0;
    frame.data_length = data_length;

    MessageFrame response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);
    TEST_ASSERT_EQUAL(0x81, response.function);
    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);
    TEST_ASSERT_EQUAL(0x81, response.function);

    frame.function = 0x07;
    response       = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    TEST_ASSERT_EQUAL(2, response.data_length);
    TEST_ASSERT_EQUAL(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION, response.data[0]);
    TEST_ASSERT_EQUAL(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION, response.data[1]);
    TEST_ASSERT_EQUAL(0x00, response.data[2]);
}

// sub_0x10
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6810;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6810;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(4000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6810;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(4000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6810;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(4000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6810;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(4000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6810;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(4000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_6()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6810;
    unsigned int data_size = 0x06;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(14, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(6, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(4000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_7()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6810;
    unsigned int data_size = 0x07;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(16, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(7, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(4000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}

// sub_0x11
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6811;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6811;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6811;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6811;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6811;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6811;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_6()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6811;
    unsigned int data_size = 0x06;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(14, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(6, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_7()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6811;
    unsigned int data_size = 0x07;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(16, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(7, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(5000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}

// sub_0x12
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6812;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6812;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6812;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6812;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6812;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6812;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_6()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6812;
    unsigned int data_size = 0x06;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(14, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(6, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_7()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6812;
    unsigned int data_size = 0x07;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(16, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(7, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(6000, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}

// sub_0x13
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6813;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6813;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6813;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6813;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6813;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6813;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_6()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6813;
    unsigned int data_size = 0x06;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(14, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(6, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_7()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6813;
    unsigned int data_size = 0x07;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(16, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(7, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(1, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}

// sub_0x14
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6814;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6814;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6814;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6814;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6814;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6814;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_6()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6814;
    unsigned int data_size = 0x06;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(14, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(6, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_7()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6814;
    unsigned int data_size = 0x07;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(16, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(7, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(2, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}

// sub_0x15
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6815;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6815;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6815;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6815;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6815;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6815;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_6()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6815;
    unsigned int data_size = 0x06;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(14, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(6, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_7()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6815;
    unsigned int data_size = 0x07;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(16, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(7, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(3, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}

//////////////////////////////////////////////////////////////////////////////////////
// pwm setting
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency & 0xFFFF, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency & 0xFFFF, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency & 0xFFFF, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency & 0xFFFF, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
//
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7001;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7001;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7001;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7001;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7001;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7001;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(oscillator_frequency & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}

//
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq >> 16, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}

//
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7003;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7003;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7003;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7003;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7003;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7003;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(pwm_freq & 0xFFFF, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
//
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7004;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7004;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7004;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7004;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7004;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size5()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7004;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(12, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(5, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}

//////////////////////////////////////////////////////////////////////////////////////
// PWM Motor
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7100;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7100;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1234, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7100;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1234, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7100;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1234, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7100;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1234, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}

void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7101;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7101;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1234, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7101;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1234, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7101;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1234, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7101;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1234, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}

void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7102;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7102;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7102;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7102;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7102;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1235, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}

void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7103;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7103;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7103;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7103;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7103;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0x1236, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}

void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size0()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7104;
    unsigned int data_size = 0x00;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7104;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(4, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size2()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7104;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(6, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size3()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7104;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(8, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size4()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x7104;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(10, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}

//////////////////////////////////////////////////////////////////////////////////////
void test_neck_controller_reception_0x06_write_single_register_0x7000()
{
    unsigned int function = 0x06;
    unsigned int address  = 0x7000;
    unsigned int data     = 0x32FA;
    int response_data     = 0;
    int data_length       = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t oscillator_frequency = impl.get_oscillator_frequency();

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data >> 8) & 0xFF;
    frame.data[data_length++] = data & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    oscillator_frequency = ((data << 16) & 0xFFFF0000) | (oscillator_frequency & 0xFFFF);
    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
}
void test_neck_controller_reception_0x06_write_single_register_0x7001()
{
    unsigned int function = 0x06;
    unsigned int address  = 0x7001;
    unsigned int data     = 0x32FA;
    int response_data     = 0;
    int data_length       = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data >> 8) & 0xFF;
    frame.data[data_length++] = data & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    oscillator_frequency = (oscillator_frequency & 0xFFFF0000) | (data & 0xFFFF);
    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
}
void test_neck_controller_reception_0x06_write_single_register_0x7002()
{
    unsigned int function = 0x06;
    unsigned int address  = 0x7002;
    unsigned int data     = 0x32FA;
    int response_data     = 0;
    int data_length       = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq = impl.get_pwm_freq();

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data >> 8) & 0xFF;
    frame.data[data_length++] = data & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    pwm_freq = ((data << 16) & 0xFFFF0000) | (pwm_freq & 0xFFFF);
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
}
void test_neck_controller_reception_0x06_write_single_register_0x7003()
{
    unsigned int function = 0x06;
    unsigned int address  = 0x7003;
    unsigned int data     = 0x32FA;
    int response_data     = 0;
    int data_length       = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq = impl.get_pwm_freq();
    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data >> 8) & 0xFF;
    frame.data[data_length++] = data & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    pwm_freq = (pwm_freq & 0xFFFF0000) | (data & 0xFFFF);
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
}

void test_neck_controller_reception_0x06_write_single_register_0x7100()
{
    unsigned int function = 0x06;
    unsigned int address  = 0x7100;
    unsigned int data     = 0x32FA;
    int response_data     = 0;
    int data_length       = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data >> 8) & 0xFF;
    frame.data[data_length++] = data & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(data, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(data, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(data, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x06_write_single_register_0x7101()
{
    unsigned int function = 0x06;
    unsigned int address  = 0x7101;
    unsigned int data     = 0x32FA;
    int response_data     = 0;
    int data_length       = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data >> 8) & 0xFF;
    frame.data[data_length++] = data & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(data, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x06_write_single_register_0x7102()
{
    unsigned int function = 0x06;
    unsigned int address  = 0x7102;
    unsigned int data     = 0x32FA;
    int response_data     = 0;
    int data_length       = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data >> 8) & 0xFF;
    frame.data[data_length++] = data & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(data, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x06_write_single_register_0x7103()
{
    unsigned int function = 0x06;
    unsigned int address  = 0x7103;
    unsigned int data     = 0x32FA;
    int response_data     = 0;
    int data_length       = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data >> 8) & 0xFF;
    frame.data[data_length++] = data & 0xFF;
    frame.data_length         = data_length;
    response                  = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
    response_data = ((response.data[14] << 8) & 0xFF00) | (response.data[15] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(data, impl.pwm_servo_request[2]);
}

//////////////////////////////////////////////////////////////////////////////////////
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size1()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data

    // get value
    TEST_ASSERT_EQUAL((request_data & 0xFFFF0000) | (oscillator_frequency & 0xFFFF), impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size2()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data

    // get value
    TEST_ASSERT_EQUAL(request_data, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(pwm_freq, impl.get_pwm_freq());
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size3()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data

    // get value
    TEST_ASSERT_EQUAL(request_data, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL((request_data & 0xFFFF0000) | (pwm_freq & 0xFFFF), impl.get_pwm_freq());
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size4()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data

    // get value
    TEST_ASSERT_EQUAL(request_data, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(request_data, impl.get_pwm_freq());
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size5()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(4, data_length);

    // response_data

    // get value
    TEST_ASSERT_EQUAL(request_data, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(request_data, impl.get_pwm_freq());
}

void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size1()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data

    // get value
    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    request_data = (request_data & 0xFFFF0000);
    TEST_ASSERT_EQUAL(request_data, impl.get_pwm_freq());
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size2()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x02;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data

    // get value
    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(request_data, impl.get_pwm_freq());
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size3()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x03;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data

    // get value
    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(request_data, impl.get_pwm_freq());
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size4()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data

    // get value
    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(request_data, impl.get_pwm_freq());
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size5()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7002;
    unsigned int data_size = 0x05;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2, data_length);

    // response_data
    // get value
    TEST_ASSERT_EQUAL(oscillator_frequency, impl.get_oscillator_frequency());
    TEST_ASSERT_EQUAL(request_data, impl.get_pwm_freq());
}

//////////////////////////////////////////////////////////////////////////////////////
void test_neck_controller_reception_0x10_write_multiple_registers_0x7100()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7100;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(3, data_length);

    // response_data
    TEST_ASSERT_EQUAL(0xEDCB, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0xEDCB, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0xEDCB, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7101()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7101;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    TEST_ASSERT_EQUAL(0xEDCB, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7102()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7102;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0xEDCB, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7103()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7103;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 24) & 0xFF;
    frame.data[data_length++] = (request_data >> 16) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(1, data_length);

    // response_data
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0xEDCB, impl.pwm_servo_request[2]);
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7104()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7004;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController impl(&Serial);
    MessageFrame frame;
    MessageFrame response;

    uint32_t pwm_freq             = impl.get_pwm_freq();
    uint32_t oscillator_frequency = impl.get_oscillator_frequency();
    uint32_t request_data         = 0xEDCBA987;

    impl.set_accel(4, 5, 6);
    impl.set_gyro(0.001, 0.002, 0.003);
    for (int i = 0; i < 10; i++) {
        impl.set_pwm_servo(i, 0x1234 + i);
    }

    frame.function            = function;
    frame.data[data_length++] = (address >> 8) & 0xFF;
    frame.data[data_length++] = address & 0xFF;
    frame.data[data_length++] = (data_size >> 8) & 0xFF;
    frame.data[data_length++] = data_size & 0xFF;

    frame.data[data_length++] = (request_data >> 8) & 0xFF;
    frame.data[data_length++] = (request_data >> 0) & 0xFF;
    frame.data_length         = data_length;

    response = impl.pub_reception(frame);
    debug_print(frame);
    debug_print(response);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(0, data_length);

    // response_data
    TEST_ASSERT_EQUAL(0x1234, impl.pwm_servo_request[0]);
    TEST_ASSERT_EQUAL(0x1235, impl.pwm_servo_request[1]);
    TEST_ASSERT_EQUAL(0x1236, impl.pwm_servo_request[2]);
}

//////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    UNITY_BEGIN();
#if TEST_GROUP_SETTER
    RUN_TEST(test_neck_controller_set_accel);
    RUN_TEST(test_neck_controller_set_gyro);
#endif

#if TEST_GROUP_RECEPTION_EXCEPTION
    RUN_TEST(test_neck_controller_reception_0xXX_illegal);
    RUN_TEST(test_neck_controller_reception_0x07_read_exception_status);
#endif
#if TEST_GROUP_RECEPTION_PWM_MOTOR_VALUE
    // PWM Motor
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size4);

    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size4);

    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size4);

    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size4);

    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x04_size4);

    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7100);
    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7101);
    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7102);
    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7103);

    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7100);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7101);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7102);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7103);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7104);
#endif
#if TEST_GROUP_RECEPTION_PWM_MOTOR_SETTING
    // pwm_setting
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size5);

    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size5);

    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size5);

    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size5);

    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size5);

    //
    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7000);
    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7001);
    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7002);
    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7003);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size1);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size2);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size3);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size4);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size5);

    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size1);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size2);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size3);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size4);
    RUN_TEST(test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size5);

#endif
#if TEST_GROUP_RECEPTION_IMU
    // sub_0x10
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_5);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_6);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_7);

    // sub_0x11
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_5);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_6);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_7);

    // sub_0x12
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_5);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_6);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x12_size_7);

    // sub_0x13
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_5);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_6);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x13_size_7);

    // sub_0x14
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_5);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_6);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x14_size_7);

    // sub_0x15
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_0);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_1);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_2);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_3);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_4);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_5);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_6);
    RUN_TEST(test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x15_size_7);
#endif

    UNITY_END();
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////
void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}
