/**
 * @file test_neck_controller.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-11
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "neck_controller.hpp"

#include <unity.h>
#define TEST_GROUP_SETTER    1
#define TEST_GROUP_RECEPTION 1

//////////////////////////////////////////////////////////////////////////////////////
#if TEST_GROUP_SETTER
void test_neck_controller_set_accel()
{
    NeckController neck(&Serial);

    neck.set_accel(1, 2, 3);
    TEST_ASSERT_EQUAL(1000, neck.accel.x);
    TEST_ASSERT_EQUAL(2000, neck.accel.y);
    TEST_ASSERT_EQUAL(3000, neck.accel.z);

    neck.set_accel(0.001, 0.002, 0.003);
    TEST_ASSERT_EQUAL(1, neck.accel.x);
    TEST_ASSERT_EQUAL(2, neck.accel.y);
    TEST_ASSERT_EQUAL(3, neck.accel.z);
}
void test_neck_controller_set_gyro()
{
    NeckController neck(&Serial);

    neck.set_gyro(1, 2, 3);
    TEST_ASSERT_EQUAL(1000, neck.gyro.x);
    TEST_ASSERT_EQUAL(2000, neck.gyro.y);
    TEST_ASSERT_EQUAL(3000, neck.gyro.z);

    neck.set_gyro(0.001, 0.002, 0.003);
    TEST_ASSERT_EQUAL(1, neck.gyro.x);
    TEST_ASSERT_EQUAL(2, neck.gyro.y);
    TEST_ASSERT_EQUAL(3, neck.gyro.z);
}
#endif
//////////////////////////////////////////////////////////////////////////////////////
#if TEST_GROUP_RECEPTION

void test_neck_controller_reception_0xXX_illegal()
{
    NeckController neck(&Serial);
    MessageFrame frame;
    frame.function = 0x01;

    MessageFrame response = neck.pub_reception(frame);
    TEST_ASSERT_EQUAL(0x81, response.function);

    response = neck.pub_reception(frame);
    TEST_ASSERT_EQUAL(0x81, response.function);
    TEST_ASSERT_EQUAL(1, response.data_length);
    TEST_ASSERT_EQUAL(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION, response.data[0]);
}
void test_neck_controller_reception_0x07_read_exception_status()
{
    NeckController neck(&Serial);
    MessageFrame frame;
    frame.function = 0x01;

    MessageFrame response = neck.pub_reception(frame);
    TEST_ASSERT_EQUAL(0x81, response.function);
    response = neck.pub_reception(frame);
    TEST_ASSERT_EQUAL(0x81, response.function);

    frame.function = 0x07;
    response       = neck.pub_reception(frame);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
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
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x10_size_1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6810;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2, response.data_length);
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
}
void test_neck_controller_reception_0x03_read_holding_registers_imu_sub_0x11_size_1()
{
    unsigned int function  = 0x03;
    unsigned int address   = 0x6811;
    unsigned int data_size = 0x01;
    int response_data      = 0;
    int data_length        = 0;

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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
//
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size0()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size1()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size2()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size3()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size4()
{
#if 0
    unsigned int function  = 0x03;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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
#endif
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x00_size5()
{
}
//
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size0()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size1()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size2()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size3()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size4()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x01_size5()
{
}

//
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size0()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size1()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size2()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size3()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size4()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x02_size5()
{
}

//
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size0()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size1()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size2()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size3()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size4()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x03_size5()
{
}
//
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size0()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size1()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size2()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size3()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size4()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x70_setting_0x04_size5()
{
}

//////////////////////////////////////////////////////////////////////////////////////
// PWM Motor
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size0()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size1()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size2()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size3()
{
#if 0
    unsigned int function  = 0x03;
    unsigned int address   = 0x7100;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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
#endif
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x00_size4()
{
}

void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size0()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size1()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size2()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size3()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x01_size4()
{
}

void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size0()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size1()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size2()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size3()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x02_size4()
{
}

void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size0()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size1()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size2()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size3()
{
}
void test_neck_controller_reception_0x03_read_holding_registers_pwm_0x71_sub_0x03_size4()
{
}

//////////////////////////////////////////////////////////////////////////////////////
void test_neck_controller_reception_0x06_write_single_register_0x7000()
{
    unsigned int function  = 0x06;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

    // response_data
    response_data = ((response.data[2] << 8) & 0xFF00) | (response.data[3] & 0xFF);
    response_data = ((response_data << 16) & 0xFFFF0000) | ((response.data[4] << 8) & 0xFF00) | (response.data[5] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    response_data = ((response.data[6] << 8) & 0xFF00) | (response.data[7] & 0xFF);
    response_data = ((response_data << 16) & 0xFFFF0000) | ((response.data[8] << 8) & 0xFF00) | (response.data[9] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);

    response_data = ((response.data[10] << 8) & 0xFF00) | (response.data[11] & 0xFF);
    response_data = ((response_data << 16) & 0xFFFF0000) | ((response.data[12] << 8) & 0xFF00) | (response.data[13] & 0xFF);
    TEST_ASSERT_EQUAL(0, response_data);
}

void test_neck_controller_reception_0x06_write_single_register_0x7002()
{
}

//////////////////////////////////////////////////////////////////////////////////////
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size1()
{
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size2()
{
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size3()
{
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size4()
{
    unsigned int function  = 0x10;
    unsigned int address   = 0x7000;
    unsigned int data_size = 0x04;
    int response_data      = 0;
    int data_length        = 0;

    NeckController neck(&Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_accel(4, 5, 6);
    neck.set_gyro(0.001, 0.002, 0.003);

    frame.function = function;
    frame.data[0]  = (address >> 8) & 0xFF;
    frame.data[1]  = address & 0xFF;
    frame.data[2]  = (data_size >> 8) & 0xFF;
    frame.data[3]  = data_size & 0xFF;
    response       = neck.pub_reception(frame);

    // data_length
    TEST_ASSERT_EQUAL(2 + data_size * 2, response.data_length);
    // size
    data_length = ((response.data[0] << 8) & 0xFF00) | (response.data[1] & 0xFF);
    TEST_ASSERT_EQUAL(2 + data_size * 2, data_length);

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
void test_neck_controller_reception_0x10_write_multiple_registers_0x7000_size5()
{
}

void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size1()
{
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size2()
{
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size3()
{
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size4()
{
}
void test_neck_controller_reception_0x10_write_multiple_registers_0x7002_size5()
{
}

#endif

//////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    UNITY_BEGIN();
#if TEST_GROUP_SETTER
    RUN_TEST(test_neck_controller_set_accel);
    RUN_TEST(test_neck_controller_set_gyro);
#endif
#if TEST_GROUP_RECEPTION
    RUN_TEST(test_neck_controller_reception_0xXX_illegal);
    RUN_TEST(test_neck_controller_reception_0x07_read_exception_status);

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
    //
    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7000);
    RUN_TEST(test_neck_controller_reception_0x06_write_single_register_0x7002);

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
/*
    void set_accel(float x, float y, float z);
    void set_gyro(float x, float y, float z);
    bool set_pwm_servo(int index, int value);
    uint32_t get_oscillator_frequency(void);
    uint32_t get_pwm_freq(void);

*/
