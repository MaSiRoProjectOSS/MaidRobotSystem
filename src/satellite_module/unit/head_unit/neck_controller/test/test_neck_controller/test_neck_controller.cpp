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

//////////////////////////////////////////////////////////////////////////////////////

void test_neck_controller_set_accel()
{
    NeckController neck(Serial);

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
    NeckController neck(Serial);

    neck.set_gyro(1, 2, 3);
    TEST_ASSERT_EQUAL(1000, neck.gyro.x);
    TEST_ASSERT_EQUAL(2000, neck.gyro.y);
    TEST_ASSERT_EQUAL(3000, neck.gyro.z);

    neck.set_gyro(0.001, 0.002, 0.003);
    TEST_ASSERT_EQUAL(1, neck.gyro.x);
    TEST_ASSERT_EQUAL(2, neck.gyro.y);
    TEST_ASSERT_EQUAL(3, neck.gyro.z);
}
void test_neck_controller_reception_illegal()
{
    NeckController neck(Serial);
    MessageFrame frame;
    frame.function = 0x01;

    MessageFrame response = neck.reception(frame);
    TEST_ASSERT_EQUAL(0x81, response.function);

    response = neck.reception(frame);
    TEST_ASSERT_EQUAL(0x81, response.function);
    TEST_ASSERT_EQUAL(1, response.data_length);
    TEST_ASSERT_EQUAL(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION, response.data[0]);
}
void test_neck_controller_reception_read_exception_status()
{
    NeckController neck(Serial);
    MessageFrame frame;
    frame.function = 0x01;

    MessageFrame response = neck.reception(frame);
    response              = neck.reception(frame);

    frame.function = 0x07;
    response       = neck.reception(frame);

    TEST_ASSERT_EQUAL(2, response.data_length);
    for (int i = 0; i < response.data_length; i++) {
        //printf("%d", response.data[i]);
    }
    TEST_ASSERT_EQUAL(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION, response.data[0]);
    TEST_ASSERT_EQUAL(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION, response.data[1]);
}
void test_neck_controller_reception_read_holding_registers()
{
    NeckController neck(Serial);
    MessageFrame frame;
    MessageFrame response;

    neck.set_gyro(0.001, 0.002, 0.003);
    neck.set_accel(4, 5, 6);

    frame.function = 0x03;
    frame.data[0]  = 0x68;
    frame.data[1]  = 0x10;
    frame.data[2]  = 0x00;
    frame.data[3]  = 0x01;
    response       = neck.reception(frame);
    TEST_ASSERT_EQUAL(3, response.data_length);

    frame.data[0] = 0x70;
    response      = neck.reception(frame);
    TEST_ASSERT_EQUAL(1, response.data_length);
}

//////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_neck_controller_set_accel);
    RUN_TEST(test_neck_controller_set_gyro);
    RUN_TEST(test_neck_controller_reception_read_holding_registers);
    RUN_TEST(test_neck_controller_reception_illegal);
    RUN_TEST(test_neck_controller_reception_read_exception_status);
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
