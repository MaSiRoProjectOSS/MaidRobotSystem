/**
 * @file test_coordinate_euler.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-22
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 * @ref https://github.com/ThrowTheSwitch/Unity
 *
 */
#include "maid_robot_system/common/types/coordinate_euler.hpp"

#include <unity.h>

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_Construct_00(void)
{
    CoordinateEuler ins;
    TEST_ASSERT_EQUAL(0, ins.pitch);
    TEST_ASSERT_EQUAL(0, ins.roll);
    TEST_ASSERT_EQUAL(0, ins.yaw);
}

void test_Construct_01(void)
{
    CoordinateEuler ins(12, 34, 56);
    TEST_ASSERT_EQUAL(12, ins.pitch);
    TEST_ASSERT_EQUAL(34, ins.roll);
    TEST_ASSERT_EQUAL(56, ins.yaw);
}

void test_set(void)
{
    CoordinateEuler ins(12, 34, 56);
    ins.set(78, 90, 123);
    TEST_ASSERT_EQUAL(78, ins.pitch);
    TEST_ASSERT_EQUAL(90, ins.roll);
    TEST_ASSERT_EQUAL(123, ins.yaw);
}
void test_variable(void)
{
    CoordinateEuler ins(12, 34, 56);
    ins.pitch = 456;
    ins.roll  = 789;
    ins.yaw   = 1023;
    TEST_ASSERT_EQUAL(456, ins.pitch);
    TEST_ASSERT_EQUAL(789, ins.roll);
    TEST_ASSERT_EQUAL(1023, ins.yaw);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_Construct_00);
    RUN_TEST(test_Construct_01);
    RUN_TEST(test_set);
    RUN_TEST(test_variable);
    UNITY_END();
    return 0;
}
