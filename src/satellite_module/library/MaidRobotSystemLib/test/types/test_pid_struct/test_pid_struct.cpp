/**
 * @file test_pid_struct.cpp
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
#include "maid_robot_system/common/types/pid_struct.hpp"

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
    PIDstruct ins;
    TEST_ASSERT_EQUAL(0, ins.P);
    TEST_ASSERT_EQUAL(0, ins.I);
    TEST_ASSERT_EQUAL(0, ins.D);
    TEST_ASSERT_EQUAL(0, ins.previous_P);
}

void test_Construct_01(void)
{
    PIDstruct ins(12, 34, 56);
    TEST_ASSERT_EQUAL(12, ins.P);
    TEST_ASSERT_EQUAL(34, ins.I);
    TEST_ASSERT_EQUAL(56, ins.D);
    TEST_ASSERT_EQUAL(12, ins.previous_P);
}

void test_set(void)
{
    PIDstruct ins(12, 34, 56);
    ins.set(78, 90, 123);
    TEST_ASSERT_EQUAL(78, ins.P);
    TEST_ASSERT_EQUAL(90, ins.I);
    TEST_ASSERT_EQUAL(123, ins.D);
    TEST_ASSERT_EQUAL(12, ins.previous_P);
}
void test_variable(void)
{
    PIDstruct ins(12, 34, 56);
    ins.P = 456;
    ins.I = 789;
    ins.D = 1023;
    TEST_ASSERT_EQUAL(456, ins.P);
    TEST_ASSERT_EQUAL(789, ins.I);
    TEST_ASSERT_EQUAL(1023, ins.D);
    TEST_ASSERT_EQUAL(12, ins.previous_P);
}

void test_continuous_input(void)
{
    PIDstruct ins(12, 34, 56);
    ins.set(78, 90, 123);
    TEST_ASSERT_EQUAL(78, ins.P);
    TEST_ASSERT_EQUAL(90, ins.I);
    TEST_ASSERT_EQUAL(123, ins.D);
    TEST_ASSERT_EQUAL(12, ins.previous_P);
    ins.set(456, 789, 1023);
    TEST_ASSERT_EQUAL(456, ins.P);
    TEST_ASSERT_EQUAL(789, ins.I);
    TEST_ASSERT_EQUAL(1023, ins.D);
    TEST_ASSERT_EQUAL(78, ins.previous_P);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_Construct_00);
    RUN_TEST(test_Construct_01);
    RUN_TEST(test_set);
    RUN_TEST(test_variable);
    RUN_TEST(test_continuous_input);
    UNITY_END();
    return 0;
}
