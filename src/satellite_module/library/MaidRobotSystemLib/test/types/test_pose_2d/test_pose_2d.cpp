/**
 * @file test_pose_2d.cpp
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
#include "maid_robot_system/common/types/pose_2d.hpp"

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
    Pose2D ins;
    TEST_ASSERT_EQUAL(0, ins.x);
    TEST_ASSERT_EQUAL(0, ins.y);
    TEST_ASSERT_EQUAL(0, ins.yaw);
}

void test_Construct_01(void)
{
    Pose2D ins(12, 34, 56);
    TEST_ASSERT_EQUAL(12, ins.x);
    TEST_ASSERT_EQUAL(34, ins.y);
    TEST_ASSERT_EQUAL(56, ins.yaw);
}

void test_set(void)
{
    Pose2D ins(12, 34, 56);
    ins.set(78, 90, 123);
    TEST_ASSERT_EQUAL(78, ins.x);
    TEST_ASSERT_EQUAL(90, ins.y);
    TEST_ASSERT_EQUAL(123, ins.yaw);
}
void test_variable(void)
{
    Pose2D ins(12, 34, 56);
    ins.x   = 456;
    ins.y   = 789;
    ins.yaw = 1023;
    TEST_ASSERT_EQUAL(456, ins.x);
    TEST_ASSERT_EQUAL(789, ins.y);
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
