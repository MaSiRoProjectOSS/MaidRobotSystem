/**
 * @file test_time_check.cpp
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
#include "Arduino.h"
#include "maid_robot_system/common/time_check.hpp"

#include <Arduino.h>
#include <unity.h>

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

//////////////////////////////////////////

void test_time_check_update(void)
{
    millis(10);
    TimeCheck ts;
    millis(20);
    //////////////////////////////
    ts.update();

    //////////////////////////////
    millis(50);
    unsigned long elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(30, elapsed);
}

void test_time_check_from_scratch(void)
{
    millis(10);
    TimeCheck ts;
    millis(20);
    //////////////////////////////
    ts.from_scratch();
    //////////////////////////////
    millis(50);
    unsigned long elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(51, elapsed);
}

void test_time_check_check_passing_in_value(void)
{
    unsigned long elapsed = 0;
    int checking_time     = 100;
    int checking_time_max = checking_time * 2;
    millis(10);
    TimeCheck ts;
    millis(20);
    ts.update();
    //////////////////////////////
    bool result = ts.check_passing(checking_time);
    TEST_ASSERT_FALSE(result);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(2, elapsed);
    //////////////////////////////
    int count = 2;
    for (int i = 2; i < checking_time_max; i++) {
        if (true == ts.check_passing(checking_time)) {
            break;
        }
        count++;
    }
    TEST_ASSERT_EQUAL(100, count);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(1, elapsed);
}

void test_time_check_check_passing(void)
{
    unsigned long elapsed = 0;
    int checking_time     = 100;
    int checking_time_max = checking_time * 2;
    millis(10);
    TimeCheck ts(checking_time);
    millis(20);
    ts.update();
    //////////////////////////////
    bool result = ts.check_passing();
    TEST_ASSERT_FALSE(result);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(2, elapsed);
    //////////////////////////////
    int count = 2;
    for (int i = 2; i < checking_time_max; i++) {
        if (true == ts.check_passing()) {
            break;
        }
        count++;
    }
    TEST_ASSERT_EQUAL(100, count);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(1, elapsed);
}

void test_time_check_check_time_over_in_value(void)
{
    unsigned long elapsed = 0;
    int checking_time     = 100;
    int checking_time_max = checking_time * 2;
    millis(10);
    TimeCheck ts;
    millis(20);
    ts.update();
    //////////////////////////////
    bool result = ts.check_time_over(checking_time);
    TEST_ASSERT_FALSE(result);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(2, elapsed);
    //////////////////////////////
    int count = 2;
    for (int i = 2; i < checking_time_max; i++) {
        if (true == ts.check_time_over(checking_time)) {
            break;
        }
        count++;
    }
    TEST_ASSERT_EQUAL(100, count);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(102, elapsed);
}

void test_time_check_check_time_over(void)
{
    unsigned long elapsed = 0;
    int checking_time     = 100;
    int checking_time_max = checking_time * 2;
    millis(10);
    TimeCheck ts(checking_time);
    millis(20);
    ts.update();
    //////////////////////////////
    bool result = ts.check_time_over();
    TEST_ASSERT_FALSE(result);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(2, elapsed);
    //////////////////////////////
    int count = 2;
    for (int i = 2; i < checking_time_max; i++) {
        if (true == ts.check_time_over()) {
            break;
        }
        count++;
    }
    TEST_ASSERT_EQUAL(100, count);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(102, elapsed);
}

void test_time_check_check_time_over2(void)
{
    unsigned long elapsed = 0;
    int checking_time     = 100;
    int checking_time_max = checking_time * 2;
    millis(10);
    TimeCheck ts(checking_time);
    millis(20);
    ts.update();
    //////////////////////////////
    int count = 0;
    for (int i = 0; i < checking_time_max; i++) {
        if (true == ts.check_time_over()) {
            break;
        }
        count++;
    }
    TEST_ASSERT_EQUAL(100, count);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(102, elapsed);
}

void test_time_check_get_elapsed_lap_time_in_value(void)
{
    unsigned long elapsed = 0;
    int checking_time     = 100;
    int checking_time_max = checking_time * 2;
    millis(10);
    TimeCheck ts;
    millis(20);
    ts.update();
    //////////////////////////////
    elapsed = ts.get_elapsed_lap_time(checking_time);
    TEST_ASSERT_EQUAL(1, elapsed);
    //////////////////////////////
    int count = 1;
    for (int i = 1; i < checking_time_max; i++) {
        elapsed = ts.get_elapsed_lap_time(checking_time);
        if (checking_time < elapsed) {
            break;
        }
        count++;
    }
    TEST_ASSERT_EQUAL(100, count);
    TEST_ASSERT_EQUAL(101, elapsed);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(1, elapsed);
}
void test_time_check_get_elapsed_lap_time(void)
{
    unsigned long elapsed = 0;
    int checking_time     = 100;
    int checking_time_max = checking_time * 2;
    millis(10);
    TimeCheck ts(checking_time);
    millis(20);
    ts.update();
    //////////////////////////////
    elapsed = ts.get_elapsed_lap_time();
    TEST_ASSERT_EQUAL(1, elapsed);
    //////////////////////////////
    int count = 1;
    for (int i = 1; i < checking_time_max; i++) {
        elapsed = ts.get_elapsed_lap_time();
        if (checking_time < elapsed) {
            break;
        }
        count++;
    }
    TEST_ASSERT_EQUAL(100, count);
    TEST_ASSERT_EQUAL(101, elapsed);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(1, elapsed);
}
void test_time_check_get_elapsed_time(void)
{
    unsigned long elapsed = 0;
    millis(10);
    TimeCheck ts;
    millis(20);
    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(10, elapsed);
    //////////////////////////////
    ts.update();
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(1, elapsed);
}

void test_time_check_get_lap_time(void)
{
    unsigned long elapsed = 0;
    millis(10);
    TimeCheck ts;
    millis(20);
    //////////////////////////////
    elapsed = ts.get_lap_time();
    TEST_ASSERT_EQUAL(10, elapsed);

    //////////////////////////////
    elapsed = ts.get_elapsed_time();
    TEST_ASSERT_EQUAL(1, elapsed);
}

void test_time_check_get_s_curve_flag(void)
{
    double value = 0;
    millis(10);
    TimeCheck ts;
    millis(100);
    ts.update();
    //////////////////////////////
    value = ts.get_s_curve_flag(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.000247f, (float)value);
    //////////////////////////////
    millis(125);
    value = ts.get_s_curve_flag(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.146447f, (float)value);
    //////////////////////////////
    millis(150);
    value = ts.get_s_curve_flag(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.500000f, value);
    //////////////////////////////
    millis(175);
    value = ts.get_s_curve_flag(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.853553f, value);
    //////////////////////////////
    millis(200);
    value = ts.get_s_curve_flag(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.000000f, value);
    //////////////////////////////
    millis(225);
    value = ts.get_s_curve_flag(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.000000f, value);
    //////////////////////////////
    millis(500);
    value = ts.get_s_curve_flag(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.000000f, value);
}
void test_time_check_get_sin_cycle(void)
{
    double value = 0;
    millis(10);
    TimeCheck ts;
    millis(100);
    ts.update();
    //////////////////////////////
    value = ts.get_sin_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.015707f, value);
    //////////////////////////////
    millis(125);
    value = ts.get_sin_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.382683f, value);
    //////////////////////////////
    millis(150);
    value = ts.get_sin_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.707107f, value);
    //////////////////////////////
    millis(175);
    value = ts.get_sin_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.923880f, value);
    //////////////////////////////
    millis(200);
    value = ts.get_sin_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.000000f, value);
    //////////////////////////////
    millis(225);
    value = ts.get_sin_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.923880f, value);
}
void test_time_check_get_cos_cycle(void)
{
    double value = 0;
    millis(10);
    TimeCheck ts;
    millis(100);
    ts.update();
    //////////////////////////////
    value = ts.get_cos_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.999877f, value);
    //////////////////////////////
    millis(125);
    value = ts.get_cos_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.923880f, value);
    //////////////////////////////
    millis(150);
    value = ts.get_cos_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.707107f, value);
    //////////////////////////////
    millis(175);
    value = ts.get_cos_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.382683f, value);
    //////////////////////////////
    millis(200);
    value = ts.get_cos_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.000000f, value);
    //////////////////////////////
    millis(225);
    value = ts.get_cos_cycle(100);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.382683f, value);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_time_check_update);
    RUN_TEST(test_time_check_from_scratch);
    RUN_TEST(test_time_check_check_passing_in_value);
    RUN_TEST(test_time_check_check_passing);
    RUN_TEST(test_time_check_check_time_over_in_value);
    RUN_TEST(test_time_check_check_time_over);
    RUN_TEST(test_time_check_check_time_over2);
    RUN_TEST(test_time_check_get_elapsed_lap_time_in_value);
    RUN_TEST(test_time_check_get_elapsed_lap_time);
    RUN_TEST(test_time_check_get_elapsed_time);
    RUN_TEST(test_time_check_get_lap_time);
    RUN_TEST(test_time_check_get_s_curve_flag);
    RUN_TEST(test_time_check_get_sin_cycle);
    RUN_TEST(test_time_check_get_cos_cycle);
    UNITY_END();

    return 0;
}
