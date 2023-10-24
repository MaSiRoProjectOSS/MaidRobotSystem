/**
 * @file test_move_average.cpp
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
#include "maid_robot_system/common/move_average.hpp"

#include <unity.h>

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_move_average_construct(void)
{
    MoveAverage<10> ma;

    TEST_ASSERT_EQUAL(0, ma.average);
    TEST_ASSERT_EQUAL(0, ma.average_delta);
    //////////////////////////////////

    MoveAverage<10> ma2(20);

    TEST_ASSERT_EQUAL(20, ma2.average);
    TEST_ASSERT_EQUAL(0, ma2.average_delta);
}

void test_move_average_set(void)
{
    MoveAverage<10> ma;
    double result = 0;
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, ma.average);
    result = ma.set(1);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1, result);
    result = ma.set(3);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2, result);
    result = ma.set(5);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3, result);
    result = ma.set(7);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 4, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 4, result);
    result = ma.set(9);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 5, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 5, result);
    result = ma.set(11);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 6, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 6, result);
    result = ma.set(13);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 7, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 7, result);
    result = ma.set(15);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 8, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 8, result);
    result = ma.set(17);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 9, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 9, result);
    result = ma.set(19);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 10, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 10, result);
    result = ma.set(11);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, result);
    result = ma.set(3);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, result);
    result = ma.set(5);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, result);
    result = ma.set(7);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, result);
    result = ma.set(9);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, result);
    result = ma.set(11);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, result);
    result = ma.set(9);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 10.6f, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 10.6f, result);
}

void test_move_average_set_delta(void)
{
    MoveAverage<10> ma;
    double result = 0;
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, ma.average);
    micros(1000 * 10);
    result = ma.set_delta(1);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1000000, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1000000, result);
    micros(1000 * 20);
    result = ma.set_delta(3);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 400, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 400, result);
    micros(1000 * 30);
    result = ma.set_delta(5);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 450, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 450, result);
    micros(1000 * 40);
    result = ma.set_delta(7);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 4, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 533.3333, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 533.3333, result);
    micros(1000 * 50);
    result = ma.set_delta(9);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 5, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 625, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 625, result);
    micros(1000 * 60);
    result = ma.set_delta(11);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 6, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 720, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 720, result);
    micros(1000 * 70);
    result = ma.set_delta(13);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 7, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 816.6667, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 816.6667, result);
    micros(1000 * 80);
    result = ma.set_delta(15);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 8, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 914.2857, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 914.2857, result);
    micros(1000 * 90);
    result = ma.set_delta(17);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 9, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1012.5, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1012.5, result);
    micros(1000 * 100);
    result = ma.set_delta(19);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 10, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1111.11111, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1111.11111, result);
    micros(1000 * 110);
    result = ma.set_delta(11);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100.011, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100.011, result);
    micros(1000 * 120);
    result = ma.set_delta(3);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, result);
    micros(1000 * 130);
    result = ma.set_delta(5);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, result);
    micros(1000 * 140);
    result = ma.set_delta(7);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, result);
    micros(1000 * 150);
    result = ma.set_delta(9);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, result);
    micros(1000 * 160);
    result = ma.set_delta(11);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 11, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1100, result);
    micros(1000 * 170);
    result = ma.set_delta(9);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 10.6f, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1060, ma.average_delta);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1060, result);
}

void test_move_average_reset_in_value(void)
{
    MoveAverage<10> ma;
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, ma.average_delta);
    ma.reset(20);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 20, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, ma.average_delta);
}

void test_move_average_reset(void)
{
    MoveAverage<10> ma(30);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, ma.average_delta);
    ma.reset(40);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 40, ma.average);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, ma.average_delta);
}

void test_move_average_get_text(void)
{
    MoveAverage<10> ma;
    String txt_0 = ma.get_text();
    printf("%s\n", txt_0.c_str());
    TEST_ASSERT_EQUAL_STRING_LEN("[SIZE: 0/10] ", txt_0.c_str(), 13);
    ma.set(1);
    ma.set(2);
    ma.set(3);
    ma.set(4);
    ma.set(5);
    String txt_5 = ma.get_text();
    printf("%s\n", txt_5.c_str());
    TEST_ASSERT_EQUAL_STRING_LEN("[SIZE: 5/10] 5, 4, 3, 2, 1", txt_5.c_str(), 26);
    ma.set(6);
    ma.set(7);
    ma.set(8);
    ma.set(9);
    ma.set(10);
    ma.set(11);
    String txt_11 = ma.get_text();
    printf("%s\n", txt_11.c_str());
    TEST_ASSERT_EQUAL_STRING_LEN("[SIZE: 10/10] 11, 10, 9, 8, 7, 6, 5, 4, 3, 2", txt_11.c_str(), 44);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_move_average_construct);
    RUN_TEST(test_move_average_set);
    RUN_TEST(test_move_average_set_delta);
    RUN_TEST(test_move_average_reset_in_value);
    RUN_TEST(test_move_average_reset);
    RUN_TEST(test_move_average_get_text);
    UNITY_END();

    return 0;
}
