/**
 * @file test_candle_stick.cpp
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
#include "maid_robot_system/common/chart/candle_stick.hpp"

#include <unity.h>

//////////////////////////////////////////////////////////////////////////////////////
void test_pt_no_data()
{
    CandleStick candle_stick(1000);

    std::vector<CandleStick::CandleStickData> *list = candle_stick.get_list();
    TEST_ASSERT_EQUAL(0, list->size());
}

void test_pt_one_cycle()
{
    CandleStick candle_stick(1000);

    for (int i = 0; i < 10; i++) {
        candle_stick.set(i + 1, (i * 10) + 1);
    }
    std::vector<CandleStick::CandleStickData> *list = candle_stick.get_list();
    TEST_ASSERT_EQUAL(1, list->size());
    TEST_ASSERT_EQUAL(1, list->at(0).data.start_ms);
    TEST_ASSERT_EQUAL(91, list->at(0).data.end_ms);

    TEST_ASSERT_EQUAL(false, list->at(0).is_saved);

    TEST_ASSERT_EQUAL(10, list->at(0).data.number_of_samples);
    TEST_ASSERT_EQUAL_FLOAT(5.5, list->at(0).data.average);
    TEST_ASSERT_EQUAL_FLOAT(10.0, list->at(0).data.high);
    TEST_ASSERT_EQUAL_FLOAT(1.0, list->at(0).data.low);
    TEST_ASSERT_EQUAL_FLOAT(1.0, list->at(0).data.open);
    TEST_ASSERT_EQUAL_FLOAT(10.0, list->at(0).data.close);
}
void test_pt_three_cycle()
{
    CandleStick candle_stick(1000);

    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 10; i++) {
            candle_stick.set(i + 1, (1000 * j) + (i * 10) + 1);
        }
    }

    int index                                       = 2;
    std::vector<CandleStick::CandleStickData> *list = candle_stick.get_list();
    /////////////////////////////
    TEST_ASSERT_EQUAL(3, list->size());
    /////////////////////////////
    TEST_ASSERT_EQUAL(1, list->at(0).data.start_ms);
    TEST_ASSERT_EQUAL(91, list->at(0).data.end_ms);
    TEST_ASSERT_EQUAL(1001, list->at(1).data.start_ms);
    TEST_ASSERT_EQUAL(1091, list->at(1).data.end_ms);
    TEST_ASSERT_EQUAL(2001, list->at(2).data.start_ms);
    TEST_ASSERT_EQUAL(2091, list->at(2).data.end_ms);
    /////////////////////////////
    TEST_ASSERT_EQUAL(false, list->at(0).is_saved);

    TEST_ASSERT_EQUAL(false, list->at(1).is_saved);

    TEST_ASSERT_EQUAL(false, list->at(2).is_saved);
    /////////////////////////////

    TEST_ASSERT_EQUAL(10, list->at(index).data.number_of_samples);
    TEST_ASSERT_EQUAL_FLOAT(5.5, list->at(index).data.average);
    TEST_ASSERT_EQUAL_FLOAT(10.0, list->at(index).data.high);
    TEST_ASSERT_EQUAL_FLOAT(1.0, list->at(index).data.low);
    TEST_ASSERT_EQUAL_FLOAT(1.0, list->at(index).data.open);
    TEST_ASSERT_EQUAL_FLOAT(10.0, list->at(index).data.close);
}
void test_pt_clear()
{
    CandleStick candle_stick(1000);

    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < candle_stick.get_list()->size(); i++) {
            candle_stick.get_list()->at(i).mark();
            candle_stick.get_list()->at(i).save();
        }
        for (int i = 0; i < 10; i++) {
            candle_stick.set(i + 1, (1000 * j) + (i * 10) + 1);
        }
    }

    int index                                       = 0;
    std::vector<CandleStick::CandleStickData> *list = candle_stick.get_list();
    /////////////////////////////
    TEST_ASSERT_EQUAL(2, list->size());
    /////////////////////////////
    index = 0;
    TEST_ASSERT_EQUAL(1001, list->at(index).data.start_ms);
    TEST_ASSERT_EQUAL(1091, list->at(index).data.end_ms);
    TEST_ASSERT_EQUAL(false, list->at(index).is_saved);
    TEST_ASSERT_EQUAL(10, list->at(index).data.number_of_samples);
    TEST_ASSERT_EQUAL_FLOAT(5.5, list->at(index).data.average);
    TEST_ASSERT_EQUAL_FLOAT(10.0, list->at(index).data.high);
    TEST_ASSERT_EQUAL_FLOAT(1.0, list->at(index).data.low);
    TEST_ASSERT_EQUAL_FLOAT(1.0, list->at(index).data.open);
    TEST_ASSERT_EQUAL_FLOAT(10.0, list->at(index).data.close);
    /////////////////////////////
    index = 1;
    TEST_ASSERT_EQUAL(2001, list->at(index).data.start_ms);
    TEST_ASSERT_EQUAL(2091, list->at(index).data.end_ms);
    TEST_ASSERT_EQUAL(false, list->at(index).is_saved);
    TEST_ASSERT_EQUAL(10, list->at(index).data.number_of_samples);
    TEST_ASSERT_EQUAL_FLOAT(5.5, list->at(index).data.average);
    TEST_ASSERT_EQUAL_FLOAT(10.0, list->at(index).data.high);
    TEST_ASSERT_EQUAL_FLOAT(1.0, list->at(index).data.low);
    TEST_ASSERT_EQUAL_FLOAT(1.0, list->at(index).data.open);
    TEST_ASSERT_EQUAL_FLOAT(10.0, list->at(index).data.close);
}

//////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_pt_no_data);
    RUN_TEST(test_pt_one_cycle);
    RUN_TEST(test_pt_three_cycle);
    RUN_TEST(test_pt_clear);
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
