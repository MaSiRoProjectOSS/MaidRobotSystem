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
#include "maid_robot_system/common/chart/four_dimensional.hpp"

#include <unity.h>

//////////////////////////////////////////////////////////////////////////////////////
void test_pt_no_data()
{
    FourDimensionalChart line;

    std::vector<FourDimensionalChart::FourDimensionalChartData> *list = line.get_list();
    TEST_ASSERT_EQUAL(0, list->size());
}

void test_pt_one_cycle()
{
    FourDimensionalChart line;

    for (int i = 0; i < 10; i++) {
        line.set(i + 1, 100 + i + 1, 1000 + i + 1, 10000 + i + 1, (i * 10) + 1);
    }
    std::vector<FourDimensionalChart::FourDimensionalChartData> *list = line.get_list();
    TEST_ASSERT_EQUAL(10, list->size());
    TEST_ASSERT_EQUAL(1, list->at(0).data.x);
    TEST_ASSERT_EQUAL(10, list->at(9).data.x);
    TEST_ASSERT_EQUAL(1, list->at(0).data.time_ms);
    TEST_ASSERT_EQUAL(91, list->at(9).data.time_ms);

    TEST_ASSERT_EQUAL(false, list->at(0).is_saved);
}

void test_pt_three_cycle()
{
    FourDimensionalChart line;

    for (int i = 0; i < 3; i++) {
        line.set(i + 1, 100 + i + 1, 1000 + i + 1, 10000 + i + 1, (i * 10) + 1);
    }

    int index                                                         = 0;
    std::vector<FourDimensionalChart::FourDimensionalChartData> *list = line.get_list();
    /////////////////////////////
    TEST_ASSERT_EQUAL(3, list->size());
    /////////////////////////////
    index = 0;
    TEST_ASSERT_EQUAL(1, list->at(index).data.x);
    TEST_ASSERT_EQUAL(1, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(false, list->at(index).is_saved);
    /////////////////////////////
    index = 1;
    TEST_ASSERT_EQUAL(2, list->at(index).data.x);
    TEST_ASSERT_EQUAL(11, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(false, list->at(index).is_saved);
    /////////////////////////////
    index = 2;
    TEST_ASSERT_EQUAL(3, list->at(index).data.x);
    TEST_ASSERT_EQUAL(21, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(false, list->at(index).is_saved);
}
void test_pt_clear()
{
    FourDimensionalChart line;

    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 10; i++) {
            line.set(i + 1, 100 + i + 1, 1000 + i + 1, 10000 + i + 1, (1000 * j) + (i * 10) + 1);
        }
    }
    for (int i = 0; i < line.get_list()->size() - 1; i++) {
        line.get_list()->at(i).mark();
        line.get_list()->at(i).save();
    }

    int index                                                         = 0;
    std::vector<FourDimensionalChart::FourDimensionalChartData> *list = line.get_list();
    /////////////////////////////
    TEST_ASSERT_EQUAL(30, list->size());
    /////////////////////////////
    index = 0;
    TEST_ASSERT_EQUAL(1, list->at(index).data.x);
    TEST_ASSERT_EQUAL(1, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(true, list->at(index).is_saved);
    /////////////////////////////
    index = 1;
    TEST_ASSERT_EQUAL(2, list->at(index).data.x);
    TEST_ASSERT_EQUAL(11, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(true, list->at(index).is_saved);
    /////////////////////////////
    index = 19;
    TEST_ASSERT_EQUAL(10, list->at(index).data.x);
    TEST_ASSERT_EQUAL(1091, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(true, list->at(index).is_saved);
    /////////////////////////////
    index = 20;
    TEST_ASSERT_EQUAL(1, list->at(index).data.x);
    TEST_ASSERT_EQUAL(2001, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(true, list->at(index).is_saved);
    /////////////////////////////
    index = 28;
    TEST_ASSERT_EQUAL(9, list->at(index).data.x);
    TEST_ASSERT_EQUAL(2081, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(true, list->at(index).is_saved);
    /////////////////////////////
    index = 29;
    TEST_ASSERT_EQUAL(10, list->at(index).data.x);
    TEST_ASSERT_EQUAL(2091, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(false, list->at(index).is_saved);
}
void test_pt_rewrite()
{
    int index = 0;
    FourDimensionalChart line;
    line.set(1, 101, 1001, 10001, 10);
    for (int i = 0; i < line.get_list()->size(); i++) {
        line.get_list()->at(i).mark();
        line.get_list()->at(i).save();
    }
    line.set(2, 102, 1002, 10002, 10);
    for (int i = 0; i < line.get_list()->size(); i++) {
        line.get_list()->at(i).mark();
        line.get_list()->at(i).save();
    }
    line.set(3, 103, 1003, 10003, 10);
    std::vector<FourDimensionalChart::FourDimensionalChartData> *list = line.get_list();
    /////////////////////////////
    TEST_ASSERT_EQUAL(1, list->size());
    /////////////////////////////
    index = 0;
    TEST_ASSERT_EQUAL(3, list->at(index).data.x);
    TEST_ASSERT_EQUAL(10, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(false, list->at(index).is_saved);
}
void test_pt_overwrite()
{
    int index = 0;
    FourDimensionalChart line;
    line.set(1, 101, 1001, 10001, 10);
    line.set(2, 102, 1002, 10002, 10);
    line.set(3, 103, 1003, 10003, 10);
    std::vector<FourDimensionalChart::FourDimensionalChartData> *list = line.get_list();
    /////////////////////////////
    TEST_ASSERT_EQUAL(1, list->size());
    /////////////////////////////
    index = 0;
    TEST_ASSERT_EQUAL(3, list->at(index).data.x);
    TEST_ASSERT_EQUAL(10, list->at(index).data.time_ms);
    TEST_ASSERT_EQUAL(false, list->at(index).is_saved);
}
//////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_pt_no_data);
    RUN_TEST(test_pt_one_cycle);
    RUN_TEST(test_pt_three_cycle);
    RUN_TEST(test_pt_rewrite);
    RUN_TEST(test_pt_overwrite);
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
