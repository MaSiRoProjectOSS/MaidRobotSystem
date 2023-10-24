/**
 * @file test_model_if.cpp
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
#include "maid_robot_system/common/interface/model_if.hpp"
#include "maid_robot_system/common/time_check.hpp"

#include <Arduino.h>
#include <unity.h>

ModeList g_mode;
void change_mode(ModeList mode)
{
    g_mode = mode;
}

class ImplModel : public ModelIf {
public:
    ImplModel()
    {
    }

public:
    void set_mode(ModeList mode)
    {
        this->_set_mode(mode);
    }
    void set_flag_setup(bool flag)
    {
        this->flag_setup = flag;
    }
    void set_flag_receive(bool flag)
    {
        this->flag_receive = flag;
    }
    void set_flag_calculate(bool flag)
    {
        this->flag_calculate = flag;
    }
    void clear_counter()
    {
        this->cnt_setup     = 0;
        this->cnt_receive   = 0;
        this->cnt_calculate = 0;

        this->cnt_send         = 0;
        this->cnt_error_check  = 0;
        this->cnt_debug_output = 0;
    }

    int cnt_setup        = 0;
    int cnt_receive      = 0;
    int cnt_calculate    = 0;
    int cnt_send         = 0;
    int cnt_error_check  = 0;
    int cnt_debug_output = 0;

private:
    bool flag_setup     = true;
    bool flag_receive   = true;
    bool flag_calculate = true;

protected:
    bool _setup()
    {
        this->cnt_setup++;
        return this->flag_setup;
    }
    bool _receive(ModeList mode)
    {
        this->cnt_receive++;
        return this->flag_receive;
    }
    bool _calculate(ModeList mode)
    {
        this->cnt_calculate++;
        return this->flag_calculate;
    }
    void _send(ModeList mode)
    {
        this->cnt_send++;
    }
    void _error_check(ModeList mode)
    {
        this->cnt_error_check++;
    }
    void _debug_output(ModeList mode)
    {
        this->cnt_debug_output++;
    }
};

void setUp(void)
{
    // set stuff up here
    millis(0);
}

void tearDown(void)
{
    // clean stuff up here
}

void test_model_if_set_callback_change_mode(void)
{
    millis(0);
    bool result = false;
    ImplModel impl;
    impl.clear_counter();
    impl.set_flag_setup(true);
    ////////////////////////////////
    impl.set_callback_change_mode(&change_mode);
    g_mode = ModeList::MODE_NOT_INITIALIZED;
    impl.set_mode(ModeList::MODE_FINISHED);
    TEST_ASSERT_EQUAL(ModeList::MODE_FINISHED, g_mode);
}
void test_model_if_setup(void)
{
    millis(0);
    bool result = false;
    ImplModel impl;
    impl.set_callback_change_mode(&change_mode);
    impl.clear_counter();
    ////////////////////////////////
    impl.set_flag_setup(false);
    result = impl.setup();
    TEST_ASSERT_EQUAL(ModeList::MODE_NOT_INITIALIZED, g_mode);
    TEST_ASSERT_FALSE(result);
    ////////////////////////////////
    impl.set_flag_setup(true);
    result = impl.setup();
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, g_mode);
    TEST_ASSERT_TRUE(result);
}
void test_model_if_loop(void)
{
    millis(0);
    bool result = false;
    ImplModel impl;
    impl.set_callback_change_mode(&change_mode);
    ////////////////////////////////
    impl.clear_counter();
    impl.set_flag_setup(false);
    impl.setup();
    impl.loop();
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(0, impl.cnt_receive);
    TEST_ASSERT_EQUAL(0, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(0, impl.cnt_send);
    TEST_ASSERT_EQUAL(0, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(1, impl.cnt_debug_output);
    ////////////////////////////////
    impl.set_flag_setup(true);
    impl.setup();
    ////////////////////////////////
    millis(1000 * 10);
    impl.set_flag_receive(true);
    impl.set_flag_calculate(true);
    impl.loop();
    TEST_ASSERT_EQUAL(2, impl.cnt_setup);
    TEST_ASSERT_EQUAL(1, impl.cnt_receive);
    TEST_ASSERT_EQUAL(1, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(1, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(2, impl.cnt_debug_output);
    ////////////////////////////////
    millis(1000 * 20 + 1);
    impl.set_flag_receive(false);
    impl.set_flag_calculate(false);
    impl.loop();
    TEST_ASSERT_EQUAL(2, impl.cnt_setup);
    TEST_ASSERT_EQUAL(2, impl.cnt_receive);
    TEST_ASSERT_EQUAL(1, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(1, impl.cnt_send);
    TEST_ASSERT_EQUAL(2, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(3, impl.cnt_debug_output);
    ////////////////////////////////
    millis(1000 * 30 + 1);
    impl.set_flag_receive(true);
    impl.set_flag_calculate(false);
    impl.loop();
    TEST_ASSERT_EQUAL(2, impl.cnt_setup);
    TEST_ASSERT_EQUAL(3, impl.cnt_receive);
    TEST_ASSERT_EQUAL(2, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(1, impl.cnt_send);
    TEST_ASSERT_EQUAL(3, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(4, impl.cnt_debug_output);
    ////////////////////////////////
    millis(1000 * 40 + 1);
    impl.set_flag_receive(false);
    impl.set_flag_calculate(true);
    impl.loop();
    TEST_ASSERT_EQUAL(2, impl.cnt_setup);
    TEST_ASSERT_EQUAL(4, impl.cnt_receive);
    TEST_ASSERT_EQUAL(2, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(1, impl.cnt_send);
    TEST_ASSERT_EQUAL(4, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(5, impl.cnt_debug_output);
    ////////////////////////////////
    millis(1000 * 50 + 1);
    impl.set_flag_receive(true);
    impl.set_flag_calculate(true);
    impl.loop();
    TEST_ASSERT_EQUAL(2, impl.cnt_setup);
    TEST_ASSERT_EQUAL(5, impl.cnt_receive);
    TEST_ASSERT_EQUAL(3, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(2, impl.cnt_send);
    TEST_ASSERT_EQUAL(5, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(6, impl.cnt_debug_output);
    ////////////////////////////////
}

void test_model_if_get_set_mode(void)
{
    millis(0);
    bool result = false;
    ImplModel impl;
    ModeList mode;
    impl.set_callback_change_mode(&change_mode);
    ////////////////////////////////
    impl.set_mode(ModeList::MODE_NOT_INITIALIZED);
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_NOT_INITIALIZED, mode);
    TEST_ASSERT_EQUAL(ModeList::MODE_NOT_INITIALIZED, g_mode);
    ////////////////////////////////
    impl.set_mode(ModeList::MODE_RUNNING);
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, mode);
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, g_mode);
    ////////////////////////////////
    impl.set_mode(ModeList::MODE_FINISHED);
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_FINISHED, mode);
    TEST_ASSERT_EQUAL(ModeList::MODE_FINISHED, g_mode);
    ////////////////////////////////
    impl.set_mode(ModeList::MODE_ERROR_GENERAL);
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_GENERAL, mode);
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_GENERAL, g_mode);
    ////////////////////////////////
    impl.set_mode(ModeList::MODE_ERROR_NOT_COMMUNICATION);
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_NOT_COMMUNICATION, mode);
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_NOT_COMMUNICATION, g_mode);
    ////////////////////////////////
    impl.set_mode(ModeList::MODE_ERROR_NOT_STOP);
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_NOT_STOP, mode);
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_NOT_STOP, g_mode);
}

void test_model_if_set_interval_check_error_100(void)
{
    bool result           = false;
    int set_time          = 0;
    int model_period_time = 0;
    ImplModel impl;
    ModeList mode;
    impl.set_callback_change_mode(&change_mode);
    impl.set_flag_setup(true);
    impl.set_flag_receive(true);
    impl.set_flag_calculate(true);
    impl.clear_counter();
    ////////////////////////////////
    impl.set_interval_check_error(100);
    ////////////////////////////////
    millis(0);
    impl.setup();
    ////////////////////////////////
    set_time = model_period_time + 10;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(1, impl.cnt_receive);
    TEST_ASSERT_EQUAL(1, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(1, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(1, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 50;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(2, impl.cnt_receive);
    TEST_ASSERT_EQUAL(2, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(2, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(2, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 99;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(3, impl.cnt_receive);
    TEST_ASSERT_EQUAL(3, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(3, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(3, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 100;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(4, impl.cnt_receive);
    TEST_ASSERT_EQUAL(4, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(4, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(4, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 101;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(5, impl.cnt_receive);
    TEST_ASSERT_EQUAL(5, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(5, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(5, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 150;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(6, impl.cnt_receive);
    TEST_ASSERT_EQUAL(6, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(6, impl.cnt_send);
    TEST_ASSERT_EQUAL(2, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(6, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 252;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(7, impl.cnt_receive);
    TEST_ASSERT_EQUAL(7, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(7, impl.cnt_send);
    TEST_ASSERT_EQUAL(3, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(7, impl.cnt_debug_output);
}
void test_model_if_set_interval_check_error_500(void)
{
    bool result           = false;
    int set_time          = 0;
    int model_period_time = 0;
    ImplModel impl;
    ModeList mode;
    impl.set_callback_change_mode(&change_mode);
    impl.set_flag_setup(true);
    impl.set_flag_receive(true);
    impl.set_flag_calculate(true);
    impl.clear_counter();
    ////////////////////////////////
    impl.set_interval_check_error(500);
    ////////////////////////////////
    millis(0);
    impl.setup();
    ////////////////////////////////
    set_time = model_period_time + 10;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(1, impl.cnt_receive);
    TEST_ASSERT_EQUAL(1, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(1, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(1, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 50;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(2, impl.cnt_receive);
    TEST_ASSERT_EQUAL(2, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(2, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(2, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 499;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(3, impl.cnt_receive);
    TEST_ASSERT_EQUAL(3, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(3, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(3, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 500;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(4, impl.cnt_receive);
    TEST_ASSERT_EQUAL(4, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(4, impl.cnt_send);
    TEST_ASSERT_EQUAL(1, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(4, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 512;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(5, impl.cnt_receive);
    TEST_ASSERT_EQUAL(5, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(5, impl.cnt_send);
    TEST_ASSERT_EQUAL(2, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(5, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 1014;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(6, impl.cnt_receive);
    TEST_ASSERT_EQUAL(6, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(6, impl.cnt_send);
    TEST_ASSERT_EQUAL(3, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(6, impl.cnt_debug_output);
    ////////////////////////////////
    set_time = model_period_time + 1514;
    millis(set_time);
    impl.loop();
    printf("=========loop[%d]\n", set_time);
    TEST_ASSERT_EQUAL(1, impl.cnt_setup);
    TEST_ASSERT_EQUAL(7, impl.cnt_receive);
    TEST_ASSERT_EQUAL(7, impl.cnt_calculate);
    TEST_ASSERT_EQUAL(7, impl.cnt_send);
    TEST_ASSERT_EQUAL(3, impl.cnt_error_check);
    TEST_ASSERT_EQUAL(7, impl.cnt_debug_output);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_model_if_set_callback_change_mode);
    RUN_TEST(test_model_if_setup);
    RUN_TEST(test_model_if_loop);
    RUN_TEST(test_model_if_get_set_mode);
    RUN_TEST(test_model_if_set_interval_check_error_100);
    RUN_TEST(test_model_if_set_interval_check_error_500);
    UNITY_END();

    return 0;
}
