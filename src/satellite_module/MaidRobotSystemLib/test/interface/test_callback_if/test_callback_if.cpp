/**
 * @file test_callback_if.cpp
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
#include "maid_robot_system/common/interface/callback_if.hpp"

#include <unity.h>

volatile int result_value_int = 0;
void func_int(int value)
{
    result_value_int = value;
}

volatile bool result_value_bool = 0;
void func_bool(bool value)
{
    result_value_bool = value;
}

class testCallback_int : public CallbackIF<int> {
public:
    testCallback_int()
    {
        hock_counter = 0;
    }

protected:
    bool hook_set_callback(CallbackIF<int>::CALLBACK_FUNC callback)
    {
        hock_counter++;
        return this->flag_result;
    }

public:
    void call(int value)
    {
        this->happened(value);
    }
    void set_callback_flag(bool value)
    {
        this->flag_result = value;
    }
    int hock_counter = 0;
    bool flag_result = true;
};

class testCallback_bool : public CallbackIF<bool> {
public:
    testCallback_bool()
    {
        hock_counter = 0;
    }

protected:
    bool hook_set_callback(CallbackIF<bool>::CALLBACK_FUNC callback)
    {
        hock_counter++;
        return this->flag_result;
    }

public:
    void call(bool value)
    {
        this->happened(value);
    }
    void set_callback_flag(bool value)
    {
        this->flag_result = value;
    }
    int hock_counter = 0;
    bool flag_result = true;
};

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_callback_if_int(void)
{
    result_value_int = 0;
    testCallback_int tci;
    //////////////////////////////////////
    tci.set_callback_flag(false);
    tci.set_callback(&func_int);
    tci.call(10);
    TEST_ASSERT_EQUAL(0, result_value_int);
    //////////////////////////////////////
    tci.set_callback_flag(true);
    tci.set_callback(&func_int);
    tci.call(10);
    TEST_ASSERT_EQUAL(10, result_value_int);
    //////////////////////////////////////
    tci.call(100);
    TEST_ASSERT_EQUAL(100, result_value_int);
}
void test_callback_if_bool(void)
{
    result_value_bool = false;
    testCallback_bool tcb;
    //////////////////////////////////////
    tcb.set_callback_flag(false);
    tcb.set_callback(&func_bool);
    tcb.call(true);
    TEST_ASSERT_FALSE(result_value_bool);
    //////////////////////////////////////
    tcb.set_callback_flag(true);
    tcb.set_callback(&func_bool);
    tcb.call(true);
    TEST_ASSERT_TRUE(result_value_bool);
    //////////////////////////////////////
    tcb.call(false);
    TEST_ASSERT_FALSE(result_value_bool);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_callback_if_int);
    RUN_TEST(test_callback_if_bool);
    UNITY_END();

    return 0;
}
