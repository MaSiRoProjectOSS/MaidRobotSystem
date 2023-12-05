/**
 * @file test_control_if.cpp
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
#include "maid_robot_system/common/interface/control_if.hpp"

#include <unity.h>

ModeList g_mode;
void change_mode(ModeList mode)
{
    g_mode = mode;
}

class impl_ctl : public ControlIF {
public:
    impl_ctl()
    {
    }

protected:
    bool _begin()
    {
        return this->flag_begin;
    }
    bool _end()
    {
        return this->flag_end;
    }
    bool _calculate()
    {
        return this->flag_calculate;
    }
    bool _is_error()
    {
        return this->flag_error;
    }
    bool _enable()
    {
        return this->flag_enable;
    }
    bool _disable()
    {
        return this->flag_disable;
    }

public:
    void set_mode(ModeList mode)
    {
        this->_set_mode(mode);
    }

public:
    void set_flag_begin(bool flag)
    {
        this->flag_begin = flag;
    }
    bool flag_begin = true;
    void set_flag_end(bool flag)
    {
        this->flag_end = flag;
    }
    bool flag_end = true;
    void set_flag_calculate(bool flag)
    {
        this->flag_calculate = flag;
    }
    bool flag_calculate = true;
    void set_flag_is_error(bool flag)
    {
        this->flag_error = flag;
    }
    bool flag_error = true;
    void set_flag_enable(bool flag)
    {
        this->flag_enable = flag;
    }
    bool flag_enable = true;
    void set_flag_disable(bool flag)
    {
        this->flag_disable = flag;
    }
    bool flag_disable = true;
};

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_control_if_begin(void)
{
    bool result = false;
    impl_ctl impl;
    /////////////////////////////////////////
    impl.set_flag_begin(false);
    result = impl.begin();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    result = impl.begin();
    TEST_ASSERT_TRUE(result);
}
void test_control_if_end(void)
{
    bool result = false;
    impl_ctl impl;
    /////////////////////////////////////////
    result = impl.end();
    TEST_ASSERT_TRUE(result);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.set_flag_end(true);
    result = impl.begin();
    result = impl.end();
    TEST_ASSERT_TRUE(result);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.set_flag_end(false);
    result = impl.begin();
    result = impl.end();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
}
void test_control_if_calculate(void)
{
    bool result = false;
    impl_ctl impl;
    impl.set_flag_begin(true);
    impl.set_flag_calculate(true);
    impl.begin();
    /////////////////////////////////////////
    result = impl.calculate();
    TEST_ASSERT_TRUE(result);
    /////////////////////////////////////////
    impl.set_flag_calculate(false);
    result = impl.calculate();
    TEST_ASSERT_FALSE(result);
}
void test_control_if_enable_disable(void)
{
    bool result = false;
    impl_ctl impl;
    impl.set_flag_begin(true);
    impl.set_flag_calculate(true);
    impl.begin();
    /////////////////////////////////////////
    impl.disable();
    result = impl.calculate();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
    impl.enable();
    result = impl.calculate();
    TEST_ASSERT_TRUE(result);
}

void test_control_if_restart(void)
{
    bool result = false;
    impl_ctl impl;
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.set_flag_end(true);
    result = impl.restart();
    TEST_ASSERT_TRUE(result);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.set_flag_end(false);
    result = impl.restart();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
    impl.set_flag_begin(false);
    impl.set_flag_end(false);
    result = impl.restart();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
    impl.set_flag_begin(false);
    impl.set_flag_end(true);
    result = impl.restart();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
}
void test_control_if_is_enable(void)
{
    bool result = false;
    impl_ctl impl;
    impl.set_flag_begin(true);
    impl.set_flag_calculate(true);
    /////////////////////////////////////////
    impl.disable();
    result = impl.is_enable();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
    impl.enable();
    result = impl.is_enable();
    TEST_ASSERT_TRUE(result);
}
void test_control_if_is_error(void)
{
    bool result = false;
    impl_ctl impl;
    impl.set_flag_begin(true);
    impl.set_flag_calculate(true);
    /////////////////////////////////////////
    impl.set_flag_is_error(false);
    result = impl.is_error();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
    impl.set_flag_is_error(true);
    result = impl.is_error();
    TEST_ASSERT_TRUE(result);
}
void test_control_if_get_set_mode(void)
{
    ModeList mode;
    bool result = false;
    impl_ctl impl;
    impl.set_flag_begin(true);
    impl.set_flag_calculate(true);
    /////////////////////////////////////////
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_NOT_INITIALIZED, mode);
    /////////////////////////////////////////
    impl.set_flag_begin(false);
    impl.begin();
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_NOT_INITIALIZED, mode);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.begin();
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, mode);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.set_flag_end(false);
    impl.end();
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_NOT_STOP, mode);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.set_flag_end(true);
    impl.end();
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_FINISHED, mode);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.set_flag_end(true);
    impl.set_mode(ModeList::MODE_ERROR_GENERAL);
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_GENERAL, mode);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.set_flag_end(true);
    impl.set_mode(ModeList::MODE_ERROR_NOT_COMMUNICATION);
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_NOT_COMMUNICATION, mode);
    /////////////////////////////////////////
    impl.set_flag_begin(true);
    impl.set_flag_end(true);
    impl.set_mode(ModeList::MODE_ERROR_NOT_STOP);
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_NOT_STOP, mode);
    /////////////////////////////////////////
}
void test_control_if_system_lock_unlock(void)
{
    bool result = false;
    impl_ctl impl;
    impl.set_flag_begin(true);
    impl.set_flag_calculate(true);
    impl.begin();
    /////////////////////////////////////////
    impl.system_lock();
    result = impl.is_enable();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
    impl.system_unlock();
    result = impl.is_enable();
    TEST_ASSERT_TRUE(result);
    /////////////////////////////////////////
    impl.begin();
    /////////////////////////////////////////
    impl.system_lock();
    result = impl.is_enable();
    TEST_ASSERT_FALSE(result);
    /////////////////////////////////////////
    impl.system_unlock();
    result = impl.is_enable();
    TEST_ASSERT_TRUE(result);
}

void test_control_normal_sequence(void)
{
    bool result = false;
    ModeList mode;
    impl_ctl impl;
    impl.set_flag_begin(true);
    impl.set_flag_calculate(true);
    impl.set_flag_end(true);
    impl.set_flag_is_error(true);
    impl.set_flag_enable(true);
    impl.set_flag_disable(true);
    ///////////////////////////////////////////////////////////
    mode = impl.get_mode();
    TEST_ASSERT_EQUAL(ModeList::MODE_NOT_INITIALIZED, mode);
    ///////////////////////////////////////////////////////////
    result = impl.begin();
    mode   = impl.get_mode();
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, mode);
    ///////////////////////////////////////////////////////////
    result = impl.calculate();
    mode   = impl.get_mode();
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, mode);
    ///////////////////////////////////////////////////////////
    impl.disable();
    mode   = impl.get_mode();
    result = impl.is_enable();
    TEST_ASSERT_FALSE(result);
    ///////////////////////////////////////////////////////////
    result = impl.calculate();
    mode   = impl.get_mode();
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, mode);
    ///////////////////////////////////////////////////////////
    impl.enable();
    mode   = impl.get_mode();
    result = impl.is_enable();
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, mode);
    ///////////////////////////////////////////////////////////
    result = impl.calculate();
    mode   = impl.get_mode();
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, mode);
    ///////////////////////////////////////////////////////////
    impl.disable();
    result = impl.calculate();
    mode   = impl.get_mode();
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, mode);
    ///////////////////////////////////////////////////////////
    impl.set_flag_end(false);
    result = impl.end();
    mode   = impl.get_mode();
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL(ModeList::MODE_ERROR_NOT_STOP, mode);
    ///////////////////////////////////////////////////////////
    impl.set_flag_end(true);
    result = impl.end();
    mode   = impl.get_mode();
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(ModeList::MODE_FINISHED, mode);
    ///////////////////////////////////////////////////////////
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_control_if_begin);
    RUN_TEST(test_control_if_end);
    RUN_TEST(test_control_if_calculate);
    RUN_TEST(test_control_if_enable_disable);
    RUN_TEST(test_control_if_restart);
    RUN_TEST(test_control_if_is_enable);
    RUN_TEST(test_control_if_is_error);
    RUN_TEST(test_control_if_get_set_mode);
    RUN_TEST(test_control_if_system_lock_unlock);
    RUN_TEST(test_control_normal_sequence);
    UNITY_END();

    return 0;
}
