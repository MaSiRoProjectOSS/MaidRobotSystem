/**
 * @file test_mode_list.cpp
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
#include "maid_robot_system/common/mode_list.hpp"

#include <unity.h>

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_MODE_NOT_INITIALIZED(void)
{
    char *buffer = text_contorl_mode(ModeList::MODE_NOT_INITIALIZED);
    TEST_ASSERT_EQUAL_STRING("Not initialized", buffer);
}
void test_MODE_RUNNING(void)
{
    char *buffer = text_contorl_mode(ModeList::MODE_RUNNING);
    TEST_ASSERT_EQUAL_STRING("Running", buffer);
}
void test_MODE_FINISHED(void)
{
    char *buffer = text_contorl_mode(ModeList::MODE_FINISHED);
    TEST_ASSERT_EQUAL_STRING("Finished", buffer);
}
void test_MODE_ERROR_GENERAL(void)
{
    char *buffer = text_contorl_mode(ModeList::MODE_ERROR_GENERAL);
    TEST_ASSERT_EQUAL_STRING("<ERROR>Geneal", buffer);
}
void test_MODE_ERROR_NOT_COMMUNICATION(void)
{
    char *buffer = text_contorl_mode(ModeList::MODE_ERROR_NOT_COMMUNICATION);
    TEST_ASSERT_EQUAL_STRING("<ERROR>Not communication", buffer);
}
void test_MODE_ERROR_NOT_STOP(void)
{
    char *buffer = text_contorl_mode(ModeList::MODE_ERROR_NOT_STOP);
    TEST_ASSERT_EQUAL_STRING("<ERROR>CAN'T SYSTEM STOP", buffer);
}
void test_UNKNOW(void)
{
    char *buffer = text_contorl_mode((ModeList)0xFF);
    TEST_ASSERT_EQUAL_STRING("UNKNOW[255]", buffer);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_MODE_NOT_INITIALIZED);
    RUN_TEST(test_MODE_RUNNING);
    RUN_TEST(test_MODE_FINISHED);
    RUN_TEST(test_MODE_ERROR_GENERAL);
    RUN_TEST(test_MODE_ERROR_NOT_COMMUNICATION);
    RUN_TEST(test_MODE_ERROR_NOT_STOP);
    RUN_TEST(test_UNKNOW);
    UNITY_END();
    return 0;
}
