/**
 * @file test_log_callback.cpp
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
#include "maid_robot_system/common/log_callback.hpp"

#include <unity.h>

#define TEST_MESSAGE_001 "TEST_MESSAGE_001"
#define TEST_MESSAGE_002 "TEST_MESSAGE_002"
#define TEST_MESSAGE_003 "TEST_MESSAGE_003"
#define TEST_MESSAGE_004 "TEST_MESSAGE_004"
#define TEST_MESSAGE_005 "TEST_MESSAGE_005"
#define TEST_MESSAGE_006 "TEST_MESSAGE_006"
#define TEST_MESSAGE_007 "TEST_MESSAGE_007"

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}
LogCallback::OUTPUT_LOG_LEVEL g_lv;
char *g_message;
LogCallback::log_information g_info;

void MessageFunction(LogCallback::OUTPUT_LOG_LEVEL lv, //
                     const char *message,
                     LogCallback::log_information info)
{
    g_lv      = lv;
    g_message = (char *)message;
    g_info    = info;
}

void test_config_log_mode_enable(void)
{
    logger.config_log(true);
    logger.config_level(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE);
    LOG_FATAL(TEST_MESSAGE_007);
    //////////////////////////////
    LOG_TRACE(TEST_MESSAGE_001);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_001, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE, g_lv);
    //////////////////////////////
    LOG_DEBUG(TEST_MESSAGE_002);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_002, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_DEBUG, g_lv);
    //////////////////////////////
    LOG_INFO(TEST_MESSAGE_003);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_003, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO, g_lv);
    //////////////////////////////
    LOG_MESSAGE(TEST_MESSAGE_004);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_004, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_MESSAGE, g_lv);
    //////////////////////////////
    LOG_WARN(TEST_MESSAGE_005);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_005, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_WARN, g_lv);
    //////////////////////////////
    LOG_ERROR(TEST_MESSAGE_006);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_006, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_ERROR, g_lv);
    //////////////////////////////
    LOG_FATAL(TEST_MESSAGE_007);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_007, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, g_lv);
}
void test_config_log_mode_disable(void)
{
    logger.config_log(true);
    logger.config_level(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE);
    LOG_FATAL(TEST_MESSAGE_007);
    logger.config_log(false);
    //////////////////////////////
    LOG_TRACE(TEST_MESSAGE_001);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_007, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, g_lv);
    //////////////////////////////
    LOG_DEBUG(TEST_MESSAGE_002);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_007, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, g_lv);
    //////////////////////////////
    LOG_INFO(TEST_MESSAGE_003);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_007, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, g_lv);
    //////////////////////////////
    LOG_MESSAGE(TEST_MESSAGE_004);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_007, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, g_lv);
    //////////////////////////////
    LOG_WARN(TEST_MESSAGE_005);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_007, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, g_lv);
    //////////////////////////////
    LOG_ERROR(TEST_MESSAGE_006);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_007, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, g_lv);
    //////////////////////////////
    logger.config_log(true);
    LOG_TRACE(TEST_MESSAGE_001);
    logger.config_log(false);
    //////////////////////////////
    LOG_FATAL(TEST_MESSAGE_007);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_001, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE, g_lv);
}

void test_config_level(void)
{
    logger.config_log(true);
    logger.config_level(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE);
    //////////////////////////////
    LOG_INFO(TEST_MESSAGE_003);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_003, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO, g_lv);
    //////////////////////////////
    logger.config_level(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_WARN);
    LOG_INFO(TEST_MESSAGE_003);
    TEST_ASSERT_EQUAL_STRING_LEN(TEST_MESSAGE_003, g_message, 16);
    TEST_ASSERT_EQUAL(LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO, g_lv);
}

int main(int argc, char **argv)
{
    logger.set_callback_message(&MessageFunction);

    UNITY_BEGIN();
    RUN_TEST(test_config_log_mode_enable);
    RUN_TEST(test_config_log_mode_disable);
    RUN_TEST(test_config_level);
    UNITY_END();

    return 0;
}
