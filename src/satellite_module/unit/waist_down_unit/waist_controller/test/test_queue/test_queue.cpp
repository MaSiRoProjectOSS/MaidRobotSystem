/**
 * @file test_queue.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-10
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "queue.hpp"

#include <unity.h>

//////////////////////////////////////////////////////////////////////////////////////

void test_queue_int_push_pop()
{
    Queue<int> *queue = new Queue<int>(10);
    queue->push(100);
    queue->push(101);
    queue->push(102);
    queue->push(103);
    queue->push(104);
    queue->push(105);
    TEST_ASSERT_EQUAL(6, queue->length());
    TEST_ASSERT_EQUAL(100, queue->pop());
    TEST_ASSERT_EQUAL(101, queue->pop());
    TEST_ASSERT_EQUAL(102, queue->pop());
    TEST_ASSERT_EQUAL(103, queue->pop());
    TEST_ASSERT_EQUAL(104, queue->pop());
    TEST_ASSERT_EQUAL(105, queue->pop());
    TEST_ASSERT_EQUAL(0, queue->length());
}
void test_queue_int_max()
{
    Queue<int> *queue = new Queue<int>(10);
    queue->push(100);
    queue->push(101);
    queue->push(102);
    queue->push(103);
    queue->push(104);
    queue->push(105);
    queue->push(106);
    queue->push(107);
    queue->push(108);
    queue->push(109);
    queue->push(110);
    TEST_ASSERT_EQUAL(10, queue->length());
    TEST_ASSERT_EQUAL(100, queue->pop());
    TEST_ASSERT_EQUAL(101, queue->pop());
    TEST_ASSERT_EQUAL(102, queue->pop());
    TEST_ASSERT_EQUAL(103, queue->pop());
    TEST_ASSERT_EQUAL(104, queue->pop());
    TEST_ASSERT_EQUAL(105, queue->pop());
    TEST_ASSERT_EQUAL(106, queue->pop());
    TEST_ASSERT_EQUAL(107, queue->pop());
    TEST_ASSERT_EQUAL(108, queue->pop());
    TEST_ASSERT_EQUAL(109, queue->pop());
    TEST_ASSERT_EQUAL(0, queue->length());
}
//////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_queue_int_push_pop);
    RUN_TEST(test_queue_int_max);
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
