/**
 * @file test_main.cpp
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
#include "maid_robot_system/arm_controller/model_on_arm_controller.hpp"

#include <unity.h>

void setUp(void)
{
    /* set stuff up here */
}

void tearDown(void)
{
    /* clean stuff up here */
}

void test_model_setup(void)
{
    ModelOnArmController model;

    bool result = model.setup();

    TEST_ASSERT_TRUE(result);
}

void test_model_loop(void)
{
    ModelOnArmController model;
    model.setup();
    model.loop();
    ModeList mode = model.get_mode();

    TEST_ASSERT_EQUAL(ModeList::MODE_RUNNING, mode);
}

void RUN_UNITY_TESTS()
{
    UNITY_BEGIN();
    //////////////////////////////////
    RUN_TEST(test_model_setup);
    RUN_TEST(test_model_loop);
    //////////////////////////////////
    UNITY_END();
}

#ifdef ARDUINO
#include <Arduino.h>
void setup()
{
    /* NOTE!!! Wait for >2 secs */
    /* if board doesn't support software reset via Serial.DTR/RTS */
    delay(2000);

    RUN_UNITY_TESTS();
}
void loop()
{
}
#else
int main(int argc, char **argv)
{
    RUN_UNITY_TESTS();
    return 0;
}
#endif
