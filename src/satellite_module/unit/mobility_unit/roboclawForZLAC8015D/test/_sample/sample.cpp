/**
 * @file native_test.cpp
 * @author Akari-mobility (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-07
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include <M5Atom.h>
#include <unity.h>

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

///////////////////////////////////////////////////////////////////

void test_setup(void)
{
    bool result = true;

    TEST_ASSERT_TRUE(result);
}

void test_loop(void)
{
    int result = 1;

    TEST_ASSERT_EQUAL(1, result);
}

///////////////////////////////////////////////////////////////////

void RUN_UNITY_TESTS()
{
    log_d("========================================");
    log_d("M5Atom initialized.");
    log_d("========================================");
    UNITY_BEGIN();
    //////////////////////////////////
    RUN_TEST(test_setup);
    RUN_TEST(test_loop);
    //////////////////////////////////
    UNITY_END();
}

///////////////////////////////////////////////////////////////////

#ifdef ARDUINO
#include <Arduino.h>
void setup()
{
    bool enable_serial  = true;
    bool enable_i2c     = false;
    bool enable_display = true;
    (void)M5.begin(enable_serial, enable_i2c, enable_display);
    (void)M5.dis.begin();
    (void)M5.dis.fillpix(CRGB::White);
    delay(200);
    (void)M5.dis.fillpix(CRGB::Black);
    delay(200);
    (void)M5.dis.fillpix(CRGB::White);

    RUN_UNITY_TESTS();
}
void loop()
{
    (void)M5.update();
    if (M5.Btn.wasPressed()) {
        RUN_UNITY_TESTS();
    }
}
#else
int main(int argc, char **argv)
{
    RUN_UNITY_TESTS();
    return 0;
}
#endif
