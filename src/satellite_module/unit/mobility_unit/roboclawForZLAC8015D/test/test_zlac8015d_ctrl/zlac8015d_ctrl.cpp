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

//////////////////////////////////
// connect to ZLAC8015D
//////////////////////////////////
void ReadVersion()
{
    bool result = true;
    // send 128 21

    TEST_ASSERT_TRUE(result);
}
void ResetEncoders()
{
    bool result = true;
    // send 128 20
    TEST_ASSERT_TRUE(result);
}

//////////////////////////////////
// get status
//////////////////////////////////
void ReadTemp()
{
    bool result = true;
    // send 128 82
    TEST_ASSERT_TRUE(result);
}
void ReadTemp2()
{
    bool result = true;
    // send 128 83
    TEST_ASSERT_TRUE(result);
}
void ReadLogicBatteryVoltage()
{
    bool result = true;
    // send 128 24
    TEST_ASSERT_TRUE(result);
}
void ReadMainBatteryVoltage()
{
    bool result = true;
    // send 128 25
    TEST_ASSERT_TRUE(result);
}
void ReadError()
{
    bool result = true;
    // send 128 90 0 218
    TEST_ASSERT_TRUE(result);
}

//////////////////////////////////
// Motor control
//////////////////////////////////
void ReadEncM1()
{
    bool result = true;
    // send 128 16
    TEST_ASSERT_TRUE(result);
}
void ReadEncM2()
{
    bool result = true;
    // send 128 17
    TEST_ASSERT_TRUE(result);
}
void ForwardM1()
{
    bool result = true;

    // send 128 0 0 128
    TEST_ASSERT_TRUE(result);
}
void ForwardM2()
{
    bool result = true;
    // send 128 4 0 132
    TEST_ASSERT_TRUE(result);
}
void SpeedM1M2()
{
    bool result = true;
    // send 128 37
    // send 0 0 0 0
    // send 0 0 0 0
    // send 0 165
    TEST_ASSERT_TRUE(result);
}

///////////////////////////////////////////////////////////////////

void RUN_UNITY_TESTS()
{
    log_d("========================================");
    log_d("M5Atom initialized.");
    log_d("========================================");
    UNITY_BEGIN();
    //////////////////////////////////
    // connect to ZLAC8015D
    RUN_TEST(ReadVersion);
    RUN_TEST(ResetEncoders);
    // get status
    RUN_TEST(ReadTemp);
    RUN_TEST(ReadTemp2);
    RUN_TEST(ReadLogicBatteryVoltage);
    RUN_TEST(ReadMainBatteryVoltage);
    RUN_TEST(ReadError);

    // Motor control
    RUN_TEST(ReadEncM1);
    RUN_TEST(ReadEncM2);
    RUN_TEST(ForwardM1);
    RUN_TEST(ForwardM2);
    RUN_TEST(SpeedM1M2);
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
