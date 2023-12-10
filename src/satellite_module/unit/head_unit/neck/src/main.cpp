/**
 * @file main.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.4
 * @date 2023-05-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "commutation_serial.hpp"
#include "xiao_imu.hpp"

#include <Arduino.h>

///////////////////////////////////////////////////////////////
CommutationSerial<25> cs;
XIAO::XIAOImu imu;

///////////////////////////////////////////////////////////////
void setup()
{
    cs.setup(&Serial);
    imu.setup();
}

void loop()
{
    static int rcl_ms = 1;
    ////////////////////////////////////////////////////
    static int count = 0;
    if (100 <= count++) {
        count = 0;
        imu.update();
        //////////////////////////////////
        char buffer[100];
        sprintf(buffer,
                "{"
                "\"a\":[%8.3f,%8.3f,%8.3f],"
                "\"g\":[%8.3f,%8.3f,%8.3f],"
                "\"m\":[%8.3f,%8.3f,%8.3f],"
                "\"t\":%5.1f"
                "}\n", //
                (imu.get_accel().x),
                (imu.get_accel().y),
                (imu.get_accel().z), //
                (imu.get_gyro().x),
                (imu.get_gyro().y),
                (imu.get_gyro().z),
                (0), // motor.up
                (0), // motor.left
                (0), // motor.right
                (imu.get_TempC()));
        Serial.print(buffer);
        //////////////////////////////////
    }

    cs.loop();
    ////////////////////////////////////////////////////
    delay(rcl_ms);
}
