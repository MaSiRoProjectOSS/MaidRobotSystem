/**
 * @file main.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-05-03
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/model_on_arm_controller.hpp"

#include <Arduino.h>

maid_robot_system::arm_unit::ModelOnArmController model;

#ifndef PIO_UNIT_TESTING

/**
 * @brief setup function
 *
 */
void setup()
{
    model.setup();
}

/**
 * @brief loop function
 *
 */
void loop()
{
    delayMicroseconds(10);
    model.loop();
}

#endif
