/**
 * @file use_device.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-04-07
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef USE_DEVICE_HPP_
#define USE_DEVICE_HPP_

#define DEVICE_M5CORE (0)
#define DEVICE_M5ATOM (1)

#if DEVICE_NAME == DEVICE_M5ATOM
#include <M5Atom.h>
#elif DEVICE_NAME == DEVICE_M5CORE
#include <M5Stack.h>
#else
#endif

enum NOTIFY_M5
{
    NOTIFY_UNKNOWN,
    NOTIFY_STARTUP,
    NOTIFY_WIFI_NOT_INITIALIZED,
    NOTIFY_WIFI_INITIALIZED,
    NOTIFY_WIFI_DISCONNECTED,
    NOTIFY_WIFI_RETRY,
    NOTIFY_WIFI_CONNECTED_STA,
    NOTIFY_WIFI_CONNECTED_AP,
};

void notify_m5(NOTIFY_M5 color);
void setup_m5();

#if DEVICE_NAME == DEVICE_M5ATOM
#elif DEVICE_NAME == DEVICE_M5CORE
void both_println_lcd(const char *value, uint16_t color = WHITE, bool clear = false);
#else
#endif

#endif
