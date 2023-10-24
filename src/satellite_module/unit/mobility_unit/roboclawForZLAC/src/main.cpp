/**
 * @file main.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-01
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef PIO_UNIT_TESTING

#include "web/web_viewer_custom.hpp"

#include <Arduino.h>
////////////////////////////////////
#ifndef SETTING_LOOP_TIME_SLEEP_DETECT
#define SETTING_LOOP_TIME_SLEEP_DETECT (10)
#endif
////////////////////////////////////
const char *THREAD_MODEL_M5_NAME           = "ThreadModelM5";
const UBaseType_t THREAD_MODEL_M5_SIZE     = (4096);
const BaseType_t THREAD_MODEL_M5_CORE_ID   = 0;
const UBaseType_t THREAD_MODEL_M5_PRIORITY = 3;
TaskHandle_t _task_handle_model_m5;
volatile bool flag_thread_m5_initialized = false;
volatile bool flag_thread_m5_fin         = false;
const int THREAD_SEEK_INTERVAL_MODEL_M5  = 10;
static CRGB request_color                = CRGB::Black;
////////////////////////////////////
WebViewerCustom viewer;
volatile bool led_flash = true;

void m5_led(CRGB color)
{
    static CRGB current_color = CRGB::Black;
    if (color != current_color) {
        current_color = color;
        (void)M5.dis.fillpix(current_color);
    }
}
void m5_led_request(CRGB color)
{
    request_color = color;
}

void setup_m5()
{
    bool enable_serial  = true;
    bool enable_i2c     = false;
    bool enable_display = true;
    (void)M5.begin(enable_serial, enable_i2c, enable_display);
    (void)M5.dis.begin();
    m5_led(CRGB::White);
}

void thread_model_m5(void *args)
{
    const unsigned long SETTING_DOUBLE_CLICK = (1000); // 1000 ms - Double click judgment time

    flag_thread_m5_initialized = true;
    static int cnt_flash       = 0;
    static int cnt_flash_max   = 1000 / THREAD_SEEK_INTERVAL_MODEL_M5;

    while (false == flag_thread_m5_fin) {
        (void)M5.update();
        if (M5.Btn.wasPressed()) {
            static int click_time = 0;
            if (click_time > millis()) {
                viewer.pressed();
            } else {
                // one click
                click_time = millis() + SETTING_DOUBLE_CLICK;
            }
        }
        if (false == led_flash) {
            (void)m5_led(request_color);
        } else {
            cnt_flash++;
            if (cnt_flash <= cnt_flash_max) {
                (void)m5_led(request_color);
            } else if (cnt_flash <= (cnt_flash_max * 2)) {
                (void)m5_led(CRGB::Black);
            }
            if (cnt_flash >= (cnt_flash_max * 2)) {
                cnt_flash = 0;
            }
        }

        vTaskDelay(THREAD_SEEK_INTERVAL_MODEL_M5);
    }
    flag_thread_m5_initialized = false;
}

void viewer_mode(WebViewerCustom::WEB_VIEWER_MODE mode)
{
    switch (mode) {
        case WebViewerCustom::WEB_VIEWER_MODE::CONNECTED_AP:
        case WebViewerCustom::WEB_VIEWER_MODE::CONNECTED_STA:
            led_flash = false;
            break;

        default:
            led_flash = true;
            break;
    }
}

////////////////////////////////////////////////////////
void setup()
{
    (void)setup_m5();
    ////////////////////////////////////////////////////////
    viewer.set_callback_led(&m5_led_request);
    viewer.set_callback_mode(&viewer_mode);
    viewer.init();
    ////////////////////////////////////////////////////////
    xTaskCreatePinnedToCore(thread_model_m5, //
                            THREAD_MODEL_M5_NAME,
                            THREAD_MODEL_M5_SIZE,
                            NULL,
                            THREAD_MODEL_M5_PRIORITY,
                            &_task_handle_model_m5,
                            THREAD_MODEL_M5_CORE_ID);
}

void loop()
{
    (void)M5.update();
    viewer.loop();
    (void)delay(SETTING_LOOP_TIME_SLEEP_DETECT);
}

#endif
