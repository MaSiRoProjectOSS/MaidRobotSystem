/**
 * @file use_device.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-04-07
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "use_device.hpp"

#if DEVICE_NAME == DEVICE_M5ATOM

void setup_m5()
{
    bool enable_serial  = true;
    bool enable_i2c     = true;
    bool enable_display = true;
    (void)M5.begin(enable_serial, enable_i2c, enable_display);
    (void)M5.dis.begin();
    notify_m5(NOTIFY_M5::NOTIFY_STARTUP);
}

void notify_m5(NOTIFY_M5 color)
{
    static NOTIFY_M5 current_color = NOTIFY_M5::NOTIFY_UNKNOWN;
    if (color != current_color) {
        current_color = color;
        switch (color) {
            case NOTIFY_M5::NOTIFY_WIFI_NOT_INITIALIZED:
                (void)M5.dis.fillpix(CRGB::Aqua);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_INITIALIZED:
                (void)M5.dis.fillpix(CRGB::Yellow);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_DISCONNECTED:
                (void)M5.dis.fillpix(CRGB::DarkRed);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_RETRY:
                (void)M5.dis.fillpix(CRGB::Red);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_CONNECTED_STA:
                (void)M5.dis.fillpix(CRGB::Green);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_CONNECTED_AP:
                (void)M5.dis.fillpix(CRGB::Blue);
                break;
            case NOTIFY_M5::NOTIFY_STARTUP:
                (void)M5.dis.fillpix(CRGB::White);
                break;
            case NOTIFY_M5::NOTIFY_UNKNOWN:
            default:
                (void)M5.dis.fillpix(CRGB::Black);
                break;
        }
    }
}
#elif DEVICE_NAME == DEVICE_M5CORE
int fontHeight   = 16;
int screenHeight = 240;
int linePos      = 0;

void setup_m5()
{
    M5.begin();
    M5.Power.begin();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);
    fontHeight   = M5.Lcd.fontHeight();
    screenHeight = M5.Lcd.height();
    both_println_lcd("FaultDetection.", WHITE, true);
}

void both_println_lcd(const char *value, uint16_t color, bool clear)
{
    static int line = 0;

    linePos += fontHeight;
    if ((screenHeight <= fontHeight) || (true == clear)) {
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.clear();
        char str[150];
        sprintf(str, "FaultDetection.");
        M5.Lcd.setTextColor(BLUE, BLACK);
        M5.Lcd.println(str);
        line    = 1;
        linePos = fontHeight * 3;
    }
    M5.Lcd.setTextColor(color, BLACK);
    M5.Lcd.println(value);
    line++;
}
void notify_m5(NOTIFY_M5 color)
{
    static NOTIFY_M5 current_color = NOTIFY_M5::NOTIFY_UNKNOWN;
    if (color != current_color) {
        current_color = color;
        switch (color) {
            case NOTIFY_M5::NOTIFY_WIFI_NOT_INITIALIZED:
                (void)both_println_lcd("WiFi not initialized.", WHITE, true);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_INITIALIZED:
                (void)both_println_lcd("WiFi initialized.", WHITE, true);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_DISCONNECTED:
                (void)both_println_lcd("WiFi disconnected.", WHITE, true);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_RETRY:
                (void)both_println_lcd("WiFi retry...", RED, true);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_CONNECTED_STA:
                (void)both_println_lcd("Connected [STA]", GREEN, true);
                break;
            case NOTIFY_M5::NOTIFY_WIFI_CONNECTED_AP:
                (void)both_println_lcd("Connected [AP]", GREEN, true);
                break;
            case NOTIFY_M5::NOTIFY_STARTUP:
                (void)both_println_lcd("Startup.", WHITE, true);
                break;
            case NOTIFY_M5::NOTIFY_UNKNOWN:
            default:
                (void)both_println_lcd("UNKNOWN.");
                break;
        }
    }
}
#else
#endif
