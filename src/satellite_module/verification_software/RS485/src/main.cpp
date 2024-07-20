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

#define DEVICE_M5AtomLite (0)
#define DEVICE_M5AtomS3   (1)

#include <Arduino.h>
#if DEVICE_NAME == DEVICE_M5AtomLite
#include <M5Atom.h>
void m5_led(CRGB color);
#elif DEVICE_NAME == DEVICE_M5AtomS3
#include <M5AtomS3.h>
#endif
#define SETTING_LOOP_TIME_SLEEP_DETECT 10
#define BOARD_RATE                     115200
#define SETTING_LOOP_LAP               3000

void read_serial1()
{
    char buffer[255]      = "";
    static char line[255] = "";
    int cnt               = 0;

    cnt = Serial1.readBytes(buffer, 255);
#if DEVICE_NAME == DEVICE_M5AtomLite
    m5_led(CRGB::Yellow);
    Serial.write(buffer);
#elif DEVICE_NAME == DEVICE_M5AtomS3
    M5.Lcd.fillRect(0, 50, 128, 128, TFT_BLACK);
    M5.Lcd.setTextColor(TFT_GREEN);
    sprintf(line, "CNT [%03d]", cnt);
    M5.Lcd.drawCenterString(line, 64, 50, &fonts::lgfxJapanGothicP_16);

    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.drawString(buffer, 5, 70, &fonts::lgfxJapanGothicP_16);
#endif
}
void serial_setup()
{
    Serial1.begin(BOARD_RATE);
    Serial1.onReceive(read_serial1);
    Serial1.setTimeout(10);
    Serial.begin(BOARD_RATE);
}

#if DEVICE_NAME == DEVICE_M5AtomLite
void m5_led(CRGB color)
{
    static CRGB current_color = CRGB::Black;
    if (color != current_color) {
        current_color = color;
        (void)M5.dis.fillpix(current_color);
    }
}

void m5_setup()
{
    bool enable_serial  = true;
    bool enable_i2c     = false;
    bool enable_display = true;
    (void)M5.begin(enable_serial, enable_i2c, enable_display);
    (void)M5.dis.begin();
    serial_setup();
    m5_led(CRGB::White);
}
void send_data()
{
    static int count = 0;
    Serial.printf("[S0] Btn is pressed[%03d]\n", count);
    Serial1.printf("Lite[%03d]\n", count);
    if (0 == count % 2) {
        m5_led(CRGB::Green);
    } else {
        m5_led(CRGB::Blue);
    }
    count++;
    if (1000 <= count) {
        count = 0;
    }
}
void m5_loop()
{
    static int MAX = SETTING_LOOP_LAP / SETTING_LOOP_TIME_SLEEP_DETECT;
    static int cnt = 0;
    M5.update();
    if (true == M5.Btn.wasPressed()) {
        send_data();
    }
    if (cnt >= MAX) {
        cnt = 0;
        send_data();
    }
    cnt++;
}
#elif DEVICE_NAME == DEVICE_M5AtomS3

void m5_setup()
{
    m5::M5Unified::config_t cfg;
    cfg.clear_display = true;
    cfg.serial_baudrate = BOARD_RATE;
    bool ledEnable = false;
    (void)AtomS3.begin(cfg, ledEnable);

    //////////////////////////////////////////////////////
    // Initalized LCD
    M5.Lcd.init();
    M5.Lcd.setTextWrap(true);
    M5.Lcd.clear(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextWrap(true);

    // Display title
    M5.Lcd.fillRect(0, 0, 128, 25, TFT_WHITE);        // Title background
    M5.Lcd.setTextColor(M5.Lcd.color565(20, 20, 20)); // char color
    M5.Lcd.drawString("ATOM", 12, 2, &fonts::Font4);  // Character display based on top center coordinates (display content, x, y)
    M5.Lcd.setTextColor(TFT_RED);                     // char color
    M5.Lcd.drawString("S3", 88, 2, &fonts::Font4);    // Character display based on top center coordinates (display content, x, y)
    M5.Lcd.fillRect(16, 14, 7, 2, TFT_RED);           // For the horizontal line of the "A" in ATOM

    //////////////////////////////////////////////////////
    serial_setup();
}
void send_data()
{
    static char buffer[255] = "";
    static int count = 0;
    Serial.printf("[S0] BtnA is pressed[%03d]\n", count);
    sprintf(buffer, "S3[%03d]\n", count);
    M5.Lcd.fillRect(0, 28, 128, 25, TFT_BLACK);
    M5.Lcd.setTextColor(TFT_CYAN);
    M5.Lcd.drawCentreString(buffer, 64, 28, &fonts::lgfxJapanGothicP_16);

    Serial1.printf(buffer);
    count++;
    if (1000 <= count) {
        count = 0;
    }
}
void m5_loop()
{
    static int MAX = SETTING_LOOP_LAP / SETTING_LOOP_TIME_SLEEP_DETECT;
    static int cnt = 0;
    M5.update();
    if (true == M5.BtnA.wasPressed()) {
        send_data();
    }
    if (cnt >= MAX) {
        cnt = 0;
        send_data();
    }
    cnt++;
}
#endif

////////////////////////////////////////////////////////

void setup()
{
    (void)m5_setup();
}

void loop()
{
    (void)m5_loop();
    (void)delay(SETTING_LOOP_TIME_SLEEP_DETECT);
}

#endif
