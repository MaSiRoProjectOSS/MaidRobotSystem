/**
 * @file main.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "custom_web_server.hpp"
#include "sensor_ina225.hpp"
#include "sensor_sht31.hpp"
#include "use_device.hpp"

#include <SD.h>
#undef ESP32
#undef ESP8266
#undef ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#define ESP32
#include <std_msgs/Float32.h>

#ifndef TARGET_URL
#define TARGET_URL ""
#endif
#ifndef CAST_ID
#define CAST_ID 100
#endif
#ifndef TOPIC_NAME
#define TOPIC_NAME "/power_unit/power_monitor/voltage"
#endif
////////////////////////////////////////////////////////
#ifndef ID_SHT_CLOSE_TEMPERATURE
#define ID_SHT_CLOSE_TEMPERATURE 461
#endif
#ifndef ID_SHT_CLOSE_HUMIDITY
#define ID_SHT_CLOSE_HUMIDITY 462
#endif
#ifndef ID_SHT_OPEN_TEMPERATURE
#define ID_SHT_OPEN_TEMPERATURE 463
#endif
#ifndef ID_SHT_OPEN_HUMIDITY
#define ID_SHT_OPEN_HUMIDITY 464
#endif
#ifndef ID_INA_MT_BUS_VOLTAGE
#define ID_INA_MT_BUS_VOLTAGE 441
#endif
#ifndef ID_INA_MT_SHUNT_VOLTAGE
#define ID_INA_MT_SHUNT_VOLTAGE 442
#endif
#ifndef ID_INA_MT_CURRENT
#define ID_INA_MT_CURRENT 443
#endif
#ifndef ID_INA_CT_BUS_VOLTAGE
#define ID_INA_CT_BUS_VOLTAGE 451
#endif
#ifndef ID_INA_CT_SHUNT_VOLTAGE
#define ID_INA_CT_SHUNT_VOLTAGE 452
#endif
#ifndef ID_INA_CT_CURRENT
#define ID_INA_CT_CURRENT 453
#endif
////////////////////////////////////////////////////////
#ifndef FAULT_DETECTION_POST_JSON
#define FAULT_DETECTION_POST_JSON (1)
#endif
#ifndef OUTPUT_FOLDER_INFORMATION
#define OUTPUT_FOLDER_INFORMATION (0)
#endif
#ifndef ROS_BAUDRATE
#define ROS_BAUDRATE (57600)
#endif
////////////////////////////////////////////////////////
const char *THREAD_MODEL_M5_NAME           = "ThreadModelM5";
const UBaseType_t THREAD_MODEL_M5_SIZE     = (4096);
const BaseType_t THREAD_MODEL_M5_CORE_ID   = 1;
const UBaseType_t THREAD_MODEL_M5_PRIORITY = 3;
TaskHandle_t _task_handle_model_m5;
volatile bool flag_thread_m5_initialized = false;
volatile bool flag_thread_m5_fin         = false;
const int THREAD_SEEK_INTERVAL_MODEL_M5  = 10;
////////////////////////////////////////////////////////
const char store_path[]                           = "/data";
const int DATA_STORAGE_MAX                        = 1000;
const int CLEAN_UP_SIZE                           = 500;
const unsigned long SETTING_TIME_SLEEP_BACKUP     = (30 * 60 * 1000); // 30 min - Network未接続のSDへのバックアップ時間
const unsigned long SETTING_TIME_SLEEP_WEB_CLIENT = (30 * 1000);      // 30 sec - Web Clientの待機時間
const unsigned long SETTING_TIME_SLEEP_DISPLAY    = (30 * 1000);      // 30 sec - ディスプレイの表示時間
const unsigned long SETTING_TIME_SLEEP_SPLIT      = 50;               // 50 ms - 処理の最小単位
const unsigned long SETTING_DOUBLE_CLICK          = (1000);           // 1000 ms - ダブルクリックの判定時間
////////////////////////////////////////////////////////

SensorSht31 sht;
SensorIna225 ina;
CustomWebServer cushy;
bool flag_connected     = false;
volatile bool flag_push = false;
void notify_mode(CushyWebServer::WEB_VIEWER_MODE mode);
void data_save_to_sd();
void data_send_from_sd();
////////////////////////////////////////////////////////
#if OUTPUT_FOLDER_INFORMATION
void listDir(SDFS *sd, const char *dirname, uint8_t levels)
{
    log_i("Listing directory: %s", dirname);

    File root = sd->open(dirname);
    if (!root) {
        log_i(" - failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        log_i(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            log_i("  DIR : %s", file.path());
            if (levels) {
                listDir(sd, file.path(), levels - 1);
            }
        } else {
            log_i("  SIZE: [%5d]\tFILE: %s", file.size(), file.path());
        }
        file = root.openNextFile();
    }
}
#endif
////////////////////////////////////////////////////////

char *make_row_text(CandleStick::CandleStickData csd, int assy, int cast_id = CAST_ID)
{
    static char buffer[1024];
    sprintf(buffer,
            "{"
            "\"assy\": %d,"
            "\"cast\": %d,"
            "\"o\": %0.3f,"
            "\"c\": %0.3f,"
            "\"h\": %0.3f,"
            "\"l\": %0.3f,"
            "\"a\": %0.3f,"
            "\"n\": %u,"
            "\"utc_start\": %ld,"
            "\"utc_end\": %ld"
            "}",
            assy,                                    // "assy": %d
            cast_id,                                 // "cast": %d
            csd.data.open,                           // "o": %f
            csd.data.close,                          // "c": %f
            csd.data.high,                           // "h": %f
            csd.data.low,                            // "l": %f
            csd.data.average,                        // "a": %f
            csd.data.number_of_samples,              // "n": %d
            cushy.millis_to_time(csd.data.start_ms), // "utc_start": %d
            cushy.millis_to_time(csd.data.end_ms)    // "utc_end": %d
    );

    return buffer;
}

void data_saved(std::vector<CandleStick::CandleStickData> *list)
{
    for (int i = 0; i < list->size(); i++) {
        list->at(i).save();
    }
}
int set_data(std::vector<CandleStick::CandleStickData> *list, int assy, int cnt, std::string *param)
{
    static int MAX_SIZE = 25;
    for (int i = 0; i < list->size(); i++) {
        if (true == list->at(i).mark()) {
            if (MAX_SIZE <= cnt) {
                break;
            }
            if (true == list->at(i).is_saved) {
                continue;
            }
            if (0 != cnt) {
                param->append(",");
            }
            param->append(make_row_text(list->at(i), assy));
            cnt++;
        }
    }
    return cnt;
}

void cushy_setup(unsigned long current)
{
    static unsigned long next_time = 0;
    const unsigned long INTERVAL   = (45 * 1000); // 45 sec
    if (false == cushy.is_setup) {
        if (current > next_time) {
            next_time = current + INTERVAL;
            if (true == cushy.setup()) {
                next_time = __LONG_MAX__;
            }
        }
    }
}
void cushy_loop_push()
{
    UBaseType_t stack_cushy = cushy.get_stack_high_water_mark();
    UBaseType_t max_cushy   = cushy.get_stack_size();

    log_i("STACK SIZE : %d/%d", (max_cushy - stack_cushy), max_cushy);
}
void cushy_loop(unsigned long current)
{
    if (true != flag_connected) {
        cushy_setup(current);
        flag_connected = cushy.is_sntp_sync();
        ////////////////////////////////////////////////////////
        static unsigned long next_time = (2 * 60 * 1000);
        unsigned long current          = millis();
        bool flag_backup               = false;
        ////////////////////////////////////////////////////////
        if (false == flag_connected) {
            if (true == flag_push) {
                flag_push   = false;
                next_time   = 0;
                flag_backup = true;
            }
            // flag_connected is false
            if (current > next_time) {
                next_time = current + SETTING_TIME_SLEEP_BACKUP;
                if (true == sht.enable_close) {
                    if (DATA_STORAGE_MAX < sht.temperature_close->size()) {
                        flag_backup = true;
                    }
                    if (DATA_STORAGE_MAX < sht.humidity_close->size()) {
                        flag_backup = true;
                    }
                }
                if (true == sht.enable_open) {
                    if (DATA_STORAGE_MAX < sht.temperature_open->size()) {
                        flag_backup = true;
                    }
                    if (DATA_STORAGE_MAX < sht.humidity_open->size()) {
                        flag_backup = true;
                    }
                }
                if (true == ina.enable_mt) {
                    if (DATA_STORAGE_MAX < ina.mt_busVoltage->size()) {
                        flag_backup = true;
                    }
                    if (DATA_STORAGE_MAX < ina.mt_shuntVoltage->size()) {
                        flag_backup = true;
                    }
                    if (DATA_STORAGE_MAX < ina.mt_current->size()) {
                        flag_backup = true;
                    }
                }
                if (true == ina.enable_ct) {
                    if (DATA_STORAGE_MAX < ina.ct_busVoltage->size()) {
                        flag_backup = true;
                    }
                    if (DATA_STORAGE_MAX < ina.ct_shuntVoltage->size()) {
                        flag_backup = true;
                    }
                    if (DATA_STORAGE_MAX < ina.ct_current->size()) {
                        flag_backup = true;
                    }
                }
            }
            if (true == flag_backup) {
                data_save_to_sd();
            }
        } else {
            // flag_connected is true
            data_send_from_sd();
        }
    }
}
////////////////////////////////////////////////////////
void sd_save(SDFS *sd, char *file_name, std::vector<CandleStick::CandleStickData> *list, int assy)
{
    char path[255];
    sprintf(path, "%s/%d", store_path, assy);
    if (!sd->exists(store_path)) {
        if (sd->mkdir(store_path)) {
        }
    }
    if (!sd->exists(path)) {
        if (sd->mkdir(path)) {
        }
    }
    sprintf(path, "%s/%d/%s", store_path, assy, file_name);
    File file = sd->open(path, FILE_APPEND);
    if (file) {
        for (int i = 0; (i < CLEAN_UP_SIZE) && (i < list->size()); i++) {
            if (true == list->at(i).mark()) {
                if (true == list->at(i).is_saved) {
                    continue;
                }
                file.println(make_row_text(list->at(i), assy));
            }
        }
        file.close();
        data_saved(list);
    }
}
void data_save_to_sd()
{
    static int key_num = (int)random(0, 999);
    char file_name[255];
    time_t t = time(NULL);
    struct tm *tm;
    tm = localtime(&t);

    sprintf(file_name,
            "%04d%02d%02d_%02d%02d%02d_%03d.txt", //
            tm->tm_year + 1900,
            tm->tm_mon + 1,
            tm->tm_mday,
            tm->tm_hour,
            tm->tm_min,
            tm->tm_sec,
            key_num);

    if (true == SD.begin(TFCARD_CS_PIN)) {
        log_i("SAVE data to SD card");
        if (true == sht.enable_close) {
            sd_save(&SD, file_name, sht.temperature_close->get_list(), 461);
            sd_save(&SD, file_name, sht.humidity_close->get_list(), 462);
        }
        if (true == sht.enable_open) {
            sd_save(&SD, file_name, sht.temperature_open->get_list(), 461);
            sd_save(&SD, file_name, sht.humidity_open->get_list(), 462);
        }
        if (true == ina.enable_mt) {
            sd_save(&SD, file_name, ina.mt_busVoltage->get_list(), 441);
            sd_save(&SD, file_name, ina.mt_shuntVoltage->get_list(), 442);
            sd_save(&SD, file_name, ina.mt_current->get_list(), 443);
        }
        if (true == ina.enable_ct) {
            sd_save(&SD, file_name, ina.ct_busVoltage->get_list(), 451);
            sd_save(&SD, file_name, ina.ct_shuntVoltage->get_list(), 452);
            sd_save(&SD, file_name, ina.ct_current->get_list(), 453);
        }
#if OUTPUT_FOLDER_INFORMATION
        listDir(&SD, store_path, 2);
#endif
        SD.end();
    }
}

void data_send_from_sd()
{
    // ファイル検索をする。

    // 対象のファイルのデータを取得する

    // サーバに送る。
}
////////////////////////////////////////////////////////

void handle_client()
{
    static unsigned long next_time = 0;
    ////////////////////////////////////////////////////////

    if (true == flag_connected) {
        ////////////////////////////////////////////////////////
        unsigned long current = millis();
        ////////////////////////////////////////////////////////
        if (true == flag_push) {
            flag_push = false;
            next_time = 0;
        }
        if (current > next_time) {
            int cnt           = 0;
            std::string param = "";
            next_time         = current + SETTING_TIME_SLEEP_WEB_CLIENT;
            param.append("{\"data\":[");

            if (true == sht.enable_close) {
                cnt = set_data(sht.temperature_close->get_list(), ID_SHT_CLOSE_TEMPERATURE, cnt, &param);
                cnt = set_data(sht.humidity_close->get_list(), ID_SHT_CLOSE_HUMIDITY, cnt, &param);
            }
            if (true == sht.enable_open) {
                cnt = set_data(sht.temperature_open->get_list(), ID_SHT_OPEN_TEMPERATURE, cnt, &param);
                cnt = set_data(sht.humidity_open->get_list(), ID_SHT_OPEN_HUMIDITY, cnt, &param);
            }
            if (true == ina.enable_mt) {
                cnt = set_data(ina.mt_busVoltage->get_list(), ID_INA_MT_BUS_VOLTAGE, cnt, &param);
                cnt = set_data(ina.mt_shuntVoltage->get_list(), ID_INA_MT_SHUNT_VOLTAGE, cnt, &param);
                cnt = set_data(ina.mt_current->get_list(), ID_INA_MT_CURRENT, cnt, &param);
            }
            if (true == ina.enable_ct) {
                cnt = set_data(ina.ct_busVoltage->get_list(), ID_INA_CT_BUS_VOLTAGE, cnt, &param);
                cnt = set_data(ina.ct_shuntVoltage->get_list(), ID_INA_CT_SHUNT_VOLTAGE, cnt, &param);
                cnt = set_data(ina.ct_current->get_list(), ID_INA_CT_CURRENT, cnt, &param);
            }
            param.append("]}");
            if (0 < cnt) {
#if FAULT_DETECTION_POST_JSON
                StaticJsonDocument<512> replay;
                if (true == cushy.post_json(TARGET_URL, param.c_str(), &replay)) {
                    if ("OK" == replay["result"]) {
                        if (true == sht.enable_close) {
                            data_saved(sht.temperature_close->get_list());
                            data_saved(sht.humidity_close->get_list());
                        }
                        if (true == sht.enable_open) {
                            data_saved(sht.temperature_open->get_list());
                            data_saved(sht.humidity_open->get_list());
                        }
                        if (true == ina.enable_mt) {
                            data_saved(ina.mt_busVoltage->get_list());
                            data_saved(ina.mt_shuntVoltage->get_list());
                            data_saved(ina.mt_current->get_list());
                        }
                        if (true == ina.enable_ct) {
                            data_saved(ina.ct_busVoltage->get_list());
                            data_saved(ina.ct_shuntVoltage->get_list());
                            data_saved(ina.ct_current->get_list());
                        }
#if 1
                        const char *txt_result = replay["result"];
                        int code               = replay["status"]["code"];
                        log_i("<%lu>%s [code:%d][cnt:%d]", (millis() - current), txt_result, code, cnt);
#endif
                    } else {
                        const char *message = replay["status"]["messages"];
                        int code            = replay["status"]["code"];
                        log_w("<%lu>%s [code:%d][cnt:%d]", (millis() - current), message, code, cnt);
                    }
                } else {
                    log_w("Send failed.");
                }
#else
                String replay;
                if (true == cushy.post_json(TARGET_URL, param.c_str(), &replay)) {
                    if (true == sht.enable) {
                        data_saved(sht.temperature->get_list());
                        data_saved(sht.humidity->get_list());
                    }
                    if (true == ina.enable_mt) {
                        data_saved(ina.mt_busVoltage->get_list());
                        data_saved(ina.mt_shuntVoltage->get_list());
                        data_saved(ina.mt_current->get_list());
                    }
                    if (true == ina.enable_ct) {
                        data_saved(ina.ct_busVoltage->get_list());
                        data_saved(ina.ct_shuntVoltage->get_list());
                        data_saved(ina.ct_current->get_list());
                    }
                    log_i("<%lu>%s[cnt:%d]", (millis() - current), replay.c_str(), cnt);
                } else {
                    log_w("Send failed.");
                }
#endif
            } else {
                log_v("No data.");
            }
        }
    }
}
////////////////////////////////////////////////////////
#if DEVICE_NAME == DEVICE_M5ATOM
void loop_m5(unsigned long current)
{
#if 0
        (void)M5.update();
        if (true == M5.Btn.wasPressed()) {
            log_i("Button was pressed.");
            cushy_loop_push();
        }
#endif
}
void notify_mode(CushyWebServer::WEB_VIEWER_MODE mode)
{
    switch (mode) {
        case CushyWebServer::WEB_VIEWER_MODE::NOT_INITIALIZED:
            log_d("NOT_INITIALIZED");
            (void)notify_m5(NOTIFY_M5::NOTIFY_WIFI_NOT_INITIALIZED);
            flag_connected = false;
            break;
        case CushyWebServer::WEB_VIEWER_MODE::INITIALIZED:
            log_d("INITIALIZED");
            (void)notify_m5(NOTIFY_M5::NOTIFY_WIFI_INITIALIZED);
            flag_connected = false;
            break;
        case CushyWebServer::WEB_VIEWER_MODE::DISCONNECTED:
            log_d("DISCONNECTED");
            (void)notify_m5(NOTIFY_M5::NOTIFY_WIFI_DISCONNECTED);
            flag_connected = false;
            break;
        case CushyWebServer::WEB_VIEWER_MODE::RETRY:
            log_d("RETRY");
            (void)notify_m5(NOTIFY_M5::NOTIFY_WIFI_RETRY);
            flag_connected = false;
            break;
        case CushyWebServer::WEB_VIEWER_MODE::CONNECTED_STA:
            log_d("CONNECTED_STA");
            (void)notify_m5(NOTIFY_M5::NOTIFY_WIFI_CONNECTED_STA);
            break;
        case CushyWebServer::WEB_VIEWER_MODE::CONNECTED_AP:
            log_d("CONNECTED_AP");
            (void)notify_m5(NOTIFY_M5::NOTIFY_WIFI_CONNECTED_AP);
            break;
        default:
            (void)notify_m5(NOTIFY_M5::NOTIFY_UNKNOWN);
            flag_connected = false;
            break;
    }
}
#elif DEVICE_NAME == DEVICE_M5CORE
volatile bool flag_notify = false;

void loop_m5(unsigned long current)
{
    static unsigned long next_time = 0;
    static bool flag_lighting = false;
    bool flag_wakeup = false;
#if 1
    (void)M5.update();
    if (true == M5.BtnA.wasPressed()) {
        log_i("M5.BtnA.wasPressed");
        flag_wakeup = true;
        flag_push = true;
    }
    if (true == M5.BtnB.wasPressed()) {
        log_i("M5.BtnB.wasPressed");
        flag_wakeup = true;
        static int click_time = 0;
        if (click_time > millis()) {
            log_i("RECONNECT");
            cushy.list_reconnect_sta();
        } else {
            // one click
            click_time = millis() + SETTING_DOUBLE_CLICK;
        }
    }
    if (true == M5.BtnC.wasPressed()) {
        log_i("M5.BtnC.wasPressed");
        flag_wakeup = true;
    }
    ////////////////////////////////////////////////////////
    if (true == flag_notify) {
        log_i("flag_notify");
        flag_notify = false;
        flag_wakeup = true;
    }
    ////////////////////////////////////////////////////////
    if (true == flag_wakeup) {
        next_time = current + SETTING_TIME_SLEEP_DISPLAY;
        if (false == flag_lighting) {
            M5.Lcd.wakeup();
            flag_lighting = true;
        }
        char buffer[255];
        (void)both_println_lcd("", WHITE, true);
        if (true == sht.enable_close) {
            if (0 < sht.temperature_close->size()) {
                CandleStick::CandleStickData tmp_c = sht.temperature_close->get_list()->back();
                sprintf(buffer, "SHT_C: %5.3f [C]/size[%d]", tmp_c.data.average, (int)sht.temperature_close->size());
                (void)both_println_lcd(buffer);
            }
            if (0 < sht.humidity_close->size()) {
                CandleStick::CandleStickData hum_c = sht.humidity_close->get_list()->back();
                sprintf(buffer, "SHT_C: %5.3f [%%]/size[%d]", hum_c.data.average, (int)sht.humidity_close->size());
                (void)both_println_lcd(buffer);
            }
        }
        if (true == sht.enable_open) {
            if (0 < sht.temperature_open->size()) {
                CandleStick::CandleStickData tmp = sht.temperature_open->get_list()->back();
                sprintf(buffer, "SHT_O: %5.3f [C]/size[%d]", tmp.data.average, (int)sht.temperature_open->size());
                (void)both_println_lcd(buffer);
            }
            if (0 < sht.humidity_open->size()) {
                CandleStick::CandleStickData hum = sht.humidity_open->get_list()->back();
                sprintf(buffer, "SHT_O: %5.3f [%%]/size[%d]", hum.data.average, (int)sht.humidity_open->size());
                (void)both_println_lcd(buffer);
            }
        }
        if (true == ina.enable_mt) {
            if (0 < ina.mt_busVoltage->size()) {
                CandleStick::CandleStickData volt = ina.mt_busVoltage->get_list()->back();
                sprintf(buffer, "MT:%5.3f [mV]/size[%d]", volt.data.average, (int)ina.mt_busVoltage->size());
                (void)both_println_lcd(buffer);
            } else {
                (void)both_println_lcd("", WHITE);
            }
            if (0 < ina.mt_current->size()) {
                CandleStick::CandleStickData cur = ina.mt_current->get_list()->back();
                sprintf(buffer, "MT:%5.3f [mA]/size[%d]", cur.data.average, (int)ina.mt_current->size());
                (void)both_println_lcd(buffer);
            } else {
                (void)both_println_lcd("", WHITE);
            }
        } else {
            (void)both_println_lcd("", WHITE);
            (void)both_println_lcd("", WHITE);
        }
        if (true == ina.enable_ct) {
            if (0 < ina.ct_busVoltage->size()) {
                CandleStick::CandleStickData volt = ina.ct_busVoltage->get_list()->back();
                sprintf(buffer, "CT:%5.3f [mV]/size[%d]", volt.data.average, (int)ina.ct_busVoltage->size());
                (void)both_println_lcd(buffer);
            } else {
                (void)both_println_lcd("", WHITE);
            }
            if (0 < ina.ct_current->size()) {
                CandleStick::CandleStickData cur = ina.ct_current->get_list()->back();
                sprintf(buffer, "CT:%5.3f [mA]/size[%d]", cur.data.average, (int)ina.ct_current->size());
                (void)both_println_lcd(buffer);
            } else {
                (void)both_println_lcd("", WHITE);
            }
        } else {
            (void)both_println_lcd("", WHITE);
            (void)both_println_lcd("", WHITE);
        }

        (void)both_println_lcd("", WHITE);
        CushyWebServer::WEB_VIEWER_MODE mode = cushy.get_mode();
        switch (mode) {
            case CushyWebServer::WEB_VIEWER_MODE::NOT_INITIALIZED:
                log_d("NOT_INITIALIZED");
                (void)both_println_lcd("WiFi not initialized.", WHITE);
                break;
            case CushyWebServer::WEB_VIEWER_MODE::INITIALIZED:
                log_d("INITIALIZED");
                (void)both_println_lcd("WiFi initialized.", WHITE);
                break;
            case CushyWebServer::WEB_VIEWER_MODE::DISCONNECTED:
                log_d("DISCONNECTED");
                (void)both_println_lcd("WiFi disconnected.", WHITE);
                break;
            case CushyWebServer::WEB_VIEWER_MODE::RETRY:
                log_d("RETRY");
                (void)both_println_lcd("WiFi retry...", RED);
                break;
            case CushyWebServer::WEB_VIEWER_MODE::CONNECTED_STA:
                log_d("CONNECTED_STA");
                (void)both_println_lcd("Connected [STA]", GREEN);
                sprintf(buffer, "SSID: %s", cushy.get_ssid_sta());
                (void)both_println_lcd(buffer, GREEN);
                sprintf(buffer, "  IP: %s", cushy.get_ip_address_sta().toString().c_str());
                (void)both_println_lcd(buffer, GREEN);
                break;
            case CushyWebServer::WEB_VIEWER_MODE::CONNECTED_AP:
                log_d("CONNECTED_AP");
                (void)both_println_lcd("Connected [AP]", GREEN);
                sprintf(buffer, "SSID: %s", cushy.get_ssid_ap());
                (void)both_println_lcd(buffer, GREEN);
                sprintf(buffer, "  IP: %s", cushy.get_ip_address_ap().toString().c_str());
                (void)both_println_lcd(buffer, GREEN);
                break;
            default:
                (void)both_println_lcd("Startup.", WHITE);
                break;
        }
        log_i("M5.Lcd.wakeup");
    }
    if (current > next_time) {
        if (true == flag_lighting) {
            flag_lighting = false;
            M5.Lcd.sleep();
            log_i("M5.Lcd.sleep");
        }
    }
    ////////////////////////////////////////////////////////
#endif
}
void notify_mode(CushyWebServer::WEB_VIEWER_MODE mode)
{
    switch (mode) {
        case CushyWebServer::WEB_VIEWER_MODE::NOT_INITIALIZED:
            log_d("NOT_INITIALIZED");
            flag_connected = false;
            break;
        case CushyWebServer::WEB_VIEWER_MODE::INITIALIZED:
            log_d("INITIALIZED");
            flag_connected = false;
            break;
        case CushyWebServer::WEB_VIEWER_MODE::DISCONNECTED:
            log_d("DISCONNECTED");
            flag_connected = false;
            break;
        case CushyWebServer::WEB_VIEWER_MODE::RETRY:
            log_d("RETRY");
            flag_connected = false;
            break;
        case CushyWebServer::WEB_VIEWER_MODE::CONNECTED_STA:
            log_d("CONNECTED_STA");
            break;
        case CushyWebServer::WEB_VIEWER_MODE::CONNECTED_AP:
            log_d("CONNECTED_AP");
            break;
        default:
            flag_connected = false;
            break;
    }

    flag_notify = true;
    log_i("notify_mode");
}
#else
#endif
////////////////////////////////////////////////////////

void thread_model_m5(void *args)
{
    const unsigned long SETTING_DOUBLE_CLICK = (1000); // 1000 ms - Double click judgment time
    flag_thread_m5_initialized               = true;
    unsigned long LOOP_INTERVAL              = 100;
    unsigned long next_time                  = millis() + LOOP_INTERVAL;
    std_msgs::Float32 msg;
    ros::NodeHandle nh;
    ros::Publisher chatter(TOPIC_NAME, &msg);
    nh.initNode();
    nh.setSpinTimeout(THREAD_SEEK_INTERVAL_MODEL_M5);
    nh.getHardware()->setBaud(ROS_BAUDRATE);
    nh.advertise(chatter);

    while (false == flag_thread_m5_fin) {
        unsigned long current = millis();
        ////////////////////////////////////////////////////////
        if (current > next_time) {
            next_time = current + LOOP_INTERVAL;
            if (true == ina.enable_ct) {
                msg.data = ina.send_ct_data;
                if (true == nh.connected()) {
                    chatter.publish(&msg);
                }
            }
        }
        nh.spinOnce();
        vTaskDelay(THREAD_SEEK_INTERVAL_MODEL_M5);
    }
    flag_thread_m5_initialized = false;
}
////////////////////////////////////////////////////////

void setup()
{
    (void)setup_m5();
    unsigned long current = millis();
    cushy.set_callback_handle_client(&handle_client);
    cushy.set_callback_mode(&notify_mode);
    cushy_setup(current);
    sht.setup(current);
    ina.setup(current);
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
    static unsigned long next_time = 0;
    ////////////////////////////////////////////////////////
    unsigned long current = millis();

    if (current > next_time) {
        ////////////////////////////////////////////////////////
        next_time = current + SETTING_TIME_SLEEP_SPLIT;
        sht.loop(current);
        ina.loop(current);
        cushy_loop(current);
        ////////////////////////////////////////////////////////
        loop_m5(current);
        ////////////////////////////////////////////////////////
    }
    (void)delay(1);
}
