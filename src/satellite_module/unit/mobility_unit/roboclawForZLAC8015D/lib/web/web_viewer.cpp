/**
 * @file web_viewer.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-12
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "web_viewer.hpp"

#include "impl/web_communication.hpp"
#include "web_setting.hpp"

#include <Arduino.h>

namespace MaSiRoProject
{
namespace Web
{
WebCommunication ctrl_web;

#pragma region thread_fuction
///////////////////////////////////////////////////////////////////////////////////////////////

const char *THREAD_NAME_WIFI         = "ThreadWiFi";
const int THREAD_SEEK_INTERVAL_WIFI  = (10 * 1000);
const int THREAD_RETRY_INTERVAL_WIFI = (5 * 1000);
const int THREAD_INTERVAL_WIFI       = (25);

volatile bool flag_thread_wifi_initialized = false;
volatile bool flag_thread_wifi_fin         = false;
WebViewer::WEB_VIEWER_MODE _mode           = WebViewer::WEB_VIEWER_MODE::NOT_INITIALIZED;
WebViewer::ModeFunction _callback_mode;

void set_mode(WebViewer::WEB_VIEWER_MODE mode)
{
    if (_mode != mode) {
        _mode = mode;
        if (nullptr != _callback_mode) {
            _callback_mode(mode);
        }
    }
}

void thread_wifi(void *args)
{
#if SETTING_WIFI_MODE_AUTO_TRANSITIONS
    unsigned long err_begin = millis() + SETTING_WIFI_AUTO_TRANSITIONS_DEFAULT_TIMEOUT;
#endif
    flag_thread_wifi_fin = false;
    set_mode(WebViewer::WEB_VIEWER_MODE::NOT_INITIALIZED);

    while (false == ctrl_web.begin()) {
        if (true == flag_thread_wifi_fin) {
            break;
        }
        vTaskDelay(THREAD_SEEK_INTERVAL_WIFI);
    }
    set_mode(WebViewer::WEB_VIEWER_MODE::INITIALIZED);

    while (false == flag_thread_wifi_fin) {
        try {
            if (false == ctrl_web.reconnect()) {
                set_mode(WebViewer::WEB_VIEWER_MODE::RETRY);
#if SETTING_WIFI_MODE_AUTO_TRANSITIONS
                if (true != ctrl_web.is_ap_mode()) {
                    if (err_begin < millis()) {
                        err_begin = millis() + SETTING_WIFI_AUTO_TRANSITIONS_DEFAULT_TIMEOUT;
                        ctrl_web.load_default(false);
                    }
                }
#endif
            } else {
                set_mode((true == ctrl_web.is_ap_mode()) ? //
                                 WebViewer::WEB_VIEWER_MODE::CONNECTED_AP :
                                 WebViewer::WEB_VIEWER_MODE::CONNECTED_STA);
                if (nullptr != ctrl_web.get_server()) {
                    ctrl_web.get_server()->begin();
                    while (false == flag_thread_wifi_fin) {
                        ctrl_web.get_server()->handleClient();
                        if (true != ctrl_web.is_connected()) {
                            break;
                        }
                        vTaskDelay(THREAD_INTERVAL_WIFI);
                    }
                    ctrl_web.get_server()->close();
                }
                set_mode(WebViewer::WEB_VIEWER_MODE::DISCONNECTED);
            }
            vTaskDelay(THREAD_RETRY_INTERVAL_WIFI);
        } catch (...) {
        }
    }
    flag_thread_wifi_initialized = false;
}
///////////////////////////////////////////////////////////////////////////////////////////////
#pragma endregion

///////////////////////////////////////////////////////////////////////////////////////////////
WebViewer::WebViewer()
{
}
WebViewer::~WebViewer()
{
    flag_thread_wifi_fin = true;
}

bool WebViewer::begin()
{
    bool result = false;
    try {
        ////////////////////////////////////////////////////////
        if (true == ctrl_web.setup()) {
            if (true == this->setup_server(ctrl_web.get_server())) {
                if (false == flag_thread_wifi_initialized) {
                    flag_thread_wifi_initialized = true;
                    xTaskCreatePinnedToCore(thread_wifi, //
                                            THREAD_NAME_WIFI,
                                            this->TASK_ASSIGNED_SIZE,
                                            NULL,
                                            SETTING_THREAD_PRIORITY,
                                            &this->_task_handle,
                                            SETTING_THREAD_CORE_WIFI);
                }
                result = true;
            }
        }

        ////////////////////////////////////////////////////////
    } catch (...) {
    }
    return result;
}

bool WebViewer::setup_server(WebServer *server)
{
    return true;
}
WebServer *WebViewer::get_server()
{
    return ctrl_web.get_server();
}
IPAddress WebViewer::get_ip()
{
    return ctrl_web.get_ip();
}

void WebViewer::set_callback_mode(ModeFunction callback)
{
    _callback_mode = callback;
}
WebViewer::WEB_VIEWER_MODE WebViewer::get_mode()
{
    return _mode;
}
/////////////////////////////////
// value conversion
/////////////////////////////////
String WebViewer::ip_to_string(IPAddress ip)
{
    String res = "";
    for (int i = 0; i < 3; i++) {
        res += String((ip >> (8 * i)) & 0xFF) + ".";
    }
    res += String(((ip >> 8 * 3)) & 0xFF);
    return res;
}
byte WebViewer::to_byte(String data)
{
    if (true != data.isEmpty()) {
        int value = std::stoi(data.c_str());
        return (byte)(value);
    } else {
        return 0;
    }
}
unsigned long WebViewer::to_ulong(String data)
{
    if (true != data.isEmpty()) {
        unsigned long value = std::stoul(data.c_str());
        return value;
    } else {
        return 0;
    }
}
int WebViewer::to_int(String data)
{
    if (true != data.isEmpty()) {
        int value = std::stoi(data.c_str());
        return value;
    } else {
        return 0;
    }
}
String WebViewer::file_readString(const char *path)
{
    return ctrl_web.file_readString(path);
}
std::string WebViewer::template_json_result(bool result, std::string data, std::string message)
{
    return ctrl_web.template_json_result(result, data, message);
}

/////////////////////////////////////////
// get  member valuable
/////////////////////////////////////////
#if DEBUG_MODE
UBaseType_t WebViewer::get_stack_size()
{
    return this->TASK_ASSIGNED_SIZE;
}
UBaseType_t WebViewer::get_stack_high_water_mark()
{
    return uxTaskGetStackHighWaterMark(this->_task_handle);
}
#endif
} // namespace Web
} // namespace MaSiRoProject
