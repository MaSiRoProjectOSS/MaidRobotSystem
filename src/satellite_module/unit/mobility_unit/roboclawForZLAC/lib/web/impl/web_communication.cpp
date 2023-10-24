/**
 * @file web_communication.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-12
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "web_communication.hpp"

#include "../web_setting.hpp"
#include "web_default.hpp"

#include <Arduino.h>
#include <SPIFFS.h>

namespace MaSiRoProject
{
namespace Web
{
#define WEB_HEADER_CACHE_CONTROL_SHORT_TIME "max-age=100, immutable"
#define WEB_HEADER_CACHE_CONTROL_LONGTIME   "max-age=31536000, immutable"
#define WEB_HEADER_CACHE_CONTROL_NO_CACHE   "no-cache"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String WebCommunication::file_readString(const char *path)
{
    String word;
    word.clear();
    if (true == SPIFFS.begin()) {
        File file   = SPIFFS.open(path, FILE_READ);
        size_t size = file.size();
        word        = file.readString();
        file.close();
        SPIFFS.end();
    }
    return word;
}

void WebCommunication::handle_not_found()
{
    std::string html = "404 Not Found!";

    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_NO_CACHE);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send(404, "text/plain", html.c_str());
}
void WebCommunication::handle_favicon_ico()
{
    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_LONGTIME);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send_P(200, "image/x-icon", WEB_IMAGE_FAVICON_ICO, sizeof(WEB_IMAGE_FAVICON_ICO));
}
void WebCommunication::handle_js_ajax()
{
    String js = file_readString("/web/public/ajax.js");

    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_LONGTIME);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send(200, "text/javascript; charset=utf-8", js.c_str());
}

void WebCommunication::handle_css_general()
{
    String css = file_readString("/web/public/general.css");

    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_LONGTIME);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send(200, "text/css; charset=utf-8", css.c_str());
}

void WebCommunication::handle_network_css()
{
    String css = file_readString("/web/public/network.css");

    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_LONGTIME);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send(200, "text/css; charset=utf-8", css.c_str());
}
void WebCommunication::handle_network_js()
{
    String js = file_readString("/web/public/network.js");

    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_LONGTIME);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send(200, "text/javascript; charset=utf-8", js.c_str());
}

void WebCommunication::handle_network_html()
{
    String html = file_readString("/web/public/network.html");

    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_SHORT_TIME);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send(200, "text/html; charset=utf-8", html.c_str());
}

void WebCommunication::handle_network_set()
{
    bool result    = false;
    String ssid    = "";
    String pass    = "";
    bool mode_ap   = this->_manager.is_ap_mode();
    bool auto_tran = SETTING_WIFI_MODE_AUTO_TRANSITIONS;

    try {
        if (this->get_server()->args() > 0) {
            if (this->get_server()->hasArg("ap")) {
                int value = this->to_int(this->get_server()->arg("ap"));
                if (value == 1) {
                    mode_ap = true;
                }
                if (this->get_server()->hasArg("id")) {
                    ssid = this->get_server()->arg("id");
                    if (this->get_server()->hasArg("pa")) {
                        pass   = this->get_server()->arg("pa");
                        result = true;
                    }
                }
                if (this->get_server()->hasArg("auto")) {
                    int value = this->to_int(this->get_server()->arg("auto"));
                    auto_tran = (1 == value) ? true : false;
                    result    = true;
                }
            }
        }
    } catch (...) {
        result = false;
    }
    std::string json = this->template_json_result(result);

    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_NO_CACHE);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send(200, "application/json; charset=utf-8", json.c_str());

    if (true == result) {
        this->_flag_save = this->_manager.reconnect(ssid.c_str(), pass.c_str(), mode_ap, auto_tran, this->_flag_save);
    }
}
void WebCommunication::handle_network_get_list()
{
    bool flag_start = true;
    char buffer[255];
    std::vector<WebManagerConnection::NetworkList> items = this->_manager.get_wifi_list();

    std::string data = "[";
    for (WebManagerConnection::NetworkList item : items) {
        if (false == flag_start) {
            data.append(",");
        }
        flag_start = false;
        sprintf(buffer, "{\"name\":\"%s\" , \"power\":%d}", item.name.c_str(), item.rssi);
        data.append(buffer);
    }

    data.append("]");
    std::string json = this->template_json_result(true, data);

    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_NO_CACHE);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send(200, "application/json; charset=utf-8", json.c_str());
}
void WebCommunication::handle_network_get()
{
    bool result      = false;
    bool ap_mode     = this->_manager.is_ap_mode();
    std::string data = "{";
    data.append("\"default\": \"");
    data.append(SETTING_WIFI_SSID_DEFAULT);
    data.append("\", \"name\": \"");
    data.append(this->_manager.get_ssid());
    data.append("\", \"ap_mode\":");
    data.append((true == ap_mode) ? "1" : "0");
    data.append("}");

    std::string json = this->template_json_result(true, data);

    this->get_server()->sendHeader("Location", String("http://") + this->_manager.get_ip().toString(), true);
    this->get_server()->sendHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_NO_CACHE);
    this->get_server()->sendHeader("X-Content-Type-Options", "nosniff");
    this->get_server()->send(200, "application/json; charset=utf-8", json.c_str());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool WebCommunication::setup()
{
    bool result = true;
    if (nullptr != ctrl_server) {
        this->ctrl_server->onNotFound(std::bind(&WebCommunication::handle_not_found, this));
        this->ctrl_server->on("/favicon.ico", std::bind(&WebCommunication::handle_favicon_ico, this));
        this->ctrl_server->on("/ajax.js", std::bind(&WebCommunication::handle_js_ajax, this));
        this->ctrl_server->on("/general.css", std::bind(&WebCommunication::handle_css_general, this));

        this->ctrl_server->on("/network.css", std::bind(&WebCommunication::handle_network_css, this));
        this->ctrl_server->on("/network.js", std::bind(&WebCommunication::handle_network_js, this));
        this->ctrl_server->on("/network", std::bind(&WebCommunication::handle_network_html, this));

        this->ctrl_server->on("/set/network", std::bind(&WebCommunication::handle_network_set, this));
        this->ctrl_server->on("/get/net_list", std::bind(&WebCommunication::handle_network_get_list, this));
        this->ctrl_server->on("/get/network", std::bind(&WebCommunication::handle_network_get, this));
    }
    return result;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
bool WebCommunication::begin()
{
    return this->_manager.begin();
}

bool WebCommunication::reconnect()
{
    return this->_manager.reconnect(this->_flag_save);
}
bool WebCommunication::is_ap_mode()
{
    return this->_manager.is_ap_mode();
}
bool WebCommunication::load_default(bool save)
{
    this->_flag_save = save;
    return this->_manager.reconnect_default(save);
}
bool WebCommunication::is_connected(bool immediate)
{
    return this->_manager.is_connected(immediate);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
WebServer *WebCommunication::get_server()
{
    return ctrl_server;
}
IPAddress WebCommunication::get_ip()
{
    return this->_manager.get_ip();
}
int WebCommunication::to_int(String data)
{
    if (true != data.isEmpty()) {
        int value = std::stoi(data.c_str());
        return value;
    } else {
        return 0;
    }
}
std::string WebCommunication::template_json_result(bool result, std::string data, std::string message)
{
    static bool flag_rand = false;
    static char key[5]    = "-1";
    if (false == flag_rand) {
        sprintf(key, "%d", random(0, 1000));
        flag_rand = true;
    }
    std::string json = "{";
    if (true == result) {
        json.append("\"result\":\"OK\",");
    } else {
        json.append("\"result\":\"NG\",");
    }
    json.append("\"status\":{\"num\": 200, \"messages\":");
    if ("" == message) {
        json.append("\"\"");
    } else {
        json.append(message);
    }
    json.append(", \"KEY\":");
    json.append(key);
    json.append(", \"data\":");
    if ("" == data) {
        json.append("\"\"");
    } else {
        json.append(data);
    }

    json.append("}}");
    return json;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
WebCommunication::WebCommunication()
{
    this->ctrl_server = new WebServer(SETTING_WIFI_PORT);
}
WebCommunication::~WebCommunication()
{
    if (nullptr != this->ctrl_server) {
        this->ctrl_server->close();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace Web
} // namespace MaSiRoProject
