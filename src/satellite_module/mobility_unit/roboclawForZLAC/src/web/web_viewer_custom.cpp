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
#include "web/web_viewer_custom.hpp"

#include "web_viewer_data.hpp"

#include <Arduino.h>
#include <SPIFFS.h>

#define WEB_HEADER_CACHE_CONTROL_SHORT_TIME "max-age=100, immutable"
#define WEB_HEADER_CACHE_CONTROL_LONGTIME   "max-age=31536000, immutable"
#define WEB_HEADER_CACHE_CONTROL_NO_CACHE   "no-cache"

///////////////////////////////////////////////////////////////////////////////////////////

char *WebViewerCustom::_make_row_text(FourDimensionalChart::FourDimensionalChartData csd, int assy, int cast_id)
{
    static char buffer[1024];
    sprintf(buffer,
            "{"
            "\"assy\": %d,"
            "\"cast\": %d,"
            "\"x\": %0.3f,"
            "\"y\": %0.3f,"
            "\"utc\": %ld"
            "}",
            assy,
            cast_id,
            csd.data.x,
            csd.data.y,
            this->millis_to_time(csd.data.time_ms));
    return buffer;
}

void WebViewerCustom::_data_saved(std::vector<FourDimensionalChart::FourDimensionalChartData> *list)
{
    for (int i = 0; i < list->size(); i++) {
        list->at(i).save();
    }
}

int WebViewerCustom::_set_data(std::vector<FourDimensionalChart::FourDimensionalChartData> *list, int assy, int cnt, std::string *param)
{
    static int MAX_SIZE = 100;
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
            param->append(this->_make_row_text(list->at(i), assy));
            cnt++;
        }
    }
    return cnt;
}
void WebViewerCustom::handle_client()
{
    static unsigned long next_time            = 0;
    static int SETTING_LOOP_TIME_SLEEP_DETECT = (20 * 1000);
    unsigned long current                     = millis();
    ////////////////////////////////////////////////////////

    int cnt           = 0;
    std::string param = "";
#if SETTING_URL_ENDPOINT
    if (current > next_time) {
        next_time = current + SETTING_LOOP_TIME_SLEEP_DETECT;
        if (true == this->is_sntp_sync()) {
            param.append("{\"data\":[");

            cnt = this->_set_data(this->_claw.speed_mps_feedback()->get_list(), SPEED_MPS_FEEDBACK_ID, cnt, &param);
            cnt = this->_set_data(this->_claw.speed_mps_request()->get_list(), SPEED_MPS_REQUEST_ID, cnt, &param);

            param.append("]}");
            if (0 < cnt) {
                StaticJsonDocument<512> replay;
                if (true == this->post_json(SETTING_URL_ENDPOINT, param.c_str(), &replay)) {
                    if ("OK" == replay["result"]) {
                        this->_data_saved(this->_claw.speed_mps_feedback()->get_list());
                        this->_data_saved(this->_claw.speed_mps_request()->get_list());

                        this->_claw.set_log_update();
#if DEBUG_TRACE
                        const char *txt_result = replay["result"];
                        int code               = replay["status"]["code"];
                        log_v("<%lu>%s [code:%d][cnt:%d]", (millis() - current), txt_result, code, cnt);
                    } else {
                        const char *message = replay["status"]["messages"];
                        int code            = replay["status"]["code"];
                        log_d("<%lu>%s [code:%d][cnt:%d]", (millis() - current), message, code, cnt);
                        log_d("PARAM : ", param.c_str());
#endif
                    }
                } else {
                    log_w("Send failed.");
                }
            } else {
                log_v("No data.");
            }
        }
    }
#endif
}

WebViewerCustom::WebViewerCustom()
{
    this->set_callback_handle_client( //
            std::bind(&WebViewerCustom::handle_client, this));
}
///////////////////////////////////////////////////////////////////////////////////////////
void WebViewerCustom::handle_root_html(AsyncWebServerRequest *request)
{
    String html = this->file_readString("/web/public/root.html");

    AsyncWebServerResponse *response = request->beginResponse(200, "text/html; charset=utf-8", html.c_str());
    response->addHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_SHORT_TIME);
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);
}
void WebViewerCustom::handle_set_emergency(AsyncWebServerRequest *request)
{
    bool result         = false;
    bool flag_emergency = true;
    try {
        if (request->args() > 0) {
            if (true == request->hasArg("off")) {
                flag_emergency = false;
            }
        }
        result = true;
    } catch (...) {
        result = false;
    }

    std::string json = this->template_json_result(result);

    AsyncWebServerResponse *response = request->beginResponse(200, "application/json; charset=utf-8", json.c_str());
    response->addHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_NO_CACHE);
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);

    if (true == result) {
        this->_claw.set_emergency(flag_emergency);
        this->_check_state();
    }
}
void WebViewerCustom::handle_set_free_motor(AsyncWebServerRequest *request)
{
    bool result         = false;
    bool flag_emergency = true;
    try {
        if (request->args() > 0) {
            if (true == request->hasArg("off")) {
                flag_emergency = false;
            }
        }
        result = true;
    } catch (...) {
        result = false;
    }

    std::string json = this->template_json_result(result);

    AsyncWebServerResponse *response = request->beginResponse(200, "application/json; charset=utf-8", json.c_str());
    response->addHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_NO_CACHE);
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);

    if (true == result) {
        this->_claw.set_motor_free(flag_emergency);
        this->_check_state();
    }
}
void WebViewerCustom::handle_get_motor(AsyncWebServerRequest *request)
{
    bool result = false;
    char buffer[255];
    ZLAC706Serial::zlac_info info = this->_claw.get_zlac_info();

    std::string data = "{";
    ////////////////////////////////
    data.append("\"left\": ");
    sprintf(buffer,
            "["                                           //
            "[%0.3f,%d,%d,%d,%d,%d,%ld,%ld,%d,%ld,%0.3f]" //
            ",[%d,%d,%d,%d,%d,%d,%d,%d,%d]"               //
            ,                                             //

            (float)(info.left.update_time) / 1000.0f, // l_it
            info.left.current,                        // l_ic
            info.left.voltage,                        // l_iv
            info.left.torque_request_mA,              // l_tr
            info.left.speed_rpm,                      // l_sc
            info.left.speed_request_rpm,              // l_sr
            info.left.position_request,               // l_pr
            info.left.position_given,                 // l_pg
            info.left.position_rpm,                   // l_ps
            info.left.position_feedback,              // l_pf
            info.left.position_feedback_deg,          // l_pfd
                                                      //
            (info.left.error.stop_state ? 1 : 0),     //
            (info.left.error.startup_state ? 1 : 0),  //
            (info.right.error.over_current ? 1 : 0),  //
            (info.left.error.over_voltage ? 1 : 0),   //
            (info.left.error.under_voltage ? 1 : 0),  //
            (info.left.error.overheat ? 1 : 0),       //
            (info.left.error.overload ? 1 : 0),       //
            (info.left.error.encoder_error ? 1 : 0),  //
            (info.left.error.not_connection ? 1 : 0)  //
    );
    data.append(buffer);

    switch (info.mode) {
        case ZLAC706Serial::DRIVER_MODE::POSITION_FROM_PULSE:
        case ZLAC706Serial::DRIVER_MODE::POSITION_FROM_DIGITAL:
        case ZLAC706Serial::DRIVER_MODE::POSITION_FROM_ANALOG:
            sprintf(buffer,
                    ",[%u,%u,%u,%u]" //
                    ,                //

                    info.left.position_proportional_gain, // s_pkp
                    0u,                                   //
                    info.left.position_differential_gain, // s_pkd
                    info.left.position_feed_forward_gain  // s_pkf

            );
            break;
        case ZLAC706Serial::DRIVER_MODE::SPEED_FROM_DIGITAL:
        case ZLAC706Serial::DRIVER_MODE::SPEED_FROM_ANALOG:
            sprintf(buffer,
                    ",[%u,%u,%u,%u]" //
                    ,                //

                    info.left.speed_proportional_gain, // s_skp
                    info.left.speed_integral_gain,     // s_ski
                    info.left.speed_differential_gain, // s_skd
                    0u                                 //

            );
            break;
        case ZLAC706Serial::DRIVER_MODE::TORQUE_FROM_DIGITAL:
        case ZLAC706Serial::DRIVER_MODE::TORQUE_FROM_ANALOG:
            sprintf(buffer,
                    ",[%u,%u,%u,%u]" //
                    ,                //

                    0u, // kp
                    0u, // ki
                    0u, // kd
                    0u  // kf
            );
            break;

        default:
            sprintf(buffer,
                    ",[%u,%u,%u,%u]" //
                    ,                //

                    0u, // kp
                    0u, // ki
                    0u, // kd
                    0u  // kf
            );
            break;
    }

    data.append(buffer);

    data.append("]");
    ////////////////////////////////
    data.append(", \"right\": ");
    sprintf(buffer,
            "["                                           //
            "[%0.3f,%d,%d,%d,%d,%d,%ld,%ld,%d,%ld,%0.3f]" //
            ",[%d,%d,%d,%d,%d,%d,%d,%d,%d]"               //
            ,                                             //

            (float)(info.right.update_time) / 1000.0f, //
            info.right.current,                        //
            info.right.voltage,                        //
            info.right.torque_request_mA,              //
            info.right.speed_rpm,                      //
            info.right.speed_request_rpm,              //
            info.right.position_request,               //
            info.right.position_given,                 //
            info.right.position_rpm,                   //
            info.right.position_feedback,              //
            info.right.position_feedback_deg,          //
                                                       //
            (info.right.error.stop_state ? 1 : 0),     //
            (info.right.error.startup_state ? 1 : 0),  //
            (info.right.error.over_current ? 1 : 0),   //
            (info.right.error.over_voltage ? 1 : 0),   //
            (info.right.error.under_voltage ? 1 : 0),  //
            (info.right.error.overheat ? 1 : 0),       //
            (info.right.error.overload ? 1 : 0),       //
            (info.right.error.encoder_error ? 1 : 0),  //
            (info.right.error.not_connection ? 1 : 0)  //
    );
    data.append(buffer);

    switch (info.mode) {
        case ZLAC706Serial::DRIVER_MODE::POSITION_FROM_PULSE:
        case ZLAC706Serial::DRIVER_MODE::POSITION_FROM_DIGITAL:
        case ZLAC706Serial::DRIVER_MODE::POSITION_FROM_ANALOG:
            sprintf(buffer,
                    ",[%u,%u,%u,%u]" //
                    ,                //

                    info.right.position_proportional_gain, // s_pkp
                    0u,                                    //
                    info.right.position_differential_gain, // s_pkd
                    info.right.position_feed_forward_gain  // s_pkf

            );
            break;
        case ZLAC706Serial::DRIVER_MODE::SPEED_FROM_DIGITAL:
        case ZLAC706Serial::DRIVER_MODE::SPEED_FROM_ANALOG:
            sprintf(buffer,
                    ",[%u,%u,%u,%u]" //
                    ,                //

                    info.right.speed_proportional_gain, // s_skp
                    info.right.speed_integral_gain,     // s_ski
                    info.right.speed_differential_gain, // s_skd
                    0u                                  //

            );
            break;
        case ZLAC706Serial::DRIVER_MODE::TORQUE_FROM_DIGITAL:
        case ZLAC706Serial::DRIVER_MODE::TORQUE_FROM_ANALOG:
            sprintf(buffer,
                    ",[%u,%u,%u,%u]" //
                    ,                //

                    0u, // kp
                    0u, // ki
                    0u, // kd
                    0u  // kf
            );
            break;

        default:
            sprintf(buffer,
                    ",[%u,%u,%u,%u]" //
                    ,                //

                    0u, // kp
                    0u, // ki
                    0u, // kd
                    0u  // kf
            );
            break;
    }

    data.append(buffer);

    data.append("]");
    ////////////////////////////////
    data.append(", \"both\": ");
    sprintf(buffer,
            "["                               //
            "[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]" //
            ",[%d,%d,%d]"                     //
            ,                                 //

            (int)info.mode,                 // c_mo
            info.position_absolute,         // c_pa
            (info.flag.running ? 1 : 0),    // c_fr
            (info.flag.emergency ? 1 : 0),  //
            (info.left.interval ? 1 : 0),   //
            (info.right.interval ? 1 : 0),  //
            info.acceleration_ms,           // c_sa
            info.deceleration_ms,           // c_sd
            info.SPEED_LIMIT,               // c_sm
            (info.flag.motor_free ? 1 : 0), //

            info.rated_current_mW, // c_ma
            info.POSITION_LIMIT,   // c_pl
            info.TORQUE_LIMIT      // c_tm
    );
    data.append(buffer);

    data.append("]");
    ////////////////////////////////
    data.append(", \"log\": [[");
    for (int i = 0; i < info.direct.length; i++) {
        sprintf(buffer,
                "%s[%0.3f" //
                ,          //

                ((0 == i) ? "" : ","),                      //
                (float)(info.direct.data[i].time) / 1000.0f //
        );
        data.append(buffer);

        sprintf(buffer,
                ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]" //
                ,                                          //

                (int)info.direct.data[i].a1,  //
                (int)info.direct.data[i].a2,  //
                (int)info.direct.data[i].a3,  //
                (int)info.direct.data[i].v00, //
                (int)info.direct.data[i].v01, //
                (int)info.direct.data[i].v02, //
                (int)info.direct.data[i].v03, //
                (int)info.direct.data[i].v04, //
                (int)info.direct.data[i].v05, //
                (int)info.direct.data[i].v06, //
                (int)info.direct.data[i].v07, //
                (int)info.direct.data[i].v08, //
                (int)info.direct.data[i].v09  //
        );
        data.append(buffer);
    }
    ////////////////////////////////
    data.append("],[");
    for (int i = 0; i < info.left.order.length; i++) {
        sprintf(buffer,
                "%s[%0.3f,%d,%d,%d]" //
                ,                    //

                ((0 == i) ? "" : ","),                           //
                (float)(info.left.order.data[i].time) / 1000.0f, //

                (int)info.left.order.data[i].a1, //
                (int)info.left.order.data[i].a2, //
                (int)info.left.order.data[i].a3  //
        );
        data.append(buffer);
    }
    ////////////////////////////////
    data.append("],[");
    for (int i = 0; i < info.right.order.length; i++) {
        sprintf(buffer,
                "%s[%0.3f,%d,%d,%d]" //
                ,                    //

                ((0 == i) ? "" : ","),                            //
                (float)(info.right.order.data[i].time) / 1000.0f, //

                (int)info.right.order.data[i].a1, //
                (int)info.right.order.data[i].a2, //
                (int)info.right.order.data[i].a3  //

        );
        data.append(buffer);
    }
    ////////////////////////////////
    data.append("],[");
    for (int i = 0; i < info.system.length; i++) {
        sprintf(buffer,
                "%s[%0.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]" //
                ,                                            //

                ((0 == i) ? "" : ","),                       //
                (float)(info.system.data[i].time) / 1000.0f, //

                (int)info.system.data[i].a1,  //
                (int)info.system.data[i].a2,  //
                (int)info.system.data[i].a3,  //
                (int)info.system.data[i].v00, //
                (int)info.system.data[i].v01, //
                (int)info.system.data[i].v02, //
                (int)info.system.data[i].v03, //
                (int)info.system.data[i].v04, //
                (int)info.system.data[i].v05, //
                (int)info.system.data[i].v06, //
                (int)info.system.data[i].v07  //
        );
        data.append(buffer);
    }
    ////////////////////////////////
    data.append("]");
    ////////////////////////////////
    data.append("]}");
    std::string json = this->template_json_result(true, data);

    AsyncWebServerResponse *response = request->beginResponse(200, "application/json; charset=utf-8", json.c_str());
    response->addHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_NO_CACHE);
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);
}
void WebViewerCustom::handle_js_top(AsyncWebServerRequest *request)
{
    String js = this->file_readString("/web/public/top.js");

    AsyncWebServerResponse *response = request->beginResponse(200, "text/javascript; charset=utf-8", js.c_str());
    response->addHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_LONGTIME);
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);
}
void WebViewerCustom::handle_css_custom(AsyncWebServerRequest *request)
{
    String css = this->file_readString("/web/public/custom.css");

    AsyncWebServerResponse *response = request->beginResponse(200, "text/css; charset=utf-8", css.c_str());
    response->addHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_LONGTIME);
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);
}
void WebViewerCustom::handle_set_setting(AsyncWebServerRequest *request)
{
    bool result = true;
    int value;
    try {
        if (request->args() > 0) {
            ZLAC706Serial::DRIVER_TARGET target = ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_ALL;
            if (true == request->hasArg("le")) {
                value = this->to_int(request->arg("le"));
                if (1 == value) {
                    target = ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_LEFT;
                }
            }
            if (true == request->hasArg("re")) {
                value = this->to_int(request->arg("re"));
                if (1 == value) {
                    target = ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_RIGHT;
                }
            }

            if (true == request->hasArg("kp")) {
                value  = this->to_int(request->arg("kp"));
                result = this->_claw.setting_proportional_gain(target, value);
            }
            if (true == request->hasArg("ki")) {
                value  = this->to_int(request->arg("ki"));
                result = this->_claw.setting_integral_gain(target, value);
            }
            if (true == request->hasArg("kd")) {
                value  = this->to_int(request->arg("kd"));
                result = this->_claw.setting_differential_gain(target, value);
            }
            if (true == request->hasArg("kf")) {
                value  = this->to_int(request->arg("kf"));
                result = this->_claw.setting_feed_forward_gain(target, value);
            }
            if (true == request->hasArg("in")) {
                value  = this->to_int(request->arg("in"));
                result = this->_claw.setting_inverted(target, ((1 == value) ? true : false));
            }
            if (true == request->hasArg("acc")) {
                value  = this->to_int(request->arg("acc"));
                result = this->_claw.setting_acc(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_ALL, value);
            }
            if (true == request->hasArg("dcc")) {
                value  = this->to_int(request->arg("dcc"));
                result = this->_claw.setting_dcc(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_ALL, value);
            }
            if (true == request->hasArg("limit")) {
                value  = this->to_int(request->arg("limit"));
                result = this->_claw.setting_limit(ZLAC706Serial::DRIVER_TARGET::DRIVER_TARGET_ALL, value);
            }
        }
    } catch (...) {
        result = false;
    }

    std::string json = this->template_json_result(result);

    AsyncWebServerResponse *response = request->beginResponse(200, "application/json; charset=utf-8", json.c_str());
    response->addHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_NO_CACHE);
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);
}
void WebViewerCustom::handle_set_save(AsyncWebServerRequest *request)
{
    bool result = this->_claw.setting_save();

    std::string json                 = this->template_json_result(result);
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json; charset=utf-8", json.c_str());
    response->addHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_NO_CACHE);
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);
}
void WebViewerCustom::handle_favicon_ico(AsyncWebServerRequest *request)
{
    AsyncWebServerResponse *response = request->beginResponse_P(200, "image/x-icon", WEB_IMAGE_FAVICON_ICO, WEB_IMAGE_FAVICON_ICO_LEN);
    response->addHeader("Cache-Control", WEB_HEADER_CACHE_CONTROL_LONGTIME);
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);
}
///////////////////////////////////////////////////////////////////////////////////////////
bool WebViewerCustom::setup_server(AsyncWebServer *server)
{
    server->on("/", std::bind(&WebViewerCustom::handle_root_html, this, std::placeholders::_1));
    server->on("/get/motor", std::bind(&WebViewerCustom::handle_get_motor, this, std::placeholders::_1));
    server->on("/top.js", std::bind(&WebViewerCustom::handle_js_top, this, std::placeholders::_1));
    server->on("/custom.css", std::bind(&WebViewerCustom::handle_css_custom, this, std::placeholders::_1));
    server->on("/set/stop", std::bind(&WebViewerCustom::handle_set_emergency, this, std::placeholders::_1));
    server->on("/set/free_motor", std::bind(&WebViewerCustom::handle_set_free_motor, this, std::placeholders::_1));
    server->on("/set/emergency", std::bind(&WebViewerCustom::handle_set_emergency, this, std::placeholders::_1));

    server->on("/set/setting", std::bind(&WebViewerCustom::handle_set_setting, this, std::placeholders::_1));
    server->on("/set/save", std::bind(&WebViewerCustom::handle_set_save, this, std::placeholders::_1));
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
void WebViewerCustom::set_callback_led(LEDFunction callback)
{
    this->_callback_led = callback;
}

#ifndef PIO_UNIT_TESTING
void WebViewerCustom::_led(CRGB color)
{
    if (nullptr != this->_callback_led) {
        this->_callback_led(color);
    }
}
#endif
void WebViewerCustom::_check_state()
{
    switch (this->_claw.get_state()) {
        case RoboClawForZlac::roboclaw_state::STATE_NOT_INITIALIZED:
            this->_led(CRGB::DarkCyan);
            break;
        case RoboClawForZlac::roboclaw_state::STATE_NOT_SETTING:
            this->_led(CRGB::Blue);
            break;
        case RoboClawForZlac::roboclaw_state::STATE_RUNNING:
            this->_led(CRGB::Green);
            break;
        case RoboClawForZlac::roboclaw_state::STATE_EMERGENCY:
            this->_led(CRGB::Yellow);
            break;
        default:
            this->_led(CRGB::Red);
            break;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
void WebViewerCustom::init()
{
    ////////////////////////////////////////////////////////
    (void)this->_claw.setup(&Serial, &Serial2, &Serial1, BOARD_INPUT_BAUDRATE);
    (void)this->_claw.update_id();
    ////////////////////////////////////////////////////////
    if (true == this->_claw.begin()) {
        this->_check_state();
    } else {
        this->_led(CRGB::Black);
    }
    this->begin();
}

void WebViewerCustom::loop()
{
    ////////////////////////////////////////////////////////
    if (true == this->_claw.loop()) {
        this->_check_state();
    } else {
        this->_led(CRGB::Red);
    }
    ////////////////////////////////////////////////////////
}

void WebViewerCustom::pressed()
{
#if 0
    this->_claw.set_emergency(true);
    this->_check_state();
#else
    // TODO: to Free mode
    if (true == this->_claw.reset()) {
        switch (this->_claw.get_state()) {
            case RoboClawForZlac::roboclaw_state::STATE_NOT_INITIALIZED:
                this->_led(CRGB::Yellow);
                break;
            case RoboClawForZlac::roboclaw_state::STATE_NOT_SETTING:
                this->_led(CRGB::Blue);
                break;
            case RoboClawForZlac::roboclaw_state::STATE_RUNNING:
                this->_led(CRGB::Green);
                break;
            default:
                this->_led(CRGB::Red);
                break;
        }
    } else {
        this->_led(CRGB::Black);
    }
#endif
}
