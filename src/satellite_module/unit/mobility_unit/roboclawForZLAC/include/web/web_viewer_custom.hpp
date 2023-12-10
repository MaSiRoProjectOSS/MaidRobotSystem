/**
 * @file web_viewer_custom.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-12
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef WEB_VIEWER_CUSTOM_HPP
#define WEB_VIEWER_CUSTOM_HPP

#include "roboclaw_for_zlac.hpp"

#ifndef PIO_UNIT_TESTING
#include <M5Atom.h>
#endif
#include <cushy_web_server.hpp>

//////////////////////////////////////////////////////////

#ifndef SPEED_MPS_FEEDBACK_ID
#define SPEED_MPS_FEEDBACK_ID 500
#endif
#ifndef SPEED_MPS_REQUEST_ID
#define SPEED_MPS_REQUEST_ID 501
#endif

#ifndef CAST_ID
#define CAST_ID 100
#endif

class WebViewerCustom : public CushyWebServer {
public:
    WebViewerCustom();

    using LEDFunction = void (*)(CRGB);

    void set_callback_led(LEDFunction callback);

    void init();
    void loop();
    void pressed();

protected:
    bool setup_server(AsyncWebServer *server) override;
    void handle_favicon_ico(AsyncWebServerRequest *request) override;
    void handle_client();

private:
    void _check_state();
#ifndef PIO_UNIT_TESTING
    void _led(CRGB color);
#endif
    int _set_data(std::vector<FourDimensionalChart::FourDimensionalChartData> *list, int assy, int cnt, std::string *param);
    void _data_saved(std::vector<FourDimensionalChart::FourDimensionalChartData> *list);
    char *_make_row_text(FourDimensionalChart::FourDimensionalChartData csd, int assy, int cast_id = CAST_ID);

private:
    void handle_root_html(AsyncWebServerRequest *request);
    void handle_get_motor(AsyncWebServerRequest *request);
    void handle_js_top(AsyncWebServerRequest *request);
    void handle_css_custom(AsyncWebServerRequest *request);
    void handle_set_emergency(AsyncWebServerRequest *request);
    void handle_set_free_motor(AsyncWebServerRequest *request);

    void handle_set_setting(AsyncWebServerRequest *request);
    void handle_set_save(AsyncWebServerRequest *request);

private:
    RoboClawForZlac _claw;
    LEDFunction _callback_led;
};

#endif
