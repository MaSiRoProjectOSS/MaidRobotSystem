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

#include <M5Atom.h>
#include <log_callback.hpp>
#include <web_viewer.hpp>

class WebViewerCustom : public MaSiRoProject::Web::WebViewer {
public:
    WebViewerCustom();

    using LEDFunction = void (*)(CRGB);

    void set_callback_led(LEDFunction callback);
    bool set_callback_message(LogCallback::MessageFunction callback);

    void init();
    void loop();
    void pressed();

protected:
    bool setup_server(WebServer *server) override;

private:
    void _check_state();
    void _led(CRGB color);

private:
    void handle_root_html();
    void handle_get_motor();
    void handle_js_top();
    void handle_css_custom();
    void handle_set_emergency();

    void handle_set_setting();
    void handle_set_save();

private:
    RoboClawForZlac _claw;
    LEDFunction _callback_led;
};

#endif
