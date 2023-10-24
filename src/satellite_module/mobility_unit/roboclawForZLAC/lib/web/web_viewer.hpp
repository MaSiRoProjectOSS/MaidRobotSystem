/**
 * @file web_viewer.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-12
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef MASIRO_PROJECT_WEB_VIEWER_HPP
#define MASIRO_PROJECT_WEB_VIEWER_HPP

#include <WebServer.h>
#include <string.h>

namespace MaSiRoProject
{
namespace Web
{
class WebViewer {
public:
    enum WEB_VIEWER_MODE
    {
        NOT_INITIALIZED,
        INITIALIZED,
        DISCONNECTED,
        RETRY,
        CONNECTED_STA,
        CONNECTED_AP,
    };

    using ModeFunction = void (*)(WEB_VIEWER_MODE);

public:
    WebViewer();
    ~WebViewer();

public:
    bool begin();

#if DEBUG_MODE
    /////////////////////////////////////////
    // get  member valuable
    /////////////////////////////////////////
public:
    UBaseType_t get_stack_size();
    UBaseType_t get_stack_high_water_mark();
#endif

public:
    void set_callback_mode(ModeFunction callback);
    WEB_VIEWER_MODE get_mode();

protected:
    virtual bool setup_server(WebServer *server);
    WebServer *get_server();
    IPAddress get_ip();

    String ip_to_string(IPAddress ip);
    byte to_byte(String data);
    int to_int(String data);
    unsigned long to_ulong(String data);
    String file_readString(const char *path);
    std::string template_json_result(bool result, std::string data = "", std::string message = "");

private:
    TaskHandle_t _task_handle;

private:
    const UBaseType_t TASK_ASSIGNED_SIZE = (4096 * 2);
};

} // namespace Web
} // namespace MaSiRoProject
#endif
