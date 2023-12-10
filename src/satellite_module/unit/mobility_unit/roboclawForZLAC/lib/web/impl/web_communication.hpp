/**
 * @file web_communication.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-12
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef MASIRO_PROJECT_WEB_COMMUNICATION_HPP
#define MASIRO_PROJECT_WEB_COMMUNICATION_HPP

#include "web_manager_connection.hpp"

#include <WebServer.h>

namespace MaSiRoProject
{
namespace Web
{

class WebCommunication {
public:
    WebCommunication();
    ~WebCommunication();

public:
    WebServer *get_server();
    IPAddress get_ip();
    bool setup();
    bool begin();
    bool reconnect();

    bool is_ap_mode();
    bool load_default(bool save);
    bool is_connected(bool immediate = true);
    std::string template_json_result(bool result, std::string data = "", std::string message = "");

public:
    String file_readString(const char *path);

private:
    void handle_not_found();
    void handle_favicon_ico();
    void handle_js_ajax();
    void handle_css_general();

    void handle_network_css();
    void handle_network_js();
    void handle_network_html();

    void handle_network_set();
    void handle_network_get_list();
    void handle_network_get();

private:
    int to_int(String data);
    bool _flag_save = true;

private:
    WebManagerConnection _manager;
    WebServer *ctrl_server;
    const std::string _message_network = "The system was reconfigured."
                                         "<br />"
                                         "Please change the network connection.";
};
} // namespace Web
} // namespace MaSiRoProject
#endif
