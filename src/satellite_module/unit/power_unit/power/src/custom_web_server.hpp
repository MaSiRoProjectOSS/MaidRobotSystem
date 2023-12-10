/**
 * @file custom_web_server.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-04-02
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef CUSTOM_WEB_SERVER_HPP_
#define CUSTOM_WEB_SERVER_HPP_

#include "use_device.hpp"

#include <Arduino.h>
#include <cushy_web_server.hpp>

class CustomWebServer : public CushyWebServer {
public:
    CustomWebServer();

public:
    bool setup();

public:
    bool is_setup = false;
};

#endif /* CUSTOM_WEB_SERVER_HPP_ */
