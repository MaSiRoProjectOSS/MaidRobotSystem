/**
 * @file custom_web_server.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-04-02
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "custom_web_server.hpp"

CustomWebServer::CustomWebServer()
{
}

bool CustomWebServer::setup()
{
    this->is_setup = this->begin();
    return this->is_setup;
}
