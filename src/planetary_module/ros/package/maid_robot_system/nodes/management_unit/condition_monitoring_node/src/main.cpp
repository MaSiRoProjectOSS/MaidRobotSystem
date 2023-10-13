/**
 * @file main.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "ros/node_implement.hpp"

#include <exception>
#include <stdexcept>

int main(int argc, char **argv)
{
    std::string node_name = NODE_NAME;
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<maid_robot_system::NodeImplement>( //
                node_name,
                argc,
                argv));
    } catch (...) {
    }
    rclcpp::shutdown();
    return 0;
}
