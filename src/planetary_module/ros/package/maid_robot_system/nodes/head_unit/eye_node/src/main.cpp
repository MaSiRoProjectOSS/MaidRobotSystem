/**
 * @file main.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "ros/interaction_node.hpp"
#include "ros/widget_node.hpp"

#include <exception>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <thread>
#include <unistd.h>

int main(int argc, char **argv)
{
    std::string node_name   = NODE_NAME;
    std::string widget_name = "widget_";
    widget_name.append(NODE_NAME);
    std::string logger_name = "logger_";
    logger_name.append(NODE_NAME);

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr log_node = rclcpp::Node::make_shared(logger_name);
    try {
        RCLCPP_INFO(log_node->get_logger(), "<%s> %s", node_name.c_str(), "start");
        maid_robot_system::WidgetNode widget(widget_name, argc, argv);
        auto interaction_node = std::make_shared<maid_robot_system::InteractionNode>(node_name, widget);
        widget.exec_start();

        std::thread ros_thread([&interaction_node, &widget] {
            try {
                rclcpp::executors::MultiThreadedExecutor exec;
                exec.add_node(interaction_node);
                exec.spin();
                widget.closing();
            } catch (...) {
            }
        });
        widget.running_exec();
        ros_thread.join();
        RCLCPP_INFO(log_node->get_logger(), "<%s> %s", node_name.c_str(), "end");
    } catch (char *e) {
        RCLCPP_ERROR(log_node->get_logger(), "<ERROR> %s", e);
    } catch (const std::logic_error &l_err) {
        RCLCPP_ERROR(log_node->get_logger(), "<LOGIC_ERROR> %s", l_err.what());
    } catch (const std::runtime_error &r_err) {
        RCLCPP_ERROR(log_node->get_logger(), "<RUNTIME_ERROR> %s", r_err.what());
    } catch (...) {
        RCLCPP_ERROR(log_node->get_logger(), "<ERROR> %s", "An unknown error has occurred.");
    }
    rclcpp::shutdown();
    return 0;
}
