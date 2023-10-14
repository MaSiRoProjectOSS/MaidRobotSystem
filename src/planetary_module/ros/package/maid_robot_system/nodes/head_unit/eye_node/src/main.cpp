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
#include <stdexcept>
#include <thread>
#include <unistd.h>

int main(int argc, char **argv)
{
    std::string node_name   = NODE_NAME;
    std::string widget_name = "widget_";
    widget_name.append(NODE_NAME);
    try {
        maid_robot_system::WidgetNode widget(widget_name, argc, argv);
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor exec;
        auto interaction_node = std::make_shared<maid_robot_system::InteractionNode>(node_name, widget);
        exec.add_node(interaction_node);
        widget.start_exec();
#if 1
        exec.spin();
#else
        std::thread widget_thread([&widget] {
            try {
                do {
                    sleep(0.1);
                } while (true == widget.is_start());
                while (true == widget.is_running()) {
                    sleep(0.001);
                };
            } catch (...) {
            }
        });
        exec.spin();
        widget_thread.join();
#endif
    } catch (char *e) {
        std::cout << e << std::endl;
    } catch (const std::logic_error &l_err) {
        std::cout << l_err.what() << std::endl;
    } catch (const std::runtime_error &r_err) {
        std::cout << r_err.what() << std::endl;
    } catch (...) {
        std::cout << "An unknown error has occurred." << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
