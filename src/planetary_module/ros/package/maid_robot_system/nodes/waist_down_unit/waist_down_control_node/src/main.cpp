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
    std::string node_name   = NODE_NAME;
    std::string logger_name = "logger_";
    logger_name.append(NODE_NAME);

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr log_node = rclcpp::Node::make_shared(logger_name);
    try {
        rclcpp::spin(std::make_shared<maid_robot_system::NodeImplement>( //
                node_name,
                argc,
                argv));
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
