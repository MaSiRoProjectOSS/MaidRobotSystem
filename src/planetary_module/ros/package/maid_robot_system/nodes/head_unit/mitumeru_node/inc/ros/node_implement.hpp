/**
 * @file  node_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_NODE_MITUMERU_NODE_IMPLEMENT_HPP
#define MRS_NODE_MITUMERU_NODE_IMPLEMENT_HPP

#include "models/model_implement.hpp"
#include "parameter.hpp"
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float64.hpp>

namespace maid_robot_system
{
class NodeImplement : public rclcpp::Node {
private:
public:
    NodeImplement(std::string package_name, std::string node_name, int argc, char **argv);
    ~NodeImplement();

private:
    // =============================
    // Variable
    // =============================
    ModelImplement _model;
    std_msgs::msg::Float64 _convert_msg;

private:
    // =============================
    // ROS : publisher
    // =============================
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pub_value;

private:
    // =============================
    // ROS : subscription
    // =============================
    void callback_message(const std_msgs::msg::Float64 &msg);

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub_value;

private:
    // =============================
    // ROS : parameter
    // =============================
    void callback_param();
    std::shared_ptr<rclcpp::ParameterEventHandler> _sub_parameter;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> _handle_param1;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> _handle_param2;

private:
    // =============================
    // ROS : loop function
    // =============================
    void callback_timer();
    rclcpp::TimerBase::SharedPtr _ros_timer;

    // Parameter

private:
    // =============================
    // ROS : information
    // =============================
    std::string _package_name = "";
    std::string _node_name    = "";

private:
    // =============================
    // CONST
    // =============================
    const int CONFIG_QOS               = 255;
    const int CONFIG_SUBSCRIPTION_SIZE = 5;
    const std::chrono::milliseconds TP_MSEC{ 1000 };
};

} // namespace maid_robot_system

#endif
