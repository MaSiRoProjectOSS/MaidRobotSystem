/**
 * @file  node_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef TEMPLATE_NODE_NODE_IMPLEMENT_HPP
#define TEMPLATE_NODE_NODE_IMPLEMENT_HPP

#include "models/model_implement.hpp"
#include "parameter.hpp"
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float64.hpp>

namespace maid_robot_system
{
class NodeImplement : public rclcpp::Node {
private:
public:
    NodeImplement(int argc, char **argv);
    ~NodeImplement();

private:
    // =============================
    // Variable
    // =============================
    ModelImplement _model;
    std_msgs::msg::Float64 _convert_msg;

private:
    // =============================
    // Function : callback
    // =============================
    void callback_message(const std_msgs::msg::Float64 &msg);

private:
    // =============================
    // ROS
    // =============================
    // Function
    void callback_param();
    void callback_timer();

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pub_value;
    // Subscription
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub_value;

    // Parameter
    std::shared_ptr<rclcpp::ParameterEventHandler> _sub_parameter;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> _handle_param;

    // Interval
    rclcpp::TimerBase::SharedPtr _ros_timer;
    std::shared_ptr<rclcpp::Node> _node_ptr;

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
