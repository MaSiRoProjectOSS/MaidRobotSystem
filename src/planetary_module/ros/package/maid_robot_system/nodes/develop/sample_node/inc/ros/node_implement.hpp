/**
 * @file  node_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef SAMPLE_NODE_IMPLEMENT_HPP
#define SAMPLE_NODE_IMPLEMENT_HPP

#include "models/model_implement.hpp"
#include "rclcpp/rclcpp.hpp"

#include <maid_robot_system_interfaces/msg/mrs_sample.hpp>
#include <std_msgs/msg/float64.hpp>

#ifndef LOGGER_ROS_INFO_CALL_FUNCTION
#define LOGGER_ROS_INFO_CALL_FUNCTION 0
#endif
#ifndef LOGGER_ROS_INFO_PARAMETER
#define LOGGER_ROS_INFO_PARAMETER 0
#endif
#ifndef LOGGER_ROS_INFO_GET_MESSAGE
#define LOGGER_ROS_INFO_GET_MESSAGE 0
#endif
#ifndef LOGGER_ROS_INFO_TIMER
#define LOGGER_ROS_INFO_TIMER 0
#endif

namespace maid_robot_system
{
class NodeImplement : public rclcpp::Node {
private:
public:
    NodeImplement(std::string node_name, int argc, char **argv);
    ~NodeImplement();

private:
    // =============================
    // Variable
    // =============================
    ModelImplement _model;

private:
    // =============================
    // ROS : publisher
    // =============================
    maid_robot_system_interfaces::msg::MrsSample _msg_convert;
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsSample>::SharedPtr _pub_info;

private:
    // =============================
    // ROS : subscription
    // =============================
    void _callback_message(const std_msgs::msg::Float64 &msg);

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub_value;

private:
    // =============================
    // ROS : parameter
    // =============================
    void _callback_param_init();
    OnSetParametersCallbackHandle::SharedPtr _handle_param;

private:
    // =============================
    // ROS : loop function
    // =============================
    void _callback_timer();
    rclcpp::TimerBase::SharedPtr _ros_timer;

private:
    // =============================
    // CONST
    // =============================
    const int CONFIG_QOS               = 255;
    const int CONFIG_SUBSCRIPTION_SIZE = 5;
    const std::chrono::milliseconds TP_MSEC{ 1000 };

    // =============================
    // ROS Topic / Service / Action
    // =============================
    const std::string MRS_TOPIC_OUTPUT = "out";
    const std::string MRS_TOPIC_INPUT  = "in";

    // =============================
    // ROS PARAMETER
    // =============================
    const std::string MRS_PARAMETER_SAMPLE_TIMES  = "param/times";
    const std::string MRS_PARAMETER_SAMPLE_OFFSET = "param/offset";
};

} // namespace maid_robot_system

#endif
