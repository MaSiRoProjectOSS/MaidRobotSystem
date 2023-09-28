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
#include <std_msgs/msg/string.hpp>

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
    std_msgs::msg::String _msg_text;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_text;

private:
    // =============================
    // ROS : parameter
    // =============================
    void callback_param();
    OnSetParametersCallbackHandle::SharedPtr _handle_param;

private:
    // =============================
    // ROS : loop function
    // =============================
    void callback_timer();
    rclcpp::TimerBase::SharedPtr _ros_timer;

private:
    // =============================
    // CONST
    // =============================
    const int CONFIG_QOS               = 255;
    const int CONFIG_SUBSCRIPTION_SIZE = 5;
    const std::chrono::milliseconds TP_MSEC{ 100 };

    const std::string MRS_OUTPUT_VOICE_TEXT = "voice_text";
    //const std::string MRS_PARAMETER_SAMPLE_TIMES  = "param/times";
};

} // namespace maid_robot_system

#endif
