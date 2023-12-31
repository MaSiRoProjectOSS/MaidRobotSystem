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

#include <std_msgs/msg/string.hpp>

#ifndef LOGGER_ROS_INFO_CALL_FUNCTION
#define LOGGER_ROS_INFO_CALL_FUNCTION 0
#endif
#ifndef LOGGER_ROS_INFO_PARAMETER
#define LOGGER_ROS_INFO_PARAMETER 0
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
    std_msgs::msg::String _msg_text;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_text;

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
    const rclcpp::QoS DEPTH_PUBLISHER    = rclcpp::QoS(5);
    const rclcpp::QoS DEPTH_SUBSCRIPTION = rclcpp::QoS(5);
    const std::chrono::milliseconds PERIOD_MSEC{ 100 };

    // =============================
    // ROS Topic / Service / Action
    // =============================
    const std::string MRS_OUTPUT_VOICE_TEXT = "voice_text";

    // =============================
    // ROS PARAMETER
    // =============================
    //const std::string MRS_PARAMETER_SAMPLE_TIMES  = "param/times";
};

} // namespace maid_robot_system

#endif
