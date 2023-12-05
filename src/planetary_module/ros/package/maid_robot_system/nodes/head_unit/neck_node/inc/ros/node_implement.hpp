/**
 * @file  node_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_NECK_NODE_IMPLEMENT_HPP
#define MRS_NECK_NODE_IMPLEMENT_HPP

#include "models/model_implement.hpp"
#include "rclcpp/rclcpp.hpp"

#include <maid_robot_system_interfaces/msg/mrs_neck.hpp>

#ifndef LOGGER_ROS_INFO_CALL_FUNCTION
#define LOGGER_ROS_INFO_CALL_FUNCTION 0
#endif
#ifndef LOGGER_ROS_INFO_PARAMETER
#define LOGGER_ROS_INFO_PARAMETER 0
#endif
#ifndef LOGGER_ROS_INFO_GET_MESSAGE
#define LOGGER_ROS_INFO_GET_MESSAGE 0
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
    // ROS : subscription
    // =============================
    void _callback_data(const maid_robot_system_interfaces::msg::MrsNeck &msg);
    rclcpp::Subscription<maid_robot_system_interfaces::msg::MrsNeck>::SharedPtr _sub_neck;

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
    const std::chrono::milliseconds PERIOD_MSEC{ 25 };

    // =============================
    // ROS Topic / Service / Action
    // =============================
    const std::string MRS_TOPIC_INPUT = "in";

    // =============================
    // ROS PARAMETER
    // =============================
};

} // namespace maid_robot_system

#endif
