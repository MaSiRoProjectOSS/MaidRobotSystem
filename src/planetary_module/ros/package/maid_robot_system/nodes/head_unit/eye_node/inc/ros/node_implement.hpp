/**
 * @file  node_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_IMPLEMENT_HPP
#define MRS_EYE_NODE_IMPLEMENT_HPP

#include "models/model_implement.hpp"
#include "rclcpp/rclcpp.hpp"

#include <maid_robot_system_interfaces/msg/mrs_eye.hpp>
#include <std_msgs/msg/float64.hpp>

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
    void _callback_msg_mrs_eye(const maid_robot_system_interfaces::msg::MrsEye &msg);
    rclcpp::Subscription<maid_robot_system_interfaces::msg::MrsEye>::SharedPtr _sub_mrs_eye;

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
    const std::chrono::milliseconds TP_MSEC{ 10 };

    // =============================
    // ROS Topic / Service / Action
    // =============================
    const std::string MRS_TOPIC_INPUT = "in";

    // =============================
    // ROS PARAMETER
    // =============================
    const std::string MRS_PARAMETER_SETTING_FILE = "setting_file";
    const std::string MRS_PARAMETER_BRIGHTNESS   = "brightness";
    const std::string MRS_PARAMETER_COLOR_R      = "color/r";
    const std::string MRS_PARAMETER_COLOR_G      = "color/g";
    const std::string MRS_PARAMETER_COLOR_B      = "color/b";
};

} // namespace maid_robot_system

#endif
