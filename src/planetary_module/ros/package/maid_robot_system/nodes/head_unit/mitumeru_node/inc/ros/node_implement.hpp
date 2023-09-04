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

#include <maid_robot_system_interfaces/msg/ar_markers.hpp>
#include <maid_robot_system_interfaces/msg/mrs_voice.hpp>
#include <maid_robot_system_interfaces/msg/pose_detection.hpp>

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
    maid_robot_system_interfaces::msg::MrsHitomi _msg_hitomi;
    maid_robot_system_interfaces::msg::MrsKubi _msg_kubi;
    maid_robot_system_interfaces::msg::MrsKuchibiru _msg_kuchibiru;

private:
    // =============================
    // ROS : publisher
    // =============================
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsHitomi>::SharedPtr _pub_hitomi;
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsKubi>::SharedPtr _pub_kubi;
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsKuchibiru>::SharedPtr _pub_kuchibiru;

private:
    // =============================
    // ROS : subscription
    // =============================
    void callback_pose_left(const maid_robot_system_interfaces::msg::PoseDetection &msg);
    void callback_pose_right(const maid_robot_system_interfaces::msg::PoseDetection &msg);
    void callback_ar_left(const maid_robot_system_interfaces::msg::ArMarkers &msg);
    void callback_ar_right(const maid_robot_system_interfaces::msg::ArMarkers &msg);
    void callback_voice(const maid_robot_system_interfaces::msg::MrsVoice &msg);

    rclcpp::Subscription<maid_robot_system_interfaces::msg::PoseDetection>::SharedPtr _sub_pose_left;
    rclcpp::Subscription<maid_robot_system_interfaces::msg::PoseDetection>::SharedPtr _sub_pose_right;
    rclcpp::Subscription<maid_robot_system_interfaces::msg::ArMarkers>::SharedPtr _sub_ar_left;
    rclcpp::Subscription<maid_robot_system_interfaces::msg::ArMarkers>::SharedPtr _sub_ar_right;
    rclcpp::Subscription<maid_robot_system_interfaces::msg::MrsVoice>::SharedPtr _sub_voice;

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

private:
    // =============================
    // CONST
    // =============================
    const int CONFIG_QOS               = 255;
    const int CONFIG_SUBSCRIPTION_SIZE = 2;
    const std::chrono::milliseconds TP_MSEC{ 16 };

    const std::string MRS_TOPIC_IN_POSE_LEFT  = "in/pose/left";
    const std::string MRS_TOPIC_IN_POSE_RIGHT = "in/pose/left";
    const std::string MRS_TOPIC_IN_AR_LEFT    = "in/ar/left";
    const std::string MRS_TOPIC_IN_AR_RIGHT   = "in/ar/left";
    const std::string MRS_TOPIC_IN_VOICE      = "in/voice";
    const std::string MRS_TOPIC_OUT_HITOMI    = "out/hitomi";
    const std::string MRS_TOPIC_OUT_KUBI      = "out/kubi";
    const std::string MRS_TOPIC_OUT_KUCHIBIRU = "out/kuchibiru";
};

} // namespace maid_robot_system

#endif
