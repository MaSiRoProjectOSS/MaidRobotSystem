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
    maid_robot_system_interfaces::msg::MrsHitomi _msg_oculus;
    maid_robot_system_interfaces::msg::MrsKubi _msg_neck;
    maid_robot_system_interfaces::msg::MrsKuchibiru _msg_lip;

private:
    // =============================
    // ROS : publisher
    // =============================
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsHitomi>::SharedPtr _pub_oculus;
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsKubi>::SharedPtr _pub_neck;
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsKuchibiru>::SharedPtr _pub_lip;

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
    const int CONFIG_SUBSCRIPTION_SIZE = 2;
    const std::chrono::milliseconds TP_MSEC{ 16 };

    const std::string MRS_TOPIC_IN_POSE_LEFT  = "in/pose/left";
    const std::string MRS_TOPIC_IN_POSE_RIGHT = "in/pose/left";
    const std::string MRS_TOPIC_IN_AR_LEFT    = "in/ar/left";
    const std::string MRS_TOPIC_IN_AR_RIGHT   = "in/ar/left";
    const std::string MRS_TOPIC_IN_VOICE      = "in/voice";
    const std::string MRS_TOPIC_OUT_OCULUS    = "out/oculus";
    const std::string MRS_TOPIC_OUT_NECK      = "out/neck";
    const std::string MRS_TOPIC_OUT_LIP       = "out/lip";

    const std::string MRS_PARAM_OCULUS_CENTER_X_LEFT    = "oculus/left/center/x";
    const std::string MRS_PARAM_OCULUS_CENTER_Y_LEFT    = "oculus/left/center/y";
    const std::string MRS_PARAM_OCULUS_OFFSET_UP_LEFT   = "oculus/left/offset/up";
    const std::string MRS_PARAM_OCULUS_OFFSET_DOWN_LEFT = "oculus/left/offset/down";

    const std::string MRS_PARAM_OCULUS_CENTER_X_RIGHT    = "oculus/right/center/x";
    const std::string MRS_PARAM_OCULUS_CENTER_Y_RIGHT    = "oculus/right/center/y";
    const std::string MRS_PARAM_OCULUS_OFFSET_UP_RIGHT   = "oculus/right/offset/up";
    const std::string MRS_PARAM_OCULUS_OFFSET_DOWN_RIGHT = "oculus/right/offset/down";

    const std::string MRS_PARAM_NECK_PITCH_MIN = "neck/pitch/min";
    const std::string MRS_PARAM_NECK_PITCH_MAX = "neck/pitch/max";
    const std::string MRS_PARAM_NECK_YAW_MIN   = "neck/yaw/min";
    const std::string MRS_PARAM_NECK_YAW_MAX   = "neck/yaw/max";
    const std::string MRS_PARAM_NECK_ROLL_MIN  = "neck/roll/min";
    const std::string MRS_PARAM_NECK_ROLL_MAX  = "neck/roll/max";

    const std::string MRS_PARAM_LIP_MIN = "lip/min";
    const std::string MRS_PARAM_LIP_MAX = "lip/max";
};

} // namespace maid_robot_system

#endif
