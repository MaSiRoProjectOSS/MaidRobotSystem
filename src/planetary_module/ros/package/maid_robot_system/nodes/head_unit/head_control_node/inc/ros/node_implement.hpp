/**
 * @file  node_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_HEAD_CONTROL_NODE_IMPLEMENT_HPP
#define MRS_HEAD_CONTROL_NODE_IMPLEMENT_HPP

#include "models/model_implement.hpp"
#include "rclcpp/rclcpp.hpp"

#include <maid_robot_system_interfaces/msg/ar_markers.hpp>
#include <maid_robot_system_interfaces/msg/mrs_voice.hpp>
#include <maid_robot_system_interfaces/msg/pose_detection.hpp>
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
    maid_robot_system_interfaces::msg::MrsEye _msg_eye;
    maid_robot_system_interfaces::msg::MrsNeck _msg_neck;
    maid_robot_system_interfaces::msg::MrsLip _msg_lip;

private:
    // =============================
    // ROS : publisher
    // =============================
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsEye>::SharedPtr _pub_eye;
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsNeck>::SharedPtr _pub_neck;
    rclcpp::Publisher<maid_robot_system_interfaces::msg::MrsLip>::SharedPtr _pub_lip;

private:
    // =============================
    // ROS : subscription
    // =============================
    void callback_pose_left(const maid_robot_system_interfaces::msg::PoseDetection &msg);
    void callback_pose_right(const maid_robot_system_interfaces::msg::PoseDetection &msg);
    void callback_ar_left(const maid_robot_system_interfaces::msg::ArMarkers &msg);
    void callback_ar_right(const maid_robot_system_interfaces::msg::ArMarkers &msg);
    void callback_voice(const maid_robot_system_interfaces::msg::MrsVoice &msg);
    void callback_voltage(const std_msgs::msg::Float64 &msg);

    rclcpp::Subscription<maid_robot_system_interfaces::msg::PoseDetection>::SharedPtr _sub_pose_left;
    rclcpp::Subscription<maid_robot_system_interfaces::msg::PoseDetection>::SharedPtr _sub_pose_right;
    rclcpp::Subscription<maid_robot_system_interfaces::msg::ArMarkers>::SharedPtr _sub_ar_left;
    rclcpp::Subscription<maid_robot_system_interfaces::msg::ArMarkers>::SharedPtr _sub_ar_right;
    rclcpp::Subscription<maid_robot_system_interfaces::msg::MrsVoice>::SharedPtr _sub_voice;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub_voltage;

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
    void callback_timer();
    rclcpp::TimerBase::SharedPtr _ros_timer;

private:
    // =============================
    // CONST
    // =============================
    const int CONFIG_QOS               = 255;
    const int CONFIG_SUBSCRIPTION_SIZE = 2;
    const std::chrono::milliseconds TP_MSEC{ 16 };

    const std::string MRS_TOPIC_IN_POSTURE_LEFT  = "in/posture/left";
    const std::string MRS_TOPIC_IN_POSTURE_RIGHT = "in/posture/right";
    const std::string MRS_TOPIC_IN_MARKS_LEFT    = "in/marks/left";
    const std::string MRS_TOPIC_IN_MARKS_RIGHT   = "in/marks/right";
    const std::string MRS_TOPIC_IN_VOICE         = "in/voice_text";
    const std::string MRS_TOPIC_IN_VOLTAGE       = "in/voltage";
    const std::string MRS_TOPIC_OUT_EYE          = "out/eye";
    const std::string MRS_TOPIC_OUT_NECK         = "out/neck";
    const std::string MRS_TOPIC_OUT_LIP          = "out/lip";

    const std::string MRS_PARAM_EYE_LEFT_OFFSET_X     = "eye/left/offset/x";
    const std::string MRS_PARAM_EYE_LEFT_OFFSET_Y     = "eye/left/offset/y";
    const std::string MRS_PARAM_EYE_LEFT_OFFSET_ANGLE = "eye/left/offset/angle";

    const std::string MRS_PARAM_EYE_RIGHT_OFFSET_X     = "eye/right/offset/x";
    const std::string MRS_PARAM_EYE_RIGHT_OFFSET_Y     = "eye/right/offset/y";
    const std::string MRS_PARAM_EYE_RIGHT_OFFSET_ANGLE = "eye/right/offset/angle";

    const std::string MRS_PARAM_NECK_PITCH_MIN = "neck/pitch/min";
    const std::string MRS_PARAM_NECK_PITCH_MAX = "neck/pitch/max";
    const std::string MRS_PARAM_NECK_YAW_MIN   = "neck/yaw/min";
    const std::string MRS_PARAM_NECK_YAW_MAX   = "neck/yaw/max";
    const std::string MRS_PARAM_NECK_ROLL_MIN  = "neck/roll/min";
    const std::string MRS_PARAM_NECK_ROLL_MAX  = "neck/roll/max";

    const std::string MRS_PARAM_LIP_MIN = "lip/min";
    const std::string MRS_PARAM_LIP_MAX = "lip/max";

    const std::string MRS_PARAM_TIREDNESS = "tiredness";

    const std::string MRS_PARAM_PRIORITY_TO_THE_RIGHT = "priority/right_hand";

    const std::string MRS_PARAM_TIMEOUT_S_RECEIVED = "timeout_s/received";
    const std::string MRS_PARAM_TIMEOUT_S_CHASED   = "timeout_s/chased";
};

} // namespace maid_robot_system

#endif
