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
    void _callback_pose_left(const maid_robot_system_interfaces::msg::PoseDetection &msg);
    void _callback_pose_right(const maid_robot_system_interfaces::msg::PoseDetection &msg);
    void _callback_ar_left(const maid_robot_system_interfaces::msg::ArMarkers &msg);
    void _callback_ar_right(const maid_robot_system_interfaces::msg::ArMarkers &msg);
    void _callback_voice(const maid_robot_system_interfaces::msg::MrsVoice &msg);
    void _callback_voltage(const std_msgs::msg::Float64 &msg);

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
    void _callback_timer();
    rclcpp::TimerBase::SharedPtr _ros_timer;

private:
    // =============================
    // CONST
    // =============================
    const rclcpp::QoS DEPTH_PUBLISHER    = rclcpp::QoS(5);
    const rclcpp::QoS DEPTH_SUBSCRIPTION = rclcpp::QoS(2);
    const std::chrono::milliseconds PERIOD_MSEC{ 16 };

    // =============================
    // ROS Topic / Service / Action
    // =============================
    const std::string MRS_TOPIC_IN_POSTURE_LEFT  = "in/posture/left";
    const std::string MRS_TOPIC_IN_POSTURE_RIGHT = "in/posture/right";
    const std::string MRS_TOPIC_IN_MARKS_LEFT    = "in/marks/left";
    const std::string MRS_TOPIC_IN_MARKS_RIGHT   = "in/marks/right";
    const std::string MRS_TOPIC_IN_VOICE         = "in/voice_text";
    const std::string MRS_TOPIC_IN_VOLTAGE       = "in/voltage";
    const std::string MRS_TOPIC_OUT_EYE          = "out/eye";
    const std::string MRS_TOPIC_OUT_NECK         = "out/neck";
    const std::string MRS_TOPIC_OUT_LIP          = "out/lip";

    // =============================
    // ROS PARAMETER
    // =============================
    // eye : left
    const std::string MRS_PARAM_EYE_LEFT_OFFSET_X     = "eye/left/offset/x";
    const std::string MRS_PARAM_EYE_LEFT_OFFSET_Y     = "eye/left/offset/y";
    const std::string MRS_PARAM_EYE_LEFT_OFFSET_ANGLE = "eye/left/offset/angle";

    // eye : right
    const std::string MRS_PARAM_EYE_RIGHT_OFFSET_X     = "eye/right/offset/x";
    const std::string MRS_PARAM_EYE_RIGHT_OFFSET_Y     = "eye/right/offset/y";
    const std::string MRS_PARAM_EYE_RIGHT_OFFSET_ANGLE = "eye/right/offset/angle";

    // neck
    const std::string MRS_PARAM_NECK_PITCH_MIN = "neck/pitch/min";
    const std::string MRS_PARAM_NECK_PITCH_MAX = "neck/pitch/max";
    const std::string MRS_PARAM_NECK_YAW_MIN   = "neck/yaw/min";
    const std::string MRS_PARAM_NECK_YAW_MAX   = "neck/yaw/max";
    const std::string MRS_PARAM_NECK_ROLL_MIN  = "neck/roll/min";
    const std::string MRS_PARAM_NECK_ROLL_MAX  = "neck/roll/max";

    // lip
    const std::string MRS_PARAM_LIP_MIN = "lip/min";
    const std::string MRS_PARAM_LIP_MAX = "lip/max";

    // tiredness
    const std::string MRS_PARAM_TIREDNESS = "tiredness";

    // priority
    const std::string MRS_PARAM_PRIORITY_TO_THE_RIGHT = "priority/right_hand";

    // timeout_s
    const std::string MRS_PARAM_TIMEOUT_S_RECEIVED = "timeout_s/received";
    const std::string MRS_PARAM_TIMEOUT_S_CHASED   = "timeout_s/chased";
};

} // namespace maid_robot_system

#endif
