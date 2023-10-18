/**
 * @file  interaction_node.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_INTERACTION_NODE_HPP
#define MRS_EYE_NODE_INTERACTION_NODE_HPP

#include "eye_node_settings.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros/widget_node.hpp"

#include <maid_robot_system_interfaces/msg/mrs_eye.hpp>

namespace maid_robot_system
{
class InteractionNode : public rclcpp::Node {
public:
    // =============================
    // Constructor
    // =============================
    InteractionNode(std::string node_name, WidgetNode &widget);
    ~InteractionNode();

private:
    // =============================
    // ROS : parameter
    // =============================
    void _callback_param_init();
    OnSetParametersCallbackHandle::SharedPtr _handle_param;

private:
    // =============================
    // ROS : subscription
    // =============================
    void _callback_msg_mrs_eye(const maid_robot_system_interfaces::msg::MrsEye &msg);
    rclcpp::Subscription<maid_robot_system_interfaces::msg::MrsEye>::SharedPtr _sub_mrs_eye;

private:
    // =============================
    // ROS : loop function
    // =============================
    void _callback_timer();
    rclcpp::TimerBase::SharedPtr _ros_timer;
#if LOGGER_ROS_INFO_OUTPUT_REPORT_TIME > 0
    void _callback_output_state();
    rclcpp::TimerBase::SharedPtr _ros_output_state;
#endif

private:
    // =============================
    // PRIVATE : Function
    // =============================
    bool _set_setting_file(std::string value);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    WidgetNode *_widget;
    bool _is_notify_enable  = false;
    bool _is_notify_verbose = false;

private:
    // =============================
    // CONST
    // =============================
    const rclcpp::QoS DEPTH_SUBSCRIPTION = rclcpp::QoS(5);
    const std::chrono::milliseconds PERIOD_MSEC{ 1000 / DRAWING_MAX_FPS };
#if LOGGER_ROS_INFO_OUTPUT_REPORT_TIME > 0
    const std::chrono::milliseconds PERIOD_OUTPUT_REPORT_MSEC{ LOGGER_ROS_INFO_OUTPUT_REPORT_TIME };
#endif

private:
    // =============================
    // CONST: ROS Name :Topic / Service / Action
    // =============================
    const std::string MRS_TOPIC_INPUT = "in";

private:
    // =============================
    // CONST: ROS PARAMETER
    // =============================
    const std::string MRS_PARAMETER_SETTING_FILE    = "setting_file";
    const std::string MRS_PARAMETER_BRIGHTNESS      = "brightness";
    const std::string MRS_PARAMETER_CILIARY_COLOR_R = "ciliary/color/r";
    const std::string MRS_PARAMETER_CILIARY_COLOR_G = "ciliary/color/g";
    const std::string MRS_PARAMETER_CILIARY_COLOR_B = "ciliary/color/b";
    const std::string MRS_PARAMETER_EYELID_COLOR_R  = "eyelid/color/r";
    const std::string MRS_PARAMETER_EYELID_COLOR_G  = "eyelid/color/g";
    const std::string MRS_PARAMETER_EYELID_COLOR_B  = "eyelid/color/b";

    const std::string MRS_PARAMETER_NOTIFY_MESSAGE_ENABLE  = "notify/message/enable";
    const std::string MRS_PARAMETER_NOTIFY_MESSAGE_VERBOSE = "notify/message/verbose";
};

} // namespace maid_robot_system

#endif
