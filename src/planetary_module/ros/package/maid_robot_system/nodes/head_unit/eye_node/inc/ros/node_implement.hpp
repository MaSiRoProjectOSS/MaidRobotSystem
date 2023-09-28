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
    std_msgs::msg::Float64 _msg_convert;

private:
    // =============================
    // ROS : publisher
    // =============================
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pub_value;

private:
    // =============================
    // ROS : subscription
    // =============================
    void callback_message(const maid_robot_system_interfaces::msg::MrsEye &msg);

    rclcpp::Subscription<maid_robot_system_interfaces::msg::MrsEye>::SharedPtr _sub_value;

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
    const std::chrono::milliseconds TP_MSEC{ 10 };

    const std::string MRS_TOPIC_OUTPUT = "out";
    const std::string MRS_TOPIC_INPUT  = "in";

    const std::string MRS_PARAMETER_SETTING_FILE = "setting_file";
    const std::string MRS_PARAMETER_SKIN_NAME    = "skin_name";

    const std::string MRS_PARAMETER_LEFT_WIDTH         = "left/width";
    const std::string MRS_PARAMETER_LEFT_HEIGHT        = "left/height";
    const std::string MRS_PARAMETER_LEFT_CENTER_X      = "left/center/x";
    const std::string MRS_PARAMETER_LEFT_CENTER_Y      = "left/center/y";
    const std::string MRS_PARAMETER_LEFT_CENTER_ANGLE  = "left/center/angle";
    const std::string MRS_PARAMETER_LEFT_EYEBALL_X     = "left/eyeball/x";
    const std::string MRS_PARAMETER_LEFT_EYEBALL_Y     = "left/eyeball/y";
    const std::string MRS_PARAMETER_LEFT_EYEBALL_ANGLE = "left/eyeball/angle";

    const std::string MRS_PARAMETER_RIGHT_WIDTH         = "right/width";
    const std::string MRS_PARAMETER_RIGHT_HEIGHT        = "right/height";
    const std::string MRS_PARAMETER_RIGHT_CENTER_X      = "right/center/x";
    const std::string MRS_PARAMETER_RIGHT_CENTER_Y      = "right/center/y";
    const std::string MRS_PARAMETER_RIGHT_CENTER_ANGLE  = "right/center/angle";
    const std::string MRS_PARAMETER_RIGHT_EYEBALL_X     = "right/eyeball/x";
    const std::string MRS_PARAMETER_RIGHT_EYEBALL_Y     = "right/eyeball/y";
    const std::string MRS_PARAMETER_RIGHT_EYEBALL_ANGLE = "right/eyeball/angle";

    const std::string MRS_PARAMETER_EYELID_WIDTH  = "eyelid/width";
    const std::string MRS_PARAMETER_EYELID_HEIGHT = "eyelid/height";

    const std::string MRS_PARAMETER_BLINK_QUICKLY_MS = "blink_time/quickly_ms";
    const std::string MRS_PARAMETER_BLINK_MIN_MS     = "blink_time/min_ms";
    const std::string MRS_PARAMETER_BLINK_MAX_MS     = "blink_time/max_ms";
    const std::string MRS_PARAMETER_BLINK_LIMIT_MS   = "blink_time/limit_ms";
    const std::string MRS_PARAMETER_BLINK_OFFSET_MS  = "blink_time/offset_ms";
};

} // namespace maid_robot_system

#endif
