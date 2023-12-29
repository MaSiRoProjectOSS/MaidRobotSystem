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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "models/model_implement.hpp"
#include "rclcpp/rclcpp.hpp"

#include <maid_robot_system_interfaces/msg/mrs_sample.hpp>
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

    void set_move_velocity_reference();

private:
    // =============================
    // Variable
    // =============================
    ModelImplement _model;

    geometry_msgs::msg::Twist _msg_move_velocity_reference;

    Vector3 *_translational_velocity = new Vector3(0.0f, 0.0f, 0.0f);
    Vector3 *_rotational_velocity    = new Vector3(0.0f, 0.0f, 0.0f);

private:
    // =============================
    // ROS : publisher
    // =============================
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_move_velocity_reference;

private:
    // =============================
    // ROS : subscription
    // =============================
    void _callback_robot_position_rotation(const geometry_msgs::msg::PoseStamped &msg);
    void _callback_hand_position(const geometry_msgs::msg::Point &msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_robot_position_rotation;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _sub_hand_position;

private:
    // =============================
    // ROS : parameter
    // =============================
    int _callback_param_init();
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
    const std::chrono::milliseconds PERIOD_MSEC{ 50 };

    // =============================
    // ROS Topic / Service / Action
    // =============================
    const std::string MRS_TOPIC_OUTPUT                  = "out/move_velocity_reference";
    const std::string MRS_TOPIC_ROBOT_POSITION_ROTATION = "in/robot_position_rotation";
    const std::string MRS_TOPIC_HAND_POSITION           = "in/hand_position";

    // =============================
    // ROS PARAMETER
    // =============================
    const std::string MRS_PARAMETER_SAMPLE_TIMES  = "param/times";
    const std::string MRS_PARAMETER_SAMPLE_OFFSET = "param/offset";
};

} // namespace maid_robot_system

#endif
