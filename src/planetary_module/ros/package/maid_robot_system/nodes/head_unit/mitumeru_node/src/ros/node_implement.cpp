/**
 * @file  node_implement.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "ros/node_implement.hpp"
using std::placeholders::_1;

namespace maid_robot_system
{
// =============================
// ROS : subscription
// =============================
void NodeImplement::callback_pose_left(const maid_robot_system_interfaces::msg::PoseDetection &msg)
{
    this->_model.set_value_pose(ModelStructure::INPUT_TYPE::POSE_LEFT, msg);
}
void NodeImplement::callback_pose_right(const maid_robot_system_interfaces::msg::PoseDetection &msg)
{
    this->_model.set_value_pose(ModelStructure::INPUT_TYPE::POSE_RIGHT, msg);
}
void NodeImplement::callback_ar_left(const maid_robot_system_interfaces::msg::ArMarkers &msg)
{
    for (long unsigned int i = 0; i < msg.ids.size(); i++) {
        if (0 != msg.ids[i]) {
            this->_model.set_value_ar(ModelStructure::INPUT_TYPE::AR_LEFT, msg.ids[i]);
        }
    }
}
void NodeImplement::callback_ar_right(const maid_robot_system_interfaces::msg::ArMarkers &msg)
{
    for (long unsigned int i = 0; i < msg.ids.size(); i++) {
        if (0 != msg.ids[i]) {
            this->_model.set_value_ar(ModelStructure::INPUT_TYPE::AR_RIGHT, msg.ids[i]);
        }
    }
}
void NodeImplement::callback_voice(const maid_robot_system_interfaces::msg::MrsVoice &msg)
{
    this->_model.set_value_voice(msg.text, msg.command);
}

// =============================
// ROS : parameter
// =============================
void NodeImplement::callback_param()
{
    // declare_parameter
    // this->declare_parameter(MRS_PARAMETER_SAMPLE_TIMES, this->_model.get_times());
    // this->declare_parameter(MRS_PARAMETER_SAMPLE_OFFSET, this->_model.get_offset());

    // make parameter callback
    this->_sub_parameter = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto cb              = [this](const rclcpp::Parameter &param) {
        std::string p_name = param.get_name().c_str();
        RCLCPP_INFO(this->get_logger(), "GET param - %s", p_name.c_str());
        switch (param.get_type()) {
            case rclcpp::PARAMETER_DOUBLE:
                /*
                if (0 == p_name.compare(MRS_PARAMETER_SAMPLE_TIMES)) {
                    // this->_model.set_times(param.as_double());
                    RCLCPP_INFO(this->get_logger(), "  set param : %s[%f]", MRS_PARAMETER_SAMPLE_TIMES, this->_model.get_times());
                }
                if (0 == p_name.compare(MRS_PARAMETER_SAMPLE_OFFSET)) {
                    // this->_model.set_offset(param.as_double());
                    RCLCPP_INFO(this->get_logger(), "  set param : %s[%f]", MRS_PARAMETER_SAMPLE_OFFSET, this->_model.get_offset());
                }
                */
                break;
            case rclcpp::PARAMETER_INTEGER:
            case rclcpp::PARAMETER_NOT_SET:
            case rclcpp::PARAMETER_BOOL:
            case rclcpp::PARAMETER_STRING:
            case rclcpp::PARAMETER_BYTE_ARRAY:
            case rclcpp::PARAMETER_BOOL_ARRAY:
            case rclcpp::PARAMETER_INTEGER_ARRAY:
            case rclcpp::PARAMETER_DOUBLE_ARRAY:
            case rclcpp::PARAMETER_STRING_ARRAY:
            default:
                break;
        }
    };

    this->_handle_param1 = this->_sub_parameter->add_parameter_callback(MRS_PARAMETER_SAMPLE_OFFSET, cb);
    this->_handle_param2 = this->_sub_parameter->add_parameter_callback(MRS_PARAMETER_SAMPLE_TIMES, cb);
}

// =============================
// ROS : loop function
// =============================
void NodeImplement::callback_timer()
{
    bool result = this->_model.calculate(this->_msg_hitomi, this->_msg_kubi, this->_msg_kuchibiru);
    this->_pub_hitomi->publish(this->_msg_hitomi);
    this->_pub_kubi->publish(this->_msg_kubi);
    this->_pub_kuchibiru->publish(this->_msg_kuchibiru);
}

// =============================
// Constructor
// =============================
NodeImplement::NodeImplement(std::string node_name, int argc, char **argv) : Node(node_name)
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "start.");

    // set parameter
    this->callback_param();

    // set publisher
    this->_pub_hitomi =                                                           //
            this->create_publisher<maid_robot_system_interfaces::msg::MrsHitomi>( //
                    this->MRS_TOPIC_OUT_HITOMI,                                   //
                    rclcpp::QoS(this->CONFIG_QOS)                                 //
            );
    this->_pub_kubi =                                                           //
            this->create_publisher<maid_robot_system_interfaces::msg::MrsKubi>( //
                    this->MRS_TOPIC_OUT_KUBI,                                   //
                    rclcpp::QoS(this->CONFIG_QOS)                               //
            );
    this->_pub_kuchibiru =                                                           //
            this->create_publisher<maid_robot_system_interfaces::msg::MrsKuchibiru>( //
                    this->MRS_TOPIC_OUT_KUCHIBIRU,                                   //
                    rclcpp::QoS(this->CONFIG_QOS)                                    //
            );

    // set subscription
    this->_sub_pose_left =                                                               //
            this->create_subscription<maid_robot_system_interfaces::msg::PoseDetection>( //
                    this->MRS_TOPIC_IN_POSE_LEFT,                                        //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                      //
                    std::bind(&NodeImplement::callback_pose_left, this, _1));
    this->_sub_pose_right =                                                              //
            this->create_subscription<maid_robot_system_interfaces::msg::PoseDetection>( //
                    this->MRS_TOPIC_IN_POSE_RIGHT,                                       //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                      //
                    std::bind(&NodeImplement::callback_pose_right, this, _1));
    this->_sub_ar_left =                                                             //
            this->create_subscription<maid_robot_system_interfaces::msg::ArMarkers>( //
                    this->MRS_TOPIC_IN_AR_LEFT,                                      //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                  //
                    std::bind(&NodeImplement::callback_ar_left, this, _1));
    this->_sub_ar_right =                                                            //
            this->create_subscription<maid_robot_system_interfaces::msg::ArMarkers>( //
                    this->MRS_TOPIC_IN_AR_RIGHT,                                     //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                  //
                    std::bind(&NodeImplement::callback_ar_right, this, _1));
    this->_sub_voice =                                                              //
            this->create_subscription<maid_robot_system_interfaces::msg::MrsVoice>( //
                    this->MRS_TOPIC_IN_VOICE,                                       //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                 //
                    std::bind(&NodeImplement::callback_voice, this, _1));

    this->_ros_timer = this->create_wall_timer(this->TP_MSEC, std::bind(&NodeImplement::callback_timer, this));
}

NodeImplement::~NodeImplement()
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "fin.");
}

} // namespace maid_robot_system
