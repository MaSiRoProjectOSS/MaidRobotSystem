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
void NodeImplement::callback_message(const std_msgs::msg::Float64 &msg)
{
    this->_convert_msg.data = this->_model.calculate(msg.data);
    // RCLCPP_DEBUG(this->get_logger(), "callback_message() : %f", this->_convert_msg.data);
    RCLCPP_INFO(this->get_logger(), "callback_message() : %f", this->_convert_msg.data);
    // RCLCPP_WARN(this->get_logger(), "callback_message() : %f", this->_convert_msg.data);
    // RCLCPP_ERROR(this->get_logger(), "callback_message() : %f", this->_convert_msg.data);
    // RCLCPP_FATAL(this->get_logger(), "callback_message() : %f", this->_convert_msg.data);
}

void NodeImplement::callback_param()
{
    // declare_parameter
    this->declare_parameter(MRS_PARAMETER_SAMPLE_TIMES, this->_model.get_times());
    this->declare_parameter(MRS_PARAMETER_SAMPLE_OFFSET, this->_model.get_offset());

    // make parameter callback
    this->_sub_parameter = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto cb              = [this](const rclcpp::Parameter &param) {
        std::string p_name = param.get_name().c_str();
        RCLCPP_INFO(this->get_logger(), "GET param - %s", p_name.c_str());
        switch (param.get_type()) {
            case rclcpp::PARAMETER_DOUBLE:
                if (0 == p_name.compare(MRS_PARAMETER_SAMPLE_TIMES)) {
                    this->_model.set_times(param.as_double());
                    RCLCPP_INFO(this->get_logger(), "  set param : %s[%f]", MRS_PARAMETER_SAMPLE_TIMES, this->_model.get_times());
                }
                if (0 == p_name.compare(MRS_PARAMETER_SAMPLE_OFFSET)) {
                    this->_model.set_offset(param.as_double());
                    RCLCPP_INFO(this->get_logger(), "  set param : %s[%f]", MRS_PARAMETER_SAMPLE_OFFSET, this->_model.get_offset());
                }
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

void NodeImplement::callback_timer()
{
    this->_pub_value->publish(this->_convert_msg);

    static double __times  = 0;
    static double __offset = 0;
    static double __data   = 0;
    if ((__times != this->_model.get_times()) || (__offset != this->_model.get_offset()) || (__data != this->_convert_msg.data)) {
        RCLCPP_INFO(this->get_logger(), "callback_timer : %f = %f * n + %f", this->_convert_msg.data, this->_model.get_times(), this->_model.get_offset());
        __times  = this->_model.get_times();
        __offset = this->_model.get_offset();
        __data   = this->_convert_msg.data;
    }
}

NodeImplement::NodeImplement(std::string node_name, int argc, char **argv) : Node(node_name)
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "start.");

    // set parameter
    this->callback_param();

    // set publisher
    this->_pub_value =                                      //
            this->create_publisher<std_msgs::msg::Float64>( //
                    this->MRS_TOPIC_OUTPUT,                 //
                    rclcpp::QoS(this->CONFIG_QOS)           //
            );
    // set subscription
    this->_sub_value =                                         //
            this->create_subscription<std_msgs::msg::Float64>( //
                    this->MRS_TOPIC_INPUT,                     //
                    this->CONFIG_SUBSCRIPTION_SIZE,            //
                    std::bind(&NodeImplement::callback_message, this, _1));

    this->_ros_timer = this->create_wall_timer(this->TP_MSEC, std::bind(&NodeImplement::callback_timer, this));
}

NodeImplement::~NodeImplement()
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "fin.");
}

} // namespace maid_robot_system
