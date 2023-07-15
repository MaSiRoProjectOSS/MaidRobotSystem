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

#define NODE_IMPLEMENT_NODE MRS_NODE_TEMPLATE

void NodeImplement::callback_message(const std_msgs::msg::Float64 &msg)
{
    RCLCPP_DEBUG(this->_node_ptr->get_logger(), "callback_message()");
    RCLCPP_INFO(this->_node_ptr->get_logger(), "callback_message()");
    RCLCPP_WARN(this->_node_ptr->get_logger(), "callback_message()");
    RCLCPP_ERROR(this->_node_ptr->get_logger(), "callback_message()");
    RCLCPP_FATAL(this->_node_ptr->get_logger(), "callback_message()");
    this->_convert_msg.data = this->_model.calculate(msg.data);
}

void NodeImplement::callback_param()
{
    // declare_parameter
    this->declare_parameter(MRS_PARAMETER_TEMPLATE_TIMES, this->_model.get_times());
    this->declare_parameter(MRS_PARAMETER_TEMPLATE_OFFSET, this->_model.get_offset());

    // make parameter callback
    this->_sub_parameter = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto cb              = [this](const rclcpp::Parameter &param) {
        std::string p_name = param.get_name().c_str();
        switch (param.get_type()) {
            case rclcpp::PARAMETER_DOUBLE:
                if (0 == p_name.compare(MRS_PARAMETER_TEMPLATE_TIMES)) {
                    this->_model.set_times(param.as_double());
                    RCLCPP_INFO(this->_node_ptr->get_logger(), "set param : %s[%f]", MRS_PARAMETER_TEMPLATE_TIMES, this->_model.get_times());
                }
                if (0 == p_name.compare(MRS_PARAMETER_TEMPLATE_OFFSET)) {
                    this->_model.set_offset(param.as_double());
                    RCLCPP_INFO(this->_node_ptr->get_logger(), "set param : %s[%f]", MRS_PARAMETER_TEMPLATE_OFFSET, this->_model.get_offset());
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
    this->_handle_param = this->_sub_parameter->add_parameter_callback("param_" NODE_IMPLEMENT_NODE, cb);
}

void NodeImplement::callback_timer()
{
    this->_pub_value->publish(this->_convert_msg);
}

NodeImplement::NodeImplement(int argc, char **argv) : Node(NODE_IMPLEMENT_NODE)
{
    this->_node_ptr = rclcpp::Node::make_shared("log_" NODE_IMPLEMENT_NODE);
    // set parameter
    this->callback_param();

    // set publisher
    this->_pub_value =                                      //
            this->create_publisher<std_msgs::msg::Float64>( //
                    MRS_TOPIC_TEMPLATE_OUTPUT,              //
                    rclcpp::QoS(this->CONFIG_QOS)           //
            );
    // set subscription
    this->_sub_value =                                                               //
            this->create_subscription<std_msgs::msg::Float64>(                       //
                    MRS_TOPIC_TEMPLATE_INPUT,                                        //
                    sizeof(std_msgs::msg::Float64) * this->CONFIG_SUBSCRIPTION_SIZE, //
                    std::bind(&NodeImplement::callback_message, this, _1));

    this->_ros_timer = this->create_wall_timer(this->TP_MSEC, std::bind(&NodeImplement::callback_timer, this));
}

NodeImplement::~NodeImplement()
{
}

} // namespace maid_robot_system
