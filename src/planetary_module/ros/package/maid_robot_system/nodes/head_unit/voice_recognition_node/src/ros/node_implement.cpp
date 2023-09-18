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
    this->_model.set_value(msg.data);
    // RCLCPP_DEBUG(this->get_logger(), "callback_message() : %f", this->_msg_convert.value.data);
    RCLCPP_INFO(this->get_logger(), "callback_message() : %f", this->_msg_convert.value.data);
    // RCLCPP_WARN(this->get_logger(), "callback_message() : %f", this->_msg_convert.value.data);
    // RCLCPP_ERROR(this->get_logger(), "callback_message() : %f", this->_msg_convert.value.data);
    // RCLCPP_FATAL(this->get_logger(), "callback_message() : %f", this->_msg_convert.value.data);
}

void NodeImplement::callback_param()
{
    // declare_parameter
    this->declare_parameter(this->MRS_PARAMETER_SAMPLE_TIMES, this->_model.get_times());
    this->declare_parameter(this->MRS_PARAMETER_SAMPLE_OFFSET, this->_model.get_offset());

    // make parameter callback
    this->_handle_param = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
        auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
        RCLCPP_DEBUG(this->get_logger(), "callback param");

        results->successful = false;
        results->reason     = "";

        for (auto &&param : params) {
            switch (param.get_type()) {
                case rclcpp::PARAMETER_DOUBLE:
                    if (param.get_name() == this->MRS_PARAMETER_SAMPLE_OFFSET) {
                        this->_model.set_offset(param.as_double());
                        this->_msg_convert.offset.data = this->_model.get_offset();
                        RCLCPP_INFO(this->get_logger(), "  set param : %s[%f]",this-> MRS_PARAMETER_SAMPLE_OFFSET.c_str(), this->_model.get_offset());
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_SAMPLE_TIMES) {
                        this->_model.set_times(param.as_double());
                        this->_msg_convert.times.data = this->_model.get_times();
                        RCLCPP_INFO(this->get_logger(), "  set param : %s[%f]", this->MRS_PARAMETER_SAMPLE_TIMES.c_str(), this->_model.get_times());
                        results->successful = true;
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
                    results->successful = false;
                    results->reason     = "Wrong operation";
                    break;
            }
        }
        return *results;
    });
}

void NodeImplement::callback_timer()
{
    this->_msg_convert.value.data = this->_model.calculate();
    this->_pub_info->publish(this->_msg_convert);

    static double __times  = 0;
    static double __offset = 0;
    static double __data   = 0;
    if ((__times != this->_model.get_times()) || (__offset != this->_model.get_offset()) || (__data != this->_msg_convert.value.data)) {
        RCLCPP_INFO(this->get_logger(), "callback_timer : %f = %f * n + %f", this->_msg_convert.value.data, this->_model.get_times(), this->_model.get_offset());
        __times  = this->_model.get_times();
        __offset = this->_model.get_offset();
        __data   = this->_msg_convert.value.data;
    }
}

NodeImplement::NodeImplement(std::string node_name, int argc, char **argv) : Node(node_name)
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "start.");

    // set parameter
    this->callback_param();

    // set publisher
    this->_pub_info =                                                             //
            this->create_publisher<maid_robot_system_interfaces::msg::MrsSample>( //
                    this->MRS_TOPIC_OUTPUT,                                       //
                    rclcpp::QoS(this->CONFIG_QOS)                                 //
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
