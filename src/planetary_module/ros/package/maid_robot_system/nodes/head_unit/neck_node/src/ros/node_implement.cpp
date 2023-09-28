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
void NodeImplement::callback_data(const maid_robot_system_interfaces::msg::MrsNeck &msg)
{
    if (true == this->_model.calculate(msg.x, msg.y, msg.z, msg.w)) {
        RCLCPP_INFO(this->get_logger(), "callback_message() : %d, %d, %d, %d", msg.x, msg.y, msg.z, msg.w);
    }
}

void NodeImplement::callback_param()
{
    // declare_parameter
    // this->declare_parameter(this->MRS_PARAMETER_SAMPLE_OFFSET, this->_model.get_offset());

    // make parameter callback
    this->_handle_param = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
        auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
        RCLCPP_DEBUG(this->get_logger(), "callback param");

        results->successful = false;
        results->reason     = "";

        for (auto &&param : params) {
            switch (param.get_type()) {
                case rclcpp::PARAMETER_DOUBLE:
                    // if (param.get_name() == this->MRS_PARAMETER_SAMPLE_OFFSET) {
                    //     this->_model.set_offset(param.as_double());
                    //     this->_msg_convert.data = this->_model.get_offset();
                    //     RCLCPP_INFO(this->get_logger(), "  set param : %s[%f]", this->MRS_PARAMETER_SAMPLE_OFFSET.c_str(), this->_model.get_offset());
                    //     results->successful = true;
                    // }
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
    //
}

NodeImplement::NodeImplement(std::string node_name, int argc, char **argv) : Node(node_name)
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "start.");

    // set parameter
    this->callback_param();

    // set subscription
    this->_sub_neck =                                                              //
            this->create_subscription<maid_robot_system_interfaces::msg::MrsNeck>( //
                    this->MRS_TOPIC_INPUT,                                         //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                //
                    std::bind(&NodeImplement::callback_data, this, _1));

    this->_ros_timer = this->create_wall_timer(this->TP_MSEC, std::bind(&NodeImplement::callback_timer, this));
}

NodeImplement::~NodeImplement()
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "fin.");
}

} // namespace maid_robot_system
