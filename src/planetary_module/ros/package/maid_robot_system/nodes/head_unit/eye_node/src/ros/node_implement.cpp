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
void NodeImplement::callback_message(const maid_robot_system_interfaces::msg::MrsEye &msg)
{
#if 0
    RCLCPP_INFO(this->get_logger(),
                "callback_message() : emotions[%d] size[%d] distance[%d] x[%d] y[%d]", //
                msg.emotions,
                msg.size,     //
                msg.distance, //
                msg.x,        //
                msg.y         //
    );
#endif
}

void NodeImplement::callback_param()
{
    // declare_parameter
    this->declare_parameter(this->MRS_PARAMETER_SETTING_FILE, "");
    this->declare_parameter(this->MRS_PARAMETER_SKIN_NAME, "");
    this->declare_parameter(this->MRS_PARAMETER_LEFT_WIDTH, 0);
    this->declare_parameter(this->MRS_PARAMETER_LEFT_HEIGHT, 0);
    this->declare_parameter(this->MRS_PARAMETER_LEFT_CENTER_X, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_LEFT_CENTER_Y, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_LEFT_CENTER_ANGLE, 0);
    this->declare_parameter(this->MRS_PARAMETER_LEFT_EYEBALL_X, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_LEFT_EYEBALL_Y, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_LEFT_EYEBALL_ANGLE, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_RIGHT_WIDTH, 0);
    this->declare_parameter(this->MRS_PARAMETER_RIGHT_HEIGHT, 0);
    this->declare_parameter(this->MRS_PARAMETER_RIGHT_CENTER_X, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_RIGHT_CENTER_Y, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_RIGHT_CENTER_ANGLE, 0);
    this->declare_parameter(this->MRS_PARAMETER_RIGHT_EYEBALL_X, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_RIGHT_EYEBALL_Y, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_RIGHT_EYEBALL_ANGLE, 0.0);
    this->declare_parameter(this->MRS_PARAMETER_EYELID_WIDTH, 0);
    this->declare_parameter(this->MRS_PARAMETER_EYELID_HEIGHT, 0);
    this->declare_parameter(this->MRS_PARAMETER_BLINK_QUICKLY_MS, 0);
    this->declare_parameter(this->MRS_PARAMETER_BLINK_MIN_MS, 0);
    this->declare_parameter(this->MRS_PARAMETER_BLINK_MAX_MS, 0);
    this->declare_parameter(this->MRS_PARAMETER_BLINK_LIMIT_MS, 0);
    this->declare_parameter(this->MRS_PARAMETER_BLINK_OFFSET_MS, 0);

    // make parameter callback
    this->_handle_param = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
        auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
        RCLCPP_DEBUG(this->get_logger(), "callback param");

        results->successful = false;
        results->reason     = "";

        for (auto &&param : params) {
            switch (param.get_type()) {
                case rclcpp::PARAMETER_INTEGER:
                    RCLCPP_INFO(this->get_logger(), "  set param : %s[%ld]", param.get_name().c_str(), param.as_int());

                    if (param.get_name() == this->MRS_PARAMETER_SETTING_FILE) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_SKIN_NAME) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_LEFT_WIDTH) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_LEFT_HEIGHT) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_LEFT_CENTER_X) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_LEFT_CENTER_Y) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_LEFT_CENTER_ANGLE) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_LEFT_EYEBALL_X) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_LEFT_EYEBALL_Y) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_LEFT_EYEBALL_ANGLE) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_RIGHT_WIDTH) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_RIGHT_HEIGHT) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_RIGHT_CENTER_X) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_RIGHT_CENTER_Y) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_RIGHT_CENTER_ANGLE) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_RIGHT_EYEBALL_X) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_RIGHT_EYEBALL_Y) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_RIGHT_EYEBALL_ANGLE) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_EYELID_WIDTH) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_EYELID_HEIGHT) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_BLINK_QUICKLY_MS) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_BLINK_MIN_MS) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_BLINK_MAX_MS) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_BLINK_LIMIT_MS) {
                        results->successful = true;
                    } else if (param.get_name() == this->MRS_PARAMETER_BLINK_OFFSET_MS) {
                        results->successful = true;
                    }
                    break;
                case rclcpp::PARAMETER_DOUBLE:
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

    // set publisher
    this->_pub_value =                                      //
            this->create_publisher<std_msgs::msg::Float64>( //
                    this->MRS_TOPIC_OUTPUT,                 //
                    rclcpp::QoS(this->CONFIG_QOS)           //
            );
    // set subscription
    this->_sub_value =                                                            //
            this->create_subscription<maid_robot_system_interfaces::msg::MrsEye>( //
                    this->MRS_TOPIC_INPUT,                                        //
                    this->CONFIG_SUBSCRIPTION_SIZE,                               //
                    std::bind(&NodeImplement::callback_message, this, _1));

    this->_ros_timer = this->create_wall_timer(this->TP_MSEC, std::bind(&NodeImplement::callback_timer, this));
}

NodeImplement::~NodeImplement()
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "fin.");
}

} // namespace maid_robot_system
