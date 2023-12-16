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
int NodeImplement::_callback_param_init()
{
    int result = 1;
    try {
        // declare_parameter
        //this->declare_parameter(this->MRS_PARAMETER_SAMPLE_TIMES, this->_model.get_offset());

        // make parameter callback
        this->_handle_param = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
            auto results        = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
            results->successful = false;
            results->reason     = "";

            for (auto &&param : params) {
#if LOGGER_ROS_INFO_PARAMETER
                RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_PARAMETER, "get parameter : %s", param.get_name().c_str());
#endif
                switch (param.get_type()) {
                    case rclcpp::PARAMETER_DOUBLE:
                        //if (param.get_name() == this->MRS_PARAMETER_SAMPLE_TIMES) {
                        //    this->_model.set_offset(param.as_double());
                        //    this->_msg_convert.offset.data = this->_model.get_offset();
                        //    RCLCPP_INFO(this->get_logger(), "  set param : %s[%f]", this->MRS_PARAMETER_SAMPLE_TIMES.c_str(), this->_model.get_offset());
                        //    results->successful = true;
                        //}
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
        result = 0;
    } catch (char *e) {
        RCLCPP_ERROR(this->get_logger(), "<ERROR> %s", e);
    } catch (const std::logic_error &l_err) {
        RCLCPP_ERROR(this->get_logger(), "<LOGIC_ERROR> %s", l_err.what());
    } catch (const std::runtime_error &r_err) {
        RCLCPP_ERROR(this->get_logger(), "<RUNTIME_ERROR> %s", r_err.what());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "<ERROR> %s", "An unknown error has occurred.");
    }
    return result;
}

void NodeImplement::_callback_timer()
{
    if (true == this->_model.is_text()) {
        this->_msg_text.data = this->_model.pop();
        RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_TIMER, "text : %s", this->_msg_text.data.c_str());
        this->_pub_text->publish(this->_msg_text);
    }
}

NodeImplement::NodeImplement(std::string node_name, int argc, char **argv) : Node(node_name)
{
#if LOGGER_ROS_INFO_CALL_FUNCTION
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_CALL_FUNCTION, "[%s] : %s", this->get_name(), "start.");
#endif

    // set parameter
    if (0 == this->_callback_param_init()) {
        // set publisher
        this->_pub_text = this->create_publisher<std_msgs::msg::String>( //
                this->MRS_OUTPUT_VOICE_TEXT,                             //
                this->DEPTH_PUBLISHER                                    //
        );

        this->_ros_timer = this->create_wall_timer(this->PERIOD_MSEC, std::bind(&NodeImplement::_callback_timer, this));
    } else {
        throw new std::runtime_error("Failed init param.");
    }
}

NodeImplement::~NodeImplement()
{
#if LOGGER_ROS_INFO_CALL_FUNCTION
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_CALL_FUNCTION, "[%s] : %s", this->get_name(), "fin.");
#endif
}

} // namespace maid_robot_system
