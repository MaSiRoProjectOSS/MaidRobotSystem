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
ModelImplement *g_model;

void NodeImplement::_callback_msg_mrs_eye(const maid_robot_system_interfaces::msg::MrsEye &msg)
{
    (void)g_model->set_msg_eye(msg.emotions,
                               msg.pupil_effect,

                               msg.size,
                               msg.distance,

                               msg.left_x,
                               msg.left_y,
                               msg.right_x,
                               msg.right_y);
}

void NodeImplement::_callback_param_init()
{
    // declare_parameter
    this->declare_parameter(this->MRS_PARAMETER_SETTING_FILE, g_model->get_setting_file());
    this->declare_parameter(this->MRS_PARAMETER_BRIGHTNESS, g_model->get_brightness());
    this->declare_parameter(this->MRS_PARAMETER_COLOR_R, g_model->get_color_r());
    this->declare_parameter(this->MRS_PARAMETER_COLOR_G, g_model->get_color_g());
    this->declare_parameter(this->MRS_PARAMETER_COLOR_B, g_model->get_color_b());

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    g_model->set_brightness(this->get_parameter(this->MRS_PARAMETER_BRIGHTNESS).as_int());
    g_model->set_color_r(this->get_parameter(this->MRS_PARAMETER_COLOR_R).as_int());
    g_model->set_color_g(this->get_parameter(this->MRS_PARAMETER_COLOR_G).as_int());
    g_model->set_color_b(this->get_parameter(this->MRS_PARAMETER_COLOR_B).as_int());
    g_model->set_setting_file(this->get_parameter(this->MRS_PARAMETER_SETTING_FILE).as_string());

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // make parameter callback
    this->_handle_param = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
        auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
        RCLCPP_DEBUG(this->get_logger(), "callback param");

        results->successful = false;
        results->reason     = "";

        for (auto &&param : params) {
            switch (param.get_type()) {
                case rclcpp::PARAMETER_STRING:
                    if (param.get_name() == this->MRS_PARAMETER_SETTING_FILE) {
                        results->successful = g_model->set_setting_file(param.as_string());
                    }
                    break;
                case rclcpp::PARAMETER_INTEGER:
                    if (param.get_name() == this->MRS_PARAMETER_BRIGHTNESS) {
                        results->successful = g_model->set_brightness(param.as_int());
                    } else if (param.get_name() == this->MRS_PARAMETER_COLOR_R) {
                        results->successful = g_model->set_color_r(param.as_int());
                    } else if (param.get_name() == this->MRS_PARAMETER_COLOR_G) {
                        results->successful = g_model->set_color_g(param.as_int());
                    } else if (param.get_name() == this->MRS_PARAMETER_COLOR_B) {
                        results->successful = g_model->set_color_b(param.as_int());
                    }
                    break;
                case rclcpp::PARAMETER_DOUBLE:
                case rclcpp::PARAMETER_NOT_SET:
                case rclcpp::PARAMETER_BOOL:
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

void NodeImplement::_callback_timer()
{
    (void)g_model->calculate();
}

#if DEBUG_OUTPUT_FPS
void NodeImplement::_callback_output_state()
{
    RCLCPP_INFO(this->get_logger(), "\n%s", g_model->get_lap_time().c_str());
}
#endif

NodeImplement::NodeImplement(std::string node_name, int argc, char **argv) : Node(node_name)
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "start.");

    // set parameter
    this->_callback_param_init();

    // set subscription
    this->_sub_mrs_eye =                                                          //
            this->create_subscription<maid_robot_system_interfaces::msg::MrsEye>( //
                    this->MRS_TOPIC_INPUT,                                        //
                    this->CONFIG_SUBSCRIPTION_SIZE,                               //
                    std::bind(&NodeImplement::_callback_msg_mrs_eye, this, _1));

    this->_ros_timer = this->create_wall_timer(this->TP_MSEC, std::bind(&NodeImplement::_callback_timer, this));
#if DEBUG_OUTPUT_FPS
    this->_ros_output_state = this->create_wall_timer(this->TP_OUTPUT_STATE_MSEC, std::bind(&NodeImplement::_callback_output_state, this));
#endif

    g_model = new ModelImplement();
    g_model->open(argc, argv);
}

NodeImplement::~NodeImplement()
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "fin.");
}

} // namespace maid_robot_system
