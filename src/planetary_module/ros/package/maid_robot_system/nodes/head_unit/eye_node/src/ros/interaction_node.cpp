/**
 * @file  node_implement.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "ros/interaction_node.hpp"

#include "ros/widget_node.hpp"

#include <exception>
#include <stdexcept>
#include <stdio.h>
#include <unistd.h>

using std::placeholders::_1;

namespace maid_robot_system
{
// =============================
// Constructor
// =============================
InteractionNode::InteractionNode(std::string node_name, WidgetNode &widget) : Node(node_name)
{
#if LOGGER_ROS_INFO_CALL_FUNCTION
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_CALL_FUNCTION, "[%s] : %s", this->get_name(), "start.");
#endif
    this->_widget = &widget;
    if (nullptr != this->_widget) {
        // set parameter
        this->_callback_param_init();

        // set subscription
        this->_sub_mrs_eye =                                                          //
                this->create_subscription<maid_robot_system_interfaces::msg::MrsEye>( //
                        this->MRS_TOPIC_INPUT,                                        //
                        this->CONFIG_SUBSCRIPTION_SIZE,                               //
                        std::bind(&InteractionNode::_callback_msg_mrs_eye, this, _1));

        this->_ros_timer = this->create_wall_timer(this->TP_MSEC, std::bind(&InteractionNode::_callback_timer, this));
#if LOGGER_ROS_INFO_OUTPUT_REPORT_TIME > 0
        this->_ros_output_state = this->create_wall_timer(this->TP_OUTPUT_STATE_MSEC, std::bind(&InteractionNode::_callback_output_state, this));
#endif
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open.");
        throw new std::runtime_error("Failed to open.");
    }
}

InteractionNode::~InteractionNode()
{
#if LOGGER_ROS_INFO_CALL_FUNCTION
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_CALL_FUNCTION, "[%s] : %s", this->get_name(), "fin.");
#endif
}

// =============================
// ROS : parameter
// =============================
void InteractionNode::_callback_param_init()
{
    // declare_parameter
    this->declare_parameter(this->MRS_PARAMETER_SETTING_FILE, this->_widget->get_setting_file());
    this->declare_parameter(this->MRS_PARAMETER_BRIGHTNESS, this->_widget->get_brightness());
    this->declare_parameter(this->MRS_PARAMETER_EYELID_COLOR_R, this->_widget->get_eyelid_color_r());
    this->declare_parameter(this->MRS_PARAMETER_EYELID_COLOR_G, this->_widget->get_eyelid_color_g());
    this->declare_parameter(this->MRS_PARAMETER_EYELID_COLOR_B, this->_widget->get_eyelid_color_b());
    this->declare_parameter(this->MRS_PARAMETER_CILIARY_COLOR_R, this->_widget->get_ciliary_color_r());
    this->declare_parameter(this->MRS_PARAMETER_CILIARY_COLOR_G, this->_widget->get_ciliary_color_g());
    this->declare_parameter(this->MRS_PARAMETER_CILIARY_COLOR_B, this->_widget->get_ciliary_color_b());

    this->declare_parameter(this->MRS_PARAMETER_NOTIFY_MESSAGE_ENABLE, this->_is_notify_enable);
    this->declare_parameter(this->MRS_PARAMETER_NOTIFY_MESSAGE_VERBOSE, this->_is_notify_verbose);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    this->_widget->set_brightness(this->get_parameter(this->MRS_PARAMETER_BRIGHTNESS).as_int());
    this->_widget->set_eyelid_color_r(this->get_parameter(this->MRS_PARAMETER_EYELID_COLOR_R).as_int());
    this->_widget->set_eyelid_color_g(this->get_parameter(this->MRS_PARAMETER_EYELID_COLOR_G).as_int());
    this->_widget->set_eyelid_color_b(this->get_parameter(this->MRS_PARAMETER_EYELID_COLOR_B).as_int());
    this->_widget->set_ciliary_color_r(this->get_parameter(this->MRS_PARAMETER_CILIARY_COLOR_R).as_int());
    this->_widget->set_ciliary_color_g(this->get_parameter(this->MRS_PARAMETER_CILIARY_COLOR_G).as_int());
    this->_widget->set_ciliary_color_b(this->get_parameter(this->MRS_PARAMETER_CILIARY_COLOR_B).as_int());
    bool flag_load_file = this->_set_setting_file(this->get_parameter(this->MRS_PARAMETER_SETTING_FILE).as_string());
    if (false == flag_load_file) {
        RCLCPP_WARN(this->get_logger(), "Not found setting file : %s", this->get_parameter(this->MRS_PARAMETER_SETTING_FILE).as_string().c_str());
    }
    this->_is_notify_enable  = this->get_parameter(this->MRS_PARAMETER_NOTIFY_MESSAGE_ENABLE).as_bool();
    this->_is_notify_verbose = this->get_parameter(this->MRS_PARAMETER_NOTIFY_MESSAGE_VERBOSE).as_bool();
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // make parameter callback
    this->_handle_param = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
        auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

        results->successful = false;
        results->reason     = "";

        for (auto &&param : params) {
#if LOGGER_ROS_INFO_PARAMETER
            RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_PARAMETER, "get parameter : %s", param.get_name().c_str());
#endif
            switch (param.get_type()) {
                case rclcpp::PARAMETER_STRING:
                    if (param.get_name() == this->MRS_PARAMETER_SETTING_FILE) {
                        results->successful = this->_set_setting_file(param.as_string());
                        if (false == results->successful) {
                            RCLCPP_WARN(this->get_logger(), "Not found setting file : %s", param.as_string().c_str());
                        }
                    }
                    break;
                case rclcpp::PARAMETER_INTEGER:
                    if (param.get_name() == this->MRS_PARAMETER_BRIGHTNESS) {
                        results->successful = this->_widget->set_brightness(param.as_int());
                    } else if (param.get_name() == this->MRS_PARAMETER_EYELID_COLOR_R) {
                        results->successful = this->_widget->set_eyelid_color_r(param.as_int());
                    } else if (param.get_name() == this->MRS_PARAMETER_EYELID_COLOR_G) {
                        results->successful = this->_widget->set_eyelid_color_g(param.as_int());
                    } else if (param.get_name() == this->MRS_PARAMETER_EYELID_COLOR_B) {
                        results->successful = this->_widget->set_eyelid_color_b(param.as_int());
                    } else if (param.get_name() == this->MRS_PARAMETER_CILIARY_COLOR_R) {
                        results->successful = this->_widget->set_ciliary_color_r(param.as_int());
                    } else if (param.get_name() == this->MRS_PARAMETER_CILIARY_COLOR_G) {
                        results->successful = this->_widget->set_ciliary_color_g(param.as_int());
                    } else if (param.get_name() == this->MRS_PARAMETER_CILIARY_COLOR_B) {
                        results->successful = this->_widget->set_ciliary_color_b(param.as_int());
                    }
                    break;
                case rclcpp::PARAMETER_BOOL:
                    if (param.get_name() == this->MRS_PARAMETER_NOTIFY_MESSAGE_ENABLE) {
#if LOGGER_ROS_INFO_OUTPUT_REPORT_TIME > 0
                        this->_is_notify_enable = param.as_bool();
                        results->successful     = true;
#endif
                    } else if (param.get_name() == this->MRS_PARAMETER_NOTIFY_MESSAGE_VERBOSE) {
#if LOGGER_ROS_INFO_OUTPUT_REPORT_TIME > 0
                        this->_is_notify_verbose = param.as_bool();
                        results->successful      = true;
#endif
                    }
                    break;
                case rclcpp::PARAMETER_DOUBLE:
                case rclcpp::PARAMETER_NOT_SET:
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

// =============================
// ROS : subscription
// =============================
void InteractionNode::_callback_msg_mrs_eye(const maid_robot_system_interfaces::msg::MrsEye &msg)
{
#if LOGGER_ROS_INFO_SUBSCRIPTION_MRS_EYE
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_SUBSCRIPTION_MRS_EYE, "get message : MrsEye.msg");
#endif
    switch (msg.cornea_effect) {
        case maid_robot_system_interfaces::msg::MrsEye::EFFECT_CORNEA_ORDER:
            (void)this->_widget->effect_cornea_order();
            break;

        case maid_robot_system_interfaces::msg::MrsEye::EFFECT_CORNEA_NORMAL:
        default:
            break;
    }
    switch (msg.emotions) {
        case maid_robot_system_interfaces::msg::MrsEye::EMOTION_CLOSE:
            this->_widget->emotion(MIENS::miens_close);
            break;
        case maid_robot_system_interfaces::msg::MrsEye::EMOTION_SMILE:
            this->_widget->emotion(MIENS::miens_smile);
            break;
        case maid_robot_system_interfaces::msg::MrsEye::EMOTION_CLOSE_LEFT:
            this->_widget->emotion(MIENS::miens_close_left);
            break;
        case maid_robot_system_interfaces::msg::MrsEye::EMOTION_CLOSE_RIGHT:
            this->_widget->emotion(MIENS::miens_close_right);
            break;
        case maid_robot_system_interfaces::msg::MrsEye::EMOTION_WINK_LEFT:
            this->_widget->emotion(MIENS::miens_wink_left);
            break;
        case maid_robot_system_interfaces::msg::MrsEye::EMOTION_WINK_RIGHT:
            this->_widget->emotion(MIENS::miens_wink_right);
            break;
        case maid_robot_system_interfaces::msg::MrsEye::EMOTION_NORMAL:
        default:
            this->_widget->emotion(MIENS::miens_normal);
            break;
    }
    (void)this->_widget->set_msg_eye(msg.size,
                                     msg.distance,

                                     msg.left_y,
                                     msg.left_z,
                                     msg.right_y,
                                     msg.right_z);
}

// =============================
// ROS : loop function
// =============================
void InteractionNode::_callback_timer()
{
#if LOGGER_ROS_INFO_DETAIL
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_DETAIL, "[%s] : %s", this->get_name(), "calculate");
#endif
    (void)this->_widget->request_update();
}

#if LOGGER_ROS_INFO_OUTPUT_REPORT_TIME > 0
void InteractionNode::_callback_output_state()
{
    RCLCPP_INFO_EXPRESSION(this->get_logger(), this->_is_notify_enable, "%s", this->_widget->output_message(this->_is_notify_verbose).c_str());
}
#endif

// =============================
// PRIVATE : Function
// =============================
bool InteractionNode::_set_setting_file(std::string value)
{
    bool result = false;
    try {
        result = this->_widget->set_setting_file(value);
    } catch (const std::logic_error &err) {
        result = false;
        RCLCPP_ERROR(this->get_logger(), err.what());
    } catch (...) {
        result = false;
        RCLCPP_ERROR(this->get_logger(), "Parsing of setting file failed.");
    }
    return result;
}

} // namespace maid_robot_system
