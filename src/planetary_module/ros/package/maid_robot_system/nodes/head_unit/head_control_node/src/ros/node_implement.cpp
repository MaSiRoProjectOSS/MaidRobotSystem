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
            RCLCPP_DEBUG(this->get_logger(), "[%s] : %ld", this->get_name(), msg.ids[i]);
            this->_model.set_value_ar(ModelStructure::INPUT_TYPE::AR_LEFT, msg.ids[i]);
        }
    }
}
void NodeImplement::callback_ar_right(const maid_robot_system_interfaces::msg::ArMarkers &msg)
{
    for (long unsigned int i = 0; i < msg.ids.size(); i++) {
        if (0 != msg.ids[i]) {
            RCLCPP_DEBUG(this->get_logger(), "[%s] : %ld", this->get_name(), msg.ids[i]);
            this->_model.set_value_ar(ModelStructure::INPUT_TYPE::AR_RIGHT, msg.ids[i]);
        }
    }
}
void NodeImplement::callback_voice(const maid_robot_system_interfaces::msg::MrsVoice &msg)
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %d", this->get_name(), msg.command);
    this->_model.set_value_voice(msg.text, msg.command);
}
void NodeImplement::callback_voltage(const std_msgs::msg::Float64 &msg)
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %f", this->get_name(), msg.data);
    this->_model.set_value_tiredness(msg.data);
}

// =============================
// ROS : parameter
// =============================
void NodeImplement::callback_param()
{
    // declare_parameter
    this->declare_parameter(this->MRS_PARAM_EYE_CENTER_X_LEFT, this->_model.param.oculus_center_x_left);
    this->declare_parameter(this->MRS_PARAM_EYE_CENTER_Y_LEFT, this->_model.param.oculus_center_y_left);
    this->declare_parameter(this->MRS_PARAM_EYE_OFFSET_UP_LEFT, this->_model.param.oculus_offset_up_left);
    this->declare_parameter(this->MRS_PARAM_EYE_OFFSET_DOWN_LEFT, this->_model.param.oculus_offset_down_left);

    this->declare_parameter(this->MRS_PARAM_EYE_CENTER_X_RIGHT, this->_model.param.oculus_center_x_right);
    this->declare_parameter(this->MRS_PARAM_EYE_CENTER_Y_RIGHT, this->_model.param.oculus_center_y_right);
    this->declare_parameter(this->MRS_PARAM_EYE_OFFSET_UP_RIGHT, this->_model.param.oculus_offset_up_right);
    this->declare_parameter(this->MRS_PARAM_EYE_OFFSET_DOWN_RIGHT, this->_model.param.oculus_offset_down_right);

    this->declare_parameter(this->MRS_PARAM_NECK_PITCH_MIN, this->_model.param.neck_pitch_min);
    this->declare_parameter(this->MRS_PARAM_NECK_PITCH_MAX, this->_model.param.neck_pitch_max);
    this->declare_parameter(this->MRS_PARAM_NECK_YAW_MIN, this->_model.param.neck_yaw_min);
    this->declare_parameter(this->MRS_PARAM_NECK_YAW_MAX, this->_model.param.neck_yaw_max);
    this->declare_parameter(this->MRS_PARAM_NECK_ROLL_MIN, this->_model.param.neck_roll_min);
    this->declare_parameter(this->MRS_PARAM_NECK_ROLL_MAX, this->_model.param.neck_roll_max);

    this->declare_parameter(this->MRS_PARAM_LIP_MIN, this->_model.param.lip_min);
    this->declare_parameter(this->MRS_PARAM_LIP_MAX, this->_model.param.lip_max);

    // make parameter callback
    this->_handle_param = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
        auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
        RCLCPP_DEBUG(this->get_logger(), "callback param");

        results->successful = false;
        results->reason     = "";

        for (auto &&param : params) {
            switch (param.get_type()) {
                case rclcpp::PARAMETER_DOUBLE:
                    if (param.get_name() == this->MRS_PARAM_EYE_CENTER_X_LEFT) {
                        this->_model.param.oculus_center_x_left = param.as_double();
                        results->successful                     = true;
                    } else if (param.get_name() == this->MRS_PARAM_EYE_CENTER_Y_LEFT) {
                        this->_model.param.oculus_center_y_left = param.as_double();
                        results->successful                     = true;
                    } else if (param.get_name() == this->MRS_PARAM_EYE_OFFSET_UP_LEFT) {
                        this->_model.param.oculus_offset_up_left = param.as_double();
                        results->successful                      = true;
                    } else if (param.get_name() == this->MRS_PARAM_EYE_OFFSET_DOWN_LEFT) {
                        this->_model.param.oculus_offset_down_left = param.as_double();
                        results->successful                        = true;
                    } else if (param.get_name() == this->MRS_PARAM_EYE_CENTER_X_RIGHT) {
                        this->_model.param.oculus_center_x_right = param.as_double();
                        results->successful                      = true;
                    } else if (param.get_name() == this->MRS_PARAM_EYE_CENTER_Y_RIGHT) {
                        this->_model.param.oculus_center_y_right = param.as_double();
                        results->successful                      = true;
                    } else if (param.get_name() == this->MRS_PARAM_EYE_OFFSET_UP_RIGHT) {
                        this->_model.param.oculus_offset_up_right = param.as_double();
                        results->successful                       = true;
                    } else if (param.get_name() == this->MRS_PARAM_EYE_OFFSET_DOWN_RIGHT) {
                        this->_model.param.oculus_offset_down_right = param.as_double();
                        results->successful                         = true;
                    }
                    break;
                case rclcpp::PARAMETER_INTEGER:

                    if (param.get_name() == this->MRS_PARAM_NECK_PITCH_MIN) {
                        this->_model.param.neck_pitch_min = param.as_int();
                        results->successful               = true;
                    } else if (param.get_name() == this->MRS_PARAM_NECK_PITCH_MAX) {
                        this->_model.param.neck_pitch_max = param.as_int();
                        results->successful               = true;
                    } else if (param.get_name() == this->MRS_PARAM_NECK_YAW_MIN) {
                        this->_model.param.neck_yaw_min = param.as_int();
                        results->successful             = true;
                    } else if (param.get_name() == this->MRS_PARAM_NECK_YAW_MAX) {
                        this->_model.param.neck_yaw_max = param.as_int();
                        results->successful             = true;
                    } else if (param.get_name() == this->MRS_PARAM_NECK_ROLL_MIN) {
                        this->_model.param.neck_roll_min = param.as_int();
                        results->successful              = true;
                    } else if (param.get_name() == this->MRS_PARAM_NECK_ROLL_MAX) {
                        this->_model.param.neck_roll_max = param.as_int();
                        results->successful              = true;
                    } else if (param.get_name() == this->MRS_PARAM_LIP_MIN) {
                        this->_model.param.lip_min = param.as_int();
                        results->successful        = true;
                    } else if (param.get_name() == this->MRS_PARAM_LIP_MAX) {
                        this->_model.param.lip_max = param.as_int();
                        results->successful        = true;
                    }

                    break;
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

// =============================
// ROS : loop function
// =============================
void NodeImplement::callback_timer()
{
    bool result = this->_model.calculate();
    if (true == result) {
        this->_model.get_msg_eye(this->_msg_eye);
        this->_model.get_msg_neck(this->_msg_neck);
        this->_model.get_msg_lip(this->_msg_lip);
    }
    this->_pub_eye->publish(this->_msg_eye);
    this->_pub_neck->publish(this->_msg_neck);
    this->_pub_lip->publish(this->_msg_lip);
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
    this->_pub_eye =                                                           //
            this->create_publisher<maid_robot_system_interfaces::msg::MrsEye>( //
                    this->MRS_TOPIC_OUT_EYE,                                   //
                    rclcpp::QoS(this->CONFIG_QOS)                              //
            );
    this->_pub_neck =                                                           //
            this->create_publisher<maid_robot_system_interfaces::msg::MrsNeck>( //
                    this->MRS_TOPIC_OUT_NECK,                                   //
                    rclcpp::QoS(this->CONFIG_QOS)                               //
            );
    this->_pub_lip =                                                           //
            this->create_publisher<maid_robot_system_interfaces::msg::MrsLip>( //
                    this->MRS_TOPIC_OUT_LIP,                                   //
                    rclcpp::QoS(this->CONFIG_QOS)                              //
            );

    // set subscription
    this->_sub_pose_left =                                                               //
            this->create_subscription<maid_robot_system_interfaces::msg::PoseDetection>( //
                    this->MRS_TOPIC_IN_POSTURE_LEFT,                                     //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                      //
                    std::bind(&NodeImplement::callback_pose_left, this, _1));
    this->_sub_pose_right =                                                              //
            this->create_subscription<maid_robot_system_interfaces::msg::PoseDetection>( //
                    this->MRS_TOPIC_IN_POSTURE_RIGHT,                                    //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                      //
                    std::bind(&NodeImplement::callback_pose_right, this, _1));
    this->_sub_ar_left =                                                             //
            this->create_subscription<maid_robot_system_interfaces::msg::ArMarkers>( //
                    this->MRS_TOPIC_IN_MARKS_LEFT,                                   //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                  //
                    std::bind(&NodeImplement::callback_ar_left, this, _1));
    this->_sub_ar_right =                                                            //
            this->create_subscription<maid_robot_system_interfaces::msg::ArMarkers>( //
                    this->MRS_TOPIC_IN_MARKS_RIGHT,                                  //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                  //
                    std::bind(&NodeImplement::callback_ar_right, this, _1));
    this->_sub_voice =                                                              //
            this->create_subscription<maid_robot_system_interfaces::msg::MrsVoice>( //
                    this->MRS_TOPIC_IN_VOICE,                                       //
                    this->CONFIG_SUBSCRIPTION_SIZE,                                 //
                    std::bind(&NodeImplement::callback_voice, this, _1));
    this->_sub_voltage =                                       //
            this->create_subscription<std_msgs::msg::Float64>( //
                    this->MRS_TOPIC_IN_VOLTAGE,                //
                    this->CONFIG_SUBSCRIPTION_SIZE,            //
                    std::bind(&NodeImplement::callback_voltage, this, _1));

    this->_ros_timer = this->create_wall_timer(this->TP_MSEC, std::bind(&NodeImplement::callback_timer, this));
}

NodeImplement::~NodeImplement()
{
    RCLCPP_DEBUG(this->get_logger(), "[%s] : %s", this->get_name(), "fin.");
}

} // namespace maid_robot_system
