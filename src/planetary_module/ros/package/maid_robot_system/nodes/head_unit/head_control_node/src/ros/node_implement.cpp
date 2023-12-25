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
void NodeImplement::_callback_pose_left(const maid_robot_system_interfaces::msg::PoseDetection &msg)
{
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_GET_MESSAGE, "[%s](%d)", this->get_name(), (int)ModelStructure::INPUT_TYPE::POSE_LEFT);
    this->_model.set_value_pose(ModelStructure::INPUT_TYPE::POSE_LEFT, msg, this->get_clock()->now().seconds());
}
void NodeImplement::_callback_pose_right(const maid_robot_system_interfaces::msg::PoseDetection &msg)
{
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_GET_MESSAGE, "[%s](%d)", this->get_name(), (int)ModelStructure::INPUT_TYPE::POSE_RIGHT);
    this->_model.set_value_pose(ModelStructure::INPUT_TYPE::POSE_RIGHT, msg, this->get_clock()->now().seconds());
}
void NodeImplement::_callback_ar_left(const maid_robot_system_interfaces::msg::ArMarkers &msg)
{
    for (long unsigned int i = 0; i < msg.ids.size(); i++) {
        if (0 != msg.ids[i]) {
            RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_GET_MESSAGE, "[%s] : %ld", this->get_name(), msg.ids[i]);
            this->_model.set_value_ar(ModelStructure::INPUT_TYPE::AR_LEFT, msg.ids[i], this->get_clock()->now().seconds());
        }
    }
}
void NodeImplement::_callback_ar_right(const maid_robot_system_interfaces::msg::ArMarkers &msg)
{
    for (long unsigned int i = 0; i < msg.ids.size(); i++) {
        if (0 != msg.ids[i]) {
            RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_GET_MESSAGE, "[%s] : %ld", this->get_name(), msg.ids[i]);
            this->_model.set_value_ar(ModelStructure::INPUT_TYPE::AR_RIGHT, msg.ids[i], this->get_clock()->now().seconds());
        }
    }
}
void NodeImplement::_callback_voice(const maid_robot_system_interfaces::msg::MrsVoice &msg)
{
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_GET_MESSAGE, "[%s] : %d", this->get_name(), msg.command);
    this->_model.set_value_voice(msg.text, msg.command, this->get_clock()->now().seconds());
}
void NodeImplement::_callback_voltage(const std_msgs::msg::Float64 &msg)
{
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_GET_MESSAGE, "[%s] : %f", this->get_name(), msg.data);
    this->_model.set_value_tiredness(msg.data, this->get_clock()->now().seconds());
}

// =============================
// ROS : parameter
// =============================
int NodeImplement::_callback_param_init()
{
    int result = 1;
    try {
        // declare_parameter
        this->declare_parameter(this->MRS_PARAM_EYE_LEFT_OFFSET_X, this->_model.param.eye_left_offset_x);
        this->declare_parameter(this->MRS_PARAM_EYE_LEFT_OFFSET_Y, this->_model.param.eye_left_offset_y);
        this->declare_parameter(this->MRS_PARAM_EYE_LEFT_OFFSET_ANGLE, this->_model.param.eye_left_offset_angle);

        this->declare_parameter(this->MRS_PARAM_EYE_RIGHT_OFFSET_X, this->_model.param.eye_right_offset_x);
        this->declare_parameter(this->MRS_PARAM_EYE_RIGHT_OFFSET_Y, this->_model.param.eye_right_offset_y);
        this->declare_parameter(this->MRS_PARAM_EYE_RIGHT_OFFSET_ANGLE, this->_model.param.eye_right_offset_angle);

        this->declare_parameter(this->MRS_PARAM_NECK_PITCH_MIN, this->_model.param.neck_pitch_min);
        this->declare_parameter(this->MRS_PARAM_NECK_PITCH_MAX, this->_model.param.neck_pitch_max);
        this->declare_parameter(this->MRS_PARAM_NECK_YAW_MIN, this->_model.param.neck_yaw_min);
        this->declare_parameter(this->MRS_PARAM_NECK_YAW_MAX, this->_model.param.neck_yaw_max);
        this->declare_parameter(this->MRS_PARAM_NECK_ROLL_MIN, this->_model.param.neck_roll_min);
        this->declare_parameter(this->MRS_PARAM_NECK_ROLL_MAX, this->_model.param.neck_roll_max);

        this->declare_parameter(this->MRS_PARAM_LIP_MIN, this->_model.param.lip_min);
        this->declare_parameter(this->MRS_PARAM_LIP_MAX, this->_model.param.lip_max);

        this->declare_parameter(this->MRS_PARAM_TIREDNESS, this->_model.param.tiredness);
        this->declare_parameter(this->MRS_PARAM_PRIORITY_TO_THE_RIGHT, this->_model.param.priority_to_the_right);

        this->declare_parameter(this->MRS_PARAM_TIMEOUT_S_RECEIVED, this->_model.param.timeout_s_received);
        this->declare_parameter(this->MRS_PARAM_TIMEOUT_S_CHASED, this->_model.param.timeout_s_chased);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////

        this->_model.param.eye_left_offset_x     = this->get_parameter(this->MRS_PARAM_EYE_LEFT_OFFSET_X).get_parameter_value().get<double>();
        this->_model.param.eye_left_offset_y     = this->get_parameter(this->MRS_PARAM_EYE_LEFT_OFFSET_Y).get_parameter_value().get<double>();
        this->_model.param.eye_left_offset_angle = this->get_parameter(this->MRS_PARAM_EYE_LEFT_OFFSET_ANGLE).get_parameter_value().get<double>();

        this->_model.param.eye_right_offset_x     = this->get_parameter(this->MRS_PARAM_EYE_RIGHT_OFFSET_X).get_parameter_value().get<double>();
        this->_model.param.eye_right_offset_y     = this->get_parameter(this->MRS_PARAM_EYE_RIGHT_OFFSET_Y).get_parameter_value().get<double>();
        this->_model.param.eye_right_offset_angle = this->get_parameter(this->MRS_PARAM_EYE_RIGHT_OFFSET_ANGLE).get_parameter_value().get<double>();

        this->_model.param.neck_pitch_min = this->get_parameter(this->MRS_PARAM_NECK_PITCH_MIN).get_parameter_value().get<int>();
        this->_model.param.neck_pitch_max = this->get_parameter(this->MRS_PARAM_NECK_PITCH_MAX).get_parameter_value().get<int>();
        this->_model.param.neck_yaw_min   = this->get_parameter(this->MRS_PARAM_NECK_YAW_MIN).get_parameter_value().get<int>();
        this->_model.param.neck_yaw_max   = this->get_parameter(this->MRS_PARAM_NECK_YAW_MAX).get_parameter_value().get<int>();
        this->_model.param.neck_roll_min  = this->get_parameter(this->MRS_PARAM_NECK_ROLL_MIN).get_parameter_value().get<int>();
        this->_model.param.neck_roll_max  = this->get_parameter(this->MRS_PARAM_NECK_ROLL_MAX).get_parameter_value().get<int>();

        this->_model.param.lip_min = this->get_parameter(this->MRS_PARAM_LIP_MIN).get_parameter_value().get<int>();
        this->_model.param.lip_max = this->get_parameter(this->MRS_PARAM_LIP_MAX).get_parameter_value().get<int>();

        this->_model.param.tiredness             = this->get_parameter(this->MRS_PARAM_TIREDNESS).get_parameter_value().get<double>();
        this->_model.param.priority_to_the_right = this->get_parameter(this->MRS_PARAM_PRIORITY_TO_THE_RIGHT).get_parameter_value().get<bool>();

        this->_model.param.timeout_s_received = this->get_parameter(this->MRS_PARAM_TIMEOUT_S_RECEIVED).get_parameter_value().get<double>();
        this->_model.param.timeout_s_chased   = this->get_parameter(this->MRS_PARAM_TIMEOUT_S_CHASED).get_parameter_value().get<double>();
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
                        // left
                        if (param.get_name() == this->MRS_PARAM_EYE_LEFT_OFFSET_X) {
                            this->_model.param.eye_left_offset_x = param.as_double();
                            results->successful                  = true;
                        } else if (param.get_name() == this->MRS_PARAM_EYE_LEFT_OFFSET_Y) {
                            this->_model.param.eye_left_offset_y = param.as_double();
                            results->successful                  = true;
                        } else if (param.get_name() == this->MRS_PARAM_EYE_LEFT_OFFSET_ANGLE) {
                            this->_model.param.eye_left_offset_angle = param.as_double();
                            results->successful                      = true;
                        }
                        // right
                        else if (param.get_name() == this->MRS_PARAM_EYE_RIGHT_OFFSET_X) {
                            this->_model.param.eye_right_offset_x = param.as_double();
                            results->successful                   = true;
                        } else if (param.get_name() == this->MRS_PARAM_EYE_RIGHT_OFFSET_Y) {
                            this->_model.param.eye_right_offset_y = param.as_double();
                            results->successful                   = true;
                        } else if (param.get_name() == this->MRS_PARAM_EYE_RIGHT_OFFSET_ANGLE) {
                            this->_model.param.eye_right_offset_angle = param.as_double();
                            results->successful                       = true;
                        }
                        // timeout
                        else if (param.get_name() == this->MRS_PARAM_TIMEOUT_S_RECEIVED) {
                            this->_model.param.timeout_s_received = param.as_double();
                            results->successful                   = true;
                        } else if (param.get_name() == this->MRS_PARAM_TIMEOUT_S_CHASED) {
                            this->_model.param.timeout_s_chased = param.as_double();
                            results->successful                 = true;
                        } else if (param.get_name() == this->MRS_PARAM_TIREDNESS) {
                            this->_model.param.tiredness = param.as_double();
                            results->successful          = true;
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
                    case rclcpp::PARAMETER_BOOL:
                        if (param.get_name() == this->MRS_PARAM_PRIORITY_TO_THE_RIGHT) {
                            this->_model.param.priority_to_the_right = param.as_bool();
                            results->successful                      = true;
                        }
                        break;
                    case rclcpp::PARAMETER_NOT_SET:
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

// =============================
// ROS : loop function
// =============================
void NodeImplement::_callback_timer()
{
    bool result = this->_model.calculate(this->get_clock()->now().seconds());
    // get message
    if (true == result) {
        this->_model.get_msg_eye(this->_msg_eye);
        this->_model.get_msg_neck(this->_msg_neck);
        this->_model.get_msg_lip(this->_msg_lip);
    }
    this->_model.get_msg_head_state(this->_msg_head_status);

    // publish
    this->_pub_eye->publish(this->_msg_eye);
    this->_pub_neck->publish(this->_msg_neck);
    this->_pub_lip->publish(this->_msg_lip);
    this->_pub_head_status->publish(this->_msg_head_status);
}

// =============================
// Constructor
// =============================
NodeImplement::NodeImplement(std::string node_name, int argc, char **argv) : Node(node_name)
{
#if LOGGER_ROS_INFO_CALL_FUNCTION
    RCLCPP_INFO_EXPRESSION(this->get_logger(), LOGGER_ROS_INFO_CALL_FUNCTION, "[%s] : %s", this->get_name(), "start.");
#endif

    // set parameter
    if (0 == this->_callback_param_init()) {
        // set publisher
        this->_pub_eye =                                                           //
                this->create_publisher<maid_robot_system_interfaces::msg::MrsEye>( //
                        this->MRS_TOPIC_OUT_EYE,                                   //
                        this->DEPTH_PUBLISHER                                      //
                );
        this->_pub_neck =                                                           //
                this->create_publisher<maid_robot_system_interfaces::msg::MrsNeck>( //
                        this->MRS_TOPIC_OUT_NECK,                                   //
                        this->DEPTH_PUBLISHER                                       //
                );
        this->_pub_lip =                                                           //
                this->create_publisher<maid_robot_system_interfaces::msg::MrsLip>( //
                        this->MRS_TOPIC_OUT_LIP,                                   //
                        this->DEPTH_PUBLISHER                                      //
                );
        this->_pub_head_status =                                                          //
                this->create_publisher<maid_robot_system_interfaces::msg::MrsHeadStatus>( //
                        this->MRS_TOPIC_OUT_STATUS,                                       //
                        this->DEPTH_PUBLISHER                                             //
                );

        // set subscription
        this->_sub_pose_left =                                                               //
                this->create_subscription<maid_robot_system_interfaces::msg::PoseDetection>( //
                        this->MRS_TOPIC_IN_POSTURE_LEFT,                                     //
                        this->DEPTH_SUBSCRIPTION,                                            //
                        std::bind(&NodeImplement::_callback_pose_left, this, _1));
        this->_sub_pose_right =                                                              //
                this->create_subscription<maid_robot_system_interfaces::msg::PoseDetection>( //
                        this->MRS_TOPIC_IN_POSTURE_RIGHT,                                    //
                        this->DEPTH_SUBSCRIPTION,                                            //
                        std::bind(&NodeImplement::_callback_pose_right, this, _1));
        this->_sub_ar_left =                                                             //
                this->create_subscription<maid_robot_system_interfaces::msg::ArMarkers>( //
                        this->MRS_TOPIC_IN_MARKS_LEFT,                                   //
                        this->DEPTH_SUBSCRIPTION,                                        //
                        std::bind(&NodeImplement::_callback_ar_left, this, _1));
        this->_sub_ar_right =                                                            //
                this->create_subscription<maid_robot_system_interfaces::msg::ArMarkers>( //
                        this->MRS_TOPIC_IN_MARKS_RIGHT,                                  //
                        this->DEPTH_SUBSCRIPTION,                                        //
                        std::bind(&NodeImplement::_callback_ar_right, this, _1));
        this->_sub_voice =                                                              //
                this->create_subscription<maid_robot_system_interfaces::msg::MrsVoice>( //
                        this->MRS_TOPIC_IN_VOICE,                                       //
                        this->DEPTH_SUBSCRIPTION,                                       //
                        std::bind(&NodeImplement::_callback_voice, this, _1));
        this->_sub_voltage =                                       //
                this->create_subscription<std_msgs::msg::Float64>( //
                        this->MRS_TOPIC_IN_VOLTAGE,                //
                        this->DEPTH_SUBSCRIPTION,                  //
                        std::bind(&NodeImplement::_callback_voltage, this, _1));

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
