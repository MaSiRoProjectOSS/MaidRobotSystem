/**
 * @file communication_ros.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Communicate with micro ROS.
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/communication_ros.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

/*********************************************************
 * Static member variables
 *********************************************************/
PostureManagerArguments::neck_mode_t CommunicationROS::_received_neck_mode = PostureManagerArguments::MODE_NECK_UNKNOWN;
CoordinateEuler CommunicationROS::_received_neck_angle;
long CommunicationROS::_received_voice_command = 0;
volatile bool CommunicationROS::_flag_received = false;
CoordinateEuler CommunicationROS::_received_wheel;
CoordinateEuler CommunicationROS::_received_leg;
volatile bool CommunicationROS::_flag_received_leg = false;

/*********************************************************
 * Class definition
 *********************************************************/
CommunicationROS::CommunicationROS()
{
}

CommunicationROS::~CommunicationROS()
{
}

void CommunicationROS::_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
#if DEBUG_BOARD_LED
        static bool flag_led = false;
        if (true == flag_led) {
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
        }
        flag_led = !flag_led;
#endif
    }
}

void CommunicationROS::_subscription_neck_control(const void *msg_in)
{
    const geometry_msgs__msg__Twist *msg         = (const geometry_msgs__msg__Twist *)msg_in;
    CommunicationROS::_received_neck_mode        = (PostureManagerArguments::neck_mode_t)msg->linear.x;
    CommunicationROS::_received_neck_angle.pitch = msg->angular.y;
    CommunicationROS::_received_neck_angle.yaw   = msg->angular.z;
    CommunicationROS::_received_neck_angle.roll  = msg->angular.x;
}

void CommunicationROS::_subscription_get_voice_id(const void *msg_in)
{
    const std_msgs__msg__Int64 *msg           = (const std_msgs__msg__Int64 *)msg_in;
    CommunicationROS::_received_voice_command = (long)msg->data;
    CommunicationROS::_flag_received          = true;
}

void CommunicationROS::_subscription_wheel(const void *msg_in)
{
    const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msg_in;
    CommunicationROS::_received_wheel.set(msg->y, msg->x, msg->z);
}

void CommunicationROS::_subscription_leg(const void *msg_in)
{
    const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msg_in;
    CommunicationROS::_received_leg.set(msg->y, msg->x, msg->z);
    CommunicationROS::_flag_received_leg = true;
}

void CommunicationROS::ROS_setup()
{
#if DEBUG_BOARD_LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
#endif
    set_microros_transports();
    delay(2000);

    this->_allocator = rcl_get_default_allocator();

    /* create init_options */
    rclc_support_init(&this->_support, 0, NULL, &this->_allocator);

    /* create node */
    rclc_node_init_default(&this->_node, COMMUNICATION_ROS_NODE_NAME, COMMUNICATION_ROS_NAMESPACE_NAME, &this->_support);

    /* create publisher */
    rclc_publisher_init_default(&this->_pub_ROS_masiro_vital,
                                &this->_node, //
                                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
                                COMMUNICATION_ROS_TOPIC_VITAL);
    rclc_publisher_init_default(&this->_pub_neck_now_pos,
                                &this->_node, //
                                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                COMMUNICATION_ROS_TOPIC_NECK_POSE);
    rclc_publisher_init_default(&this->_pub_R_arm_joint_now_pos,
                                &this->_node, //
                                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                COMMUNICATION_ROS_TOPIC_ARM_RIGHT_ANGLE);
    rclc_publisher_init_default(&this->_pub_R_arm_hand_pose,
                                &this->_node, //
                                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                COMMUNICATION_ROS_TOPIC_ARM_RIGHT_HAND);
    rclc_publisher_init_default(&this->_pub_wheel_pose2d,
                                &this->_node, //
                                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
                                COMMUNICATION_ROS_TOPIC_WHEEL);
    rclc_publisher_init_default(&this->_pub_ROS_debug_msg,
                                &this->_node, //
                                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                COMMUNICATION_ROS_TOPIC_DEBUG_MSG);
    rclc_publisher_init_default(&this->_pub_ROS_debug_data,
                                &this->_node, //
                                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
                                COMMUNICATION_ROS_TOPIC_DEBUG_DATA);

    /* create subscriber */
    rclc_subscription_init_default(&this->_subscriber_neck,
                                   &this->_node, //
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                   COMMUNICATION_ROS_TOPIC_NECK_COMMAND);
    rclc_subscription_init_default(&this->_subscriber_voice,
                                   &this->_node, //
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                   COMMUNICATION_ROS_TOPIC_VOICE_ID);
    rclc_subscription_init_default(&this->_subscriber_wheel,
                                   &this->_node, //
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                   COMMUNICATION_ROS_TOPIC_WHEEL_GYRO);
    rclc_subscription_init_default(&this->_subscriber_leg,
                                   &this->_node, //
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                   COMMUNICATION_ROS_TOPIC_LEG);

    /* create timer */
    rclc_timer_init_default(&this->_timer, &this->_support, RCL_MS_TO_NS(this->_TIMER_TIMEOUT), this->_timer_callback);

    /* create executor */
    rclc_executor_init(&this->_executor, &this->_support.context, 1, &this->_allocator);
    rclc_executor_add_timer(&this->_executor, &this->_timer);

    rclc_executor_add_subscription(&this->_executor, &this->_subscriber_neck, &this->_ROS_neck_control, this->_subscription_neck_control, ON_NEW_DATA);
    rclc_executor_add_subscription(&this->_executor, &this->_subscriber_voice, &this->_ROS_get_voiceID, this->_subscription_get_voice_id, ON_NEW_DATA);
    rclc_executor_add_subscription(&this->_executor, &this->_subscriber_wheel, &this->_ROS_wheel, this->_subscription_wheel, ON_NEW_DATA);
    rclc_executor_add_subscription(&this->_executor, &this->_subscriber_leg, &this->_ROS_leg, this->_subscription_leg, ON_NEW_DATA);
}

bool CommunicationROS::ROS_send(PostureManagerArguments *args)
{
    bool result = false;
    static TimeCheck timer_ros_send;
    static int cnt = 0;
    if (true == timer_ros_send.check_passing(2)) {
        cnt++;
        switch (cnt) {
            case 1:
                this->_ROS_ciro_vital.data = this->_VITAL_FLOAT_TO_INT_FACTOR * args->states.vital;
                if (RCL_RET_OK == rcl_publish(&this->_pub_ROS_masiro_vital, &this->_ROS_ciro_vital, NULL)) {
                    result = true;
                }
                break;
            case 2:
                if (RCL_RET_OK == rcl_publish(&this->_pub_neck_now_pos, &this->_ROS_neck_now_pose, NULL)) {
                    result = true;
                }
                break;
            case 3:
                if (RCL_RET_OK == rcl_publish(&this->_pub_R_arm_joint_now_pos, &this->_ROS_R_arm_joint_now_angle, NULL)) {
                    result = true;
                }
                break;
            case 4:
                if (RCL_RET_OK == rcl_publish(&this->_pub_R_arm_hand_pose, &this->_ROS_R_arm_hand_pose, NULL)) {
                    result = true;
                }
                break;
            case 5:
                this->_ROS_wheel_pose2d.theta = args->input.wheel_theta;
                if (RCL_RET_OK == rcl_publish(&this->_pub_wheel_pose2d, &this->_ROS_wheel_pose2d, NULL)) {
                    result = true;
                }
                break;

            default:
                //ROS_debug_msg.data.data  = (1 == args->states.flag_face_track_enable) ? "enable" : "disable";
                this->_ROS_debug_data.data = args->states.delta_pose;
                if (RCL_RET_OK == rcl_publish(&this->_pub_ROS_debug_msg, &this->_ROS_debug_msg, NULL)) {
                    if (RCL_RET_OK == rcl_publish(&this->_pub_ROS_debug_data, &this->_ROS_debug_data, NULL)) {
                        result = true;
                    }
                }
                cnt = 0;
                break;
        }
    }
    return result;
}

void CommunicationROS::ROS_check(PostureManagerArguments *args)
{
    rclc_executor_spin_some(&this->_executor, RCL_MS_TO_NS(10));

    if (true == this->_flag_received) {
        args->input.voice_command = this->_received_voice_command;
        this->_flag_received      = false;
    }
    args->input.neck_angle.set(_received_neck_angle.pitch, _received_neck_angle.roll, _received_neck_angle.yaw);
    args->input.neck_mode = this->_received_neck_mode;
    args->input.wheel_gy.set(this->_received_wheel.pitch, this->_received_wheel.roll, this->_received_wheel.yaw);

    if (true == this->_flag_received_leg) {
        args->input.leg_pos              = this->_received_leg.pitch;
        args->input.leg_step_percentage  = this->_received_leg.roll;
        args->input.leg_communication_ok = true;
        this->_flag_received_leg         = false;
    }
}

} // namespace arm_unit
} // namespace maid_robot_system
