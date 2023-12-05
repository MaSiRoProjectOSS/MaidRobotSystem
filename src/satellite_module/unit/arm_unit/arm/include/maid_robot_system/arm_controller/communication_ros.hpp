/**
 * @file communication_ros.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Communicate with micro ROS.
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_COMMUNICATION_ROS_HPP
#define ARM_CONTROLLER_COMMUNICATION_ROS_HPP

#include "maid_robot_system/arm_controller/config.hpp"
#include "maid_robot_system/arm_controller/manager/posture_manager_arguments.hpp"
#include "maid_robot_system/common/time_check.hpp"
#include "maid_robot_system/common/types/coordinate_euler.hpp"

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

/***/
#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/string.h>

/***/
#define DEBUG_BOARD_LED 1
/***/
#ifndef COMMUNICATION_ROS_NODE_NAME
#define COMMUNICATION_ROS_NODE_NAME "arm_controller"
#endif
#ifndef COMMUNICATION_ROS_NAMESPACE_NAME
#define COMMUNICATION_ROS_NAMESPACE_NAME "arm_unit"
#endif

#define COMMUNICATION_ROS_TOPIC_VITAL           "vital"
#define COMMUNICATION_ROS_TOPIC_NECK_POSE       "neck/now_pose"
#define COMMUNICATION_ROS_TOPIC_ARM_RIGHT_ANGLE "arm/Right/now_angle"
#define COMMUNICATION_ROS_TOPIC_ARM_RIGHT_HAND  "arm/Right/hand"
#define COMMUNICATION_ROS_TOPIC_WHEEL           "wheel/pose"

#define COMMUNICATION_ROS_TOPIC_DEBUG_MSG  "debug_msg"
#define COMMUNICATION_ROS_TOPIC_DEBUG_DATA "debug_data"

#define COMMUNICATION_ROS_TOPIC_NECK_COMMAND "neck/cmd_command"
#define COMMUNICATION_ROS_TOPIC_VOICE_ID     "voiceID"
#define COMMUNICATION_ROS_TOPIC_WHEEL_GYRO   "wheel/gyro"
#define COMMUNICATION_ROS_TOPIC_LEG          "leg/info"

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Communicate with micro ROS.
 *
 */
class CommunicationROS {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    CommunicationROS();

    /*********************************************************
     * Destructor
     *********************************************************/
    ~CommunicationROS();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    void ROS_setup();
    bool ROS_send(PostureManagerArguments *args);
    void ROS_check(PostureManagerArguments *args);

private:
    /*********************************************************
     * Private function
     *********************************************************/
    static void _timer_callback(rcl_timer_t *timer, int64_t last_call_time);

    static void _subscription_neck_control(const void *msg_in);

    static void _subscription_get_voice_id(const void *msg_in);

    static void _subscription_wheel(const void *msg_in);

    static void _subscription_leg(const void *msg_in);

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    static PostureManagerArguments::neck_mode_t _received_neck_mode; /*!<  */
    static CoordinateEuler _received_neck_angle;                     /*!<  */
    static long _received_voice_command;                             /*!<  */
    static volatile bool _flag_received;                             /*!<  */
    static CoordinateEuler _received_wheel;                          /*!<  */
    static CoordinateEuler _received_leg;                            /*!<  */
    static volatile bool _flag_received_leg;                         /*!<  */

    rclc_executor_t _executor;  /*!<  */
    rclc_support_t _support;    /*!<  */
    rcl_allocator_t _allocator; /*!<  */
    rcl_node_t _node;           /*!<  */
    rcl_timer_t _timer;         /*!<  */

    rcl_publisher_t _pub_ROS_masiro_vital;    /*!<  */
    rcl_publisher_t _pub_neck_now_pos;        /*!<  */
    rcl_publisher_t _pub_R_arm_joint_now_pos; /*!<  */
    rcl_publisher_t _pub_R_arm_hand_pose;     /*!<  */
    rcl_publisher_t _pub_wheel_pose2d;        /*!<  */
    rcl_publisher_t _pub_ROS_debug_msg;       /*!<  */
    rcl_publisher_t _pub_ROS_debug_data;      /*!<  */

    rcl_subscription_t _subscriber_neck;  /*!<  */
    rcl_subscription_t _subscriber_voice; /*!<  */
    rcl_subscription_t _subscriber_wheel; /*!<  */
    rcl_subscription_t _subscriber_leg;   /*!<  */

    std_msgs__msg__Int16 _ROS_ciro_vital;                        /*!<  */
    geometry_msgs__msg__Twist _ROS_neck_now_pose;                /*!<  */
    std_msgs__msg__Float32MultiArray _ROS_R_arm_joint_now_angle; /*!<  */
    geometry_msgs__msg__Twist _ROS_R_arm_hand_pose;              /*!<  */
    geometry_msgs__msg__Pose2D _ROS_wheel_pose2d;                /*!<  */
    std_msgs__msg__String _ROS_debug_msg;                        /*!<  */
    std_msgs__msg__Int16 _ROS_debug_data;                        /*!<  */

    geometry_msgs__msg__Twist _ROS_neck_control; /*!<  */
    std_msgs__msg__Int64 _ROS_get_voiceID;       /*!<  */

    geometry_msgs__msg__Vector3 _ROS_wheel; /*!<  */
    geometry_msgs__msg__Vector3 _ROS_leg;   /*!<  */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const unsigned int _TIMER_TIMEOUT = 1000; /*!<  */

    const int _VITAL_FLOAT_TO_INT_FACTOR = 1000; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
