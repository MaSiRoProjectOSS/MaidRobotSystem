/**
 * @file eye_node_settings.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-10-07
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_SETTINGS_HPP
#define MRS_EYE_NODE_SETTINGS_HPP

namespace maid_robot_system
{
#define DRAWING_MAX_FPS (60)

#define DEBUG_OUTPUT_OPEN_GL    1
#define DEBUG_OUTPUT_WIDGET     0
#define DEBUG_OUTPUT_LOAD_IMAGE 0
#define DEBUG_OUTPUT_PARAM_LV   0

#define DEBUG_OUTPUT_BEHAVIOR 1

#define LOGGER_ROS_INFO_OUTPUT_REPORT_TIME   (1000 * 10)
#define LOGGER_ROS_INFO_DETAIL               0
#define LOGGER_ROS_INFO_PARAMETER            0
#define LOGGER_ROS_INFO_SUBSCRIPTION_MRS_EYE 0
#define LOGGER_ROS_INFO_CALL_FUNCTION        0

} // namespace maid_robot_system

#endif
