/**
 * @file node_settings.hpp
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
#define DRAWING_MAX_FPS     (40)
#define DEBUG_OUTPUT_REPORT (1000 * 10)

#define DEBUG_OUTPUT_OPEN_GL     1
#define DEBUG_OUTPUT_WIDGET      1
#define DEBUG_OUTPUT_FPS_VERBOSE 0
#define DEBUG_OUTPUT_LOAD_IMAGE  1
#define DEBUG_OUTPUT_PARAM_LV    1

#define DEBUG_OUTPUT_BEHAVIOR 1

#define LOGGER_INFO_DETAIL               0
#define LOGGER_INFO_PARAMETER            0
#define LOGGER_INFO_SUBSCRIPTION_MRS_EYE 0
#define LOGGER_INFO_CALL_FUNCTION        0

} // namespace maid_robot_system

#endif
