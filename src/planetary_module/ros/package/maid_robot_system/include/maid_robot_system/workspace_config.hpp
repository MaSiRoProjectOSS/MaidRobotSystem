/**
 * @file workspace_config.hpp
 * @brief
 * @date 2020-02-23
 *
 * @copyright Copyright (c) MaSiRo Project. 2021-.
 *
 */

#ifndef MAID_ROBOT_SYSTEM_WORKSPACE_CONFIG_HPP
#define MAID_ROBOT_SYSTEM_WORKSPACE_CONFIG_HPP

#include "config/project_config.hpp"
#include "ros/node_name.hpp"
#include "ros/packages_name.hpp"
#include "ros/parameter_name.hpp"
#include "ros/topic_name.hpp"

/* ======================================================================== */
/**
* @brief PATH for catkin workspace
*
*/
#ifndef PROJECT_DIR_PATH
#define PROJECT_DIR_PATH "/opt/MaidRobotSystem/data"
#endif
#define MRS_WORKSPACE_CONFIG_PATH_HOME PROJECT_DIR_PATH

/* ======================================================================== */

#endif // MAID_ROBOT_SYSTEM_WORKSPACE_CONFIG_HPP
