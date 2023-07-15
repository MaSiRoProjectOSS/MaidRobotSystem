/**
 * @file parameter_name.hpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_ROS_PARAMETER_NAME_HPP
#define MAID_ROBOT_SYSTEM_ROS_PARAMETER_NAME_HPP

#include "../config/project_config.hpp"

/* ------------------------------------------------------------------------ */
// node : hitomi_node
/* ------------------------------------------------------------------------ */
#define MRS_PARAMETER_ARRANGEMENT CONFIG_SKIN_NAME "/arrangement/"
/* ======================================================================== */
#define MRS_PARAMETER_HITOMI_EYELID_LEFT_X  MRS_PARAMETER_ARRANGEMENT "eyelid/left/x"
#define MRS_PARAMETER_HITOMI_EYELID_LEFT_Y  MRS_PARAMETER_ARRANGEMENT "eyelid/left/y"
#define MRS_PARAMETER_HITOMI_EYELID_RIGHT_X MRS_PARAMETER_ARRANGEMENT "eyelid/right/x"
#define MRS_PARAMETER_HITOMI_EYELID_RIGHT_Y MRS_PARAMETER_ARRANGEMENT "eyelid/right/y"
#define MRS_PARAMETER_HITOMI_EYEBALL_X      MRS_PARAMETER_ARRANGEMENT "eyeball/x"
#define MRS_PARAMETER_HITOMI_EYEBALL_Y      MRS_PARAMETER_ARRANGEMENT "eyeball/y"
/* ======================================================================== */

/* ------------------------------------------------------------------------ */
// node : fms
/* ------------------------------------------------------------------------ */
/* ======================================================================== */
#define MRS_PARAMETER_SERVER_ADDRESS "/server_address"
#define MRS_PARAMETER_QR_DEVICE_NAME "device"
/* ======================================================================== */

#endif // MAID_ROBOT_SYSTEM_ROS_PARAMETER_NAME_HPP
