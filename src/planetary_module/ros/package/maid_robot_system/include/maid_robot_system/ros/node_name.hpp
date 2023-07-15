/**
 * @file node_name.hpp
 * @brief
 * @date 2020-02-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
/* ========================================================================
  ROS Node name:
    ros2 run masiro_packages shell_node
                                ↑--------- this
 ======================================================================== */
#ifndef MAID_ROBOT_SYSTEM_ROS_NODE_HPP
#define MAID_ROBOT_SYSTEM_ROS_NODE_HPP

#define MRS_NODE_POWER_MANAGER "power_manager_node"

/* ------------------------------------------------------------------------ */
// unit : fms
/* ------------------------------------------------------------------------ */
#define MRS_NODE_FMS_REPORTER                 "reporter_node"
#define MRS_NODE_FMS_DISPATCHER               "dispatcher_node"
#define MRS_NODE_FMS_RECEPTIONIST             "receptionist_node"
#define MRS_NODE_FMS_RECEPTIONIST_QR_SCANNER  "qr_code_scanner_node"
#define MRS_NODE_FMS_RECEPTIONIST_GATE_KEEPER "gate_keeper_node"
#define MRS_NODE_FMS_RECEPTIONIST_ORDERLY     "orderly_node"

/* ------------------------------------------------------------------------ */
// unit : fms_edge
/* ------------------------------------------------------------------------ */
#define MRS_NODE_FMS_ACTION_MANAGER               "action_manager_node"
#define MRS_NODE_FMS_PREDICTIVE_FAILURE_DETECTION "predictive_failure_detection_node"

/* ------------------------------------------------------------------------ */
// unit : cast
/* ------------------------------------------------------------------------ */
#define MRS_NODE_EYES             "eyes_node"
#define MRS_NODE_CONVERT          "convert_node"
#define MRS_NODE_MENTAL_TODO_LIST "mental_todo_list_node"

/* ------------------------------------------------------------------------ */
// unit : NUCLEO
/* ------------------------------------------------------------------------ */
#define MRS_NODE_NUCLEO     "nucleo_node"
#define MRS_NODE_NAVIGATION "navigation_node"

/* ======================================================================== */

#endif // MAID_ROBOT_SYSTEM_ROS_NODE_HPP
