/**
 * @file model_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_NODE_MITUMERU_MODEL_IMPLEMENT_HPP
#define MRS_NODE_MITUMERU_MODEL_IMPLEMENT_HPP

#include "maid_robot_system_interfaces/angle_calculus.hpp"
#include "models/model_structure.hpp"

#include <maid_robot_system_interfaces/msg/mrs_hitomi.hpp>
#include <maid_robot_system_interfaces/msg/mrs_kubi.hpp>
#include <maid_robot_system_interfaces/msg/mrs_kuchibiru.hpp>
#include <maid_robot_system_interfaces/msg/pose_detection.hpp>
#include <maid_robot_system_interfaces/state/mrs_mode.hpp>
#include <stdio.h>
#include <string>

namespace maid_robot_system
{
class ModelImplement {
public:
    StParam param;

private:
    StTemporary _temp_left;
    StTemporary _temp_right;

public:
    // =============================
    // PUBLIC : Function
    // =============================
    bool calculate();
    bool set_value_voice(std::string text, int command);
    bool set_value_ar(ModelStructure::INPUT_TYPE type, int id);
    bool set_value_pose(ModelStructure::INPUT_TYPE type, const maid_robot_system_interfaces::msg::PoseDetection msg);
    void get_msg_oculus(maid_robot_system_interfaces::msg::MrsHitomi &msg);
    void get_msg_neck(maid_robot_system_interfaces::msg::MrsKubi &msg);
    void get_msg_lip(maid_robot_system_interfaces::msg::MrsKuchibiru &msg);

private:
    // =============================
    // PRIVATE : Function
    // =============================
    bool _calculate_pose(const maid_robot_system_interfaces::msg::PoseDetection msg, StTemporary &temp);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    MRS_MODE _mode = MRS_MODE::MRS_MODE_NONE;
    maid_robot_system_interfaces::msg::MrsHitomi _msg_oculus;
    maid_robot_system_interfaces::msg::MrsKubi _msg_neck;
    maid_robot_system_interfaces::msg::MrsKuchibiru _msg_lip;

public:
    // =============================
    // Constructor
    // =============================
    ModelImplement();
    ~ModelImplement();
};

} // namespace maid_robot_system

#endif
