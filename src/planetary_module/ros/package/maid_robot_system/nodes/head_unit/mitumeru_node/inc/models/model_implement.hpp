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
private:
    class st_eye {
    public:
        bool detected;
        int emotion;
        int size;
        int x;
        int y;
        int z;
        bool flag_reversal;
        /////
        int distance;
        double angle_x;
        double angle_y;
        double angle_z;
        double CAMERA_ANGLE_X = 140.0;
        double CAMERA_ANGLE_Y = 140.0;
        double offset_x_angle;
        double offset_y_angle;
        double target_roll;
    };
    st_eye _eye_left;
    st_eye _eye_right;

public:
    // =============================
    // PUBLIC : Function
    // =============================
    bool calculate(maid_robot_system_interfaces::msg::MrsHitomi &msg_hitomi,
                   maid_robot_system_interfaces::msg::MrsKubi &msg_kubi,
                   maid_robot_system_interfaces::msg::MrsKuchibiru &msg_kuchibiru);
    bool set_value_voice(std::string text, int command);
    bool set_value_ar(ModelStructure::INPUT_TYPE type, int id);
    bool set_value_pose(ModelStructure::INPUT_TYPE type, const maid_robot_system_interfaces::msg::PoseDetection msg);

private:
    // =============================
    // PRIVATE : Function
    // =============================
    void _get_value_hitomi(maid_robot_system_interfaces::msg::MrsHitomi &msg);
    void _get_value_kubi(maid_robot_system_interfaces::msg::MrsKubi &msg);
    void _get_value_kuchibiru(maid_robot_system_interfaces::msg::MrsKuchibiru &msg);
    bool _calculate_pose(const maid_robot_system_interfaces::msg::PoseDetection msg, st_eye &data);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    MRS_MODE _mode = MRS_MODE::MRS_MODE_NONE;
    maid_robot_system_interfaces::msg::MrsHitomi _msg_hitomi;
    maid_robot_system_interfaces::msg::MrsKubi _msg_kubi;
    maid_robot_system_interfaces::msg::MrsKuchibiru _msg_kuchibiru;

public:
    // =============================
    // Constructor
    // =============================
    ModelImplement();
    ~ModelImplement();
};

} // namespace maid_robot_system

#endif
