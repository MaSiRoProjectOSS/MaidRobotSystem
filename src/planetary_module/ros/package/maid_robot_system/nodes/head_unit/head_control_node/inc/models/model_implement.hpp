/**
 * @file model_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_HEAD_CONTROL_NODE_MODEL_IMPLEMENT_HPP
#define MRS_HEAD_CONTROL_NODE_MODEL_IMPLEMENT_HPP

#include "maid_robot_system_interfaces/angle_calculus.hpp"
#include "models/model_structure.hpp"

#include <maid_robot_system_interfaces/msg/mrs_eye.hpp>
#include <maid_robot_system_interfaces/msg/mrs_lip.hpp>
#include <maid_robot_system_interfaces/msg/mrs_neck.hpp>
#include <maid_robot_system_interfaces/msg/pose_detection.hpp>
#include <maid_robot_system_interfaces/state/mrs_mode.hpp>
#include <stdio.h>
#include <string>

namespace maid_robot_system
{
#define DEBUG_MODEL_IMPLEMENT 1

class ModelImplement {
public:
    StParam param;

private:
    StTemporaryOverall _temp_overall;
    StTemporaryEye _temp_left;
    StTemporaryEye _temp_right;

    const float _MINIMUM_LIMIT_OF_FLOAT_VALUE = 1e-6f;

public:
    // =============================
    // PUBLIC : Function
    // =============================
    bool calculate(double seconds);
    bool set_value_voice(std::string text, int command, double seconds);
    bool set_value_ar(ModelStructure::INPUT_TYPE type, int id, double seconds);
    bool set_value_pose(ModelStructure::INPUT_TYPE type, const maid_robot_system_interfaces::msg::PoseDetection msg, double seconds);
    bool set_value_tiredness(float data, double seconds);

    void get_msg_eye(maid_robot_system_interfaces::msg::MrsEye &msg);
    void get_msg_neck(maid_robot_system_interfaces::msg::MrsNeck &msg);
    void get_msg_lip(maid_robot_system_interfaces::msg::MrsLip &msg);

private:
    // =============================
    // PRIVATE : Function
    // =============================
    bool _set_value_pose(const maid_robot_system_interfaces::msg::PoseDetection msg,
                         StTemporaryEye &temp,
                         double offset_x,
                         double offset_y,
                         double offset_z,
                         double offset_angle,
                         double seconds);
    bool _msg_clear();
    bool _tmp_clear();
    void _output_param();

    void _calculate_eye(double x, double y, double dimensions, double distance);
    void _calculate_neck(double x, double y, double roll);

    bool _is_zero(float value, float min_limit);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    MRS_MODE _mode = MRS_MODE::MRS_MODE_NONE;
    maid_robot_system_interfaces::msg::MrsEye _msg_eye;
    maid_robot_system_interfaces::msg::MrsNeck _msg_neck;
    maid_robot_system_interfaces::msg::MrsLip _msg_lip;

public:
    // =============================
    // Constructor
    // =============================
    ModelImplement();
    ~ModelImplement();
};

} // namespace maid_robot_system

#endif
