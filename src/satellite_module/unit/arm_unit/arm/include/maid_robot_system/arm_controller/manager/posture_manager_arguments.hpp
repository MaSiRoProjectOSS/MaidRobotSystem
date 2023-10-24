/**
 * @file posture_manager_arguments.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Define posture_manager arguments struct.
 * @version 0.23.2
 * @date 2023-04-24
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_POSTURE_MANAGER_ARGUMENTS_HPP
#define ARM_CONTROLLER_POSTURE_MANAGER_ARGUMENTS_HPP

#include "maid_robot_system/arm_controller/config.hpp"
#include "maid_robot_system/common/time_check.hpp"
#include "maid_robot_system/common/types/coordinate_euler.hpp"

#include <Arduino.h>

struct PostureManagerArguments {
public:
    typedef enum
    {
        MODE_NECK_UNKNOWN,
        MODE_NECK_LOOK_FORWARD,
        MODE_NECK_I_AGREE,
        MODE_NECK_QUESTION,
        MODE_NECK_STARE_AT_THE_OTHER_PERSON,
        MODE_NECK_FOLLOW_YOUR_HAND,
        MODE_NECK_FACE_TRACKING = 111,
    } neck_mode_t;

public:
    struct InputArguments {
    public:
        /****************************************
         * SHOULDER
         ****************************************/
        float shoulder_left_push  = 0;
        float shoulder_right_push = 0;
        /****************************************
         * NECK
         ****************************************/
        neck_mode_t neck_mode = MODE_NECK_LOOK_FORWARD;
        CoordinateEuler neck_angle;
        CoordinateEuler neck_gyro;
        /****************************************
         * VOICE command
         ****************************************/
        long voice_command = 0;
        /****************************************
         * WHEEL
         ****************************************/
        CoordinateEuler wheel_gy;
        float wheel_theta    = 0;
        float wheel_target_v = 0;
        float wheel_target_w = 0;
        /****************************************
         * LEG
         ****************************************/
        int leg_pos = -888;
        int leg_step_percentage;
        bool leg_communication_ok = false;
    };
    struct States {
    public:
        /****************************************
         * VITAL
         ****************************************/
        int breath_speed = 4500;
        TimeCheck vital_timer;
        float vital;
        /****************************************
         * NECK
         ****************************************/
        bool flag_face_track_enable = 1; //flag for activating face_track
        int delta_pose;
        String pose_command       = "stay";
        bool mode_flag_face_track = 0;

        /****************************************
         * SENSOR_ARM
         ****************************************/
        float sensor_arm_hand_z = 0;
        /****************************************
         * WAIST
         ****************************************/
        CoordinateEuler waist_target;
        float waist_speed_gain = 1.0;
        /****************************************
         * LEG
         ****************************************/
        int leg_mode = STAND;
    };
    struct OutputArguments {
    public:
        /****************************************
         * NECK
         ****************************************/
        CoordinateEuler neck_angle;
        CoordinateEuler neck_face_track;
        CoordinateEuler neck_current;

        /****************************************
         * WAIST
         ****************************************/
        CoordinateEuler waist_angle;
        /****************************************
         * LEG
         ****************************************/
        int leg_send_speed = 0;
        int leg_send_pos   = 950;
        bool leg_send_flag = false;
    };

public:
    InputArguments input;
    States states;
    OutputArguments output;
};

#endif
