/**
 * @file model_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef SAMPLE_NODE_MODEL_IMPLEMENT_HPP
#define SAMPLE_NODE_MODEL_IMPLEMENT_HPP

#include "../../../../../maid_robot_system_interfaces/include/maid_robot_system_interfaces/angle_calculus.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace maid_robot_system
{
class ModelImplement {
public:
    // =============================
    // PUBLIC : Function
    // =============================
    void set_times(double value);
    void set_offset(double value);
    void set_position_rotation(const geometry_msgs::msg::PoseStamped &msg);
    bool calculate();

    double get_times();
    double get_offset();
    double get_value();

private:
    // =============================
    // PRIVATE : Function
    // =============================
    void _handshake_follow(float r, float sita, float z);

    float _constrain(float value, float min, float max);
    float _map(float value, float from_low, float from_high, float to_low, float to_high);

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    typedef enum DEF_WAIST_DOWN_UNIT_MODE
    {
        WAIST_DOWN_UNIT_MODE_STOP = 0,
        WAIST_DOWN_UNIT_MODE_CRUISE,
        WAIST_DOWN_UNIT_MODE_WAIT,
        WAIST_DOWN_UNIT_MODE_PC_CONTROL,
        WAIST_DOWN_UNIT_MODE_FOLLOW
    } WAIST_DOWN_UNIT_MODE;

    WAIST_DOWN_UNIT_MODE _mode = WAIST_DOWN_UNIT_MODE_STOP;

    double _offset = 200;
    double _times  = 300;

    int _max_speed        = 20000; /*!< max speed */
    float _max_turn_speed = 1.2;   /*!< max turn speed */

    Vector3 *_position    = new Vector3(0.0f, 0.0f, 0.0f);
    Quaternion *_rotation = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);

    float _wheel_target_v = 0.0; /*!< wheel target v */
    float _wheel_target_w = 0.0; /*!< wheel target w */

    int _target_r = 150;  /*!< target r */
    int _target_s = 15;   /*!< target s */
    float _K_v    = -1.8; /*!< gain of v */
    float _K_w    = 0.03; /*!< gain of w */

    int _z_count = 0; /*!<  */

    float _speed_inertia = 0.1; /*!< speed inertia  */

private:
    // =============================
    // PRIVATE : Constant
    // =============================
    const int _MAX_S_ERROR_TO_V = 90; /*!<  */

    const int _R_V_FACTOR = 10;    /*!<  */
    const int _S_W_FACTOR = 10000; /*!<  */

    const int _V_MIN = 0;   /*!<  */
    const int _V_MAX = 100; /*!<  */

    const int START_Z_UPPER = -200;

    const int _Z_COUNT_DECREASE_RATE = 3;   /*!<  */
    const int _Z_COUNT_MIN           = 0;   /*!<  */
    const int _Z_COUNT_MAX           = 100; /*!<  */

    const float _R_MOVING_LIMIT = 100.0; /*!<  */
    const float _S_MOVING_LIMIT = 0.1;   /*!<  */

    const float _SPEED_INERTIA_INCREASE_RATE = 50.0 / 1500.0; /*!<  */
    const float _SPEED_INERTIA_DECREASE_RATE = 50.0 / 2500.0; /*!<  */

    const float _SPEED_INERTIA_CONSTRAIN_MIN = 10.0;  /*!<  */
    const float _SPEED_INERTIA_CONSTRAIN_MAX = 100.0; /*!<  */

    const float _V_DECREASE_FACTOR = 1.5;

public:
    // =============================
    // Constructor
    // =============================
    ModelImplement();
    ~ModelImplement();
};

} // namespace maid_robot_system

#endif
