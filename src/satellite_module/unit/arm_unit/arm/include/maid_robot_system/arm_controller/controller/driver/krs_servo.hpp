/**
 * @file krs_servo.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Handle KRS servo
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_KRS_SERVO_HPP
#define ARM_CONTROLLER_KRS_SERVO_HPP

#include "maid_robot_system/arm_controller/config.hpp"
#include "maid_robot_system/common/move_average.hpp"
#include "maid_robot_system/common/time_check.hpp"

#include <Arduino.h>
#include <IcsHardSerialClass.h>

namespace maid_robot_system
{
namespace arm_unit
{

/**
 * @brief Handle KRS servo
 *
 */
class KRSServo {
public:
    /*********************************************************
     * Constructor
     *********************************************************/
    /**
     * @brief Construct a new KRSServo object
     *
     */
    KRSServo();

public:
    /*********************************************************
     * Public function
     *********************************************************/
    /**
     * @brief set up KRSServo
     *
     * @param _id : servo ID
     * @param _init_pos : initial position
     * @param _scale : scale
     * @param krs : KRS object
     */
    void setup(int _id, int _init_pos, float _scale, IcsHardSerialClass *krs);

    /**
     * @brief check unlock error
     *
     * @return error status
     */
    int check_unlock_error();

    /**
     * @brief calculate and send servo position as moving S curve.
     *
     * @return target position
     */
    float servo_move_time();

    /*********************************************************
     * Setter / Getter
     *********************************************************/
    /**
     * @brief Get current angle [deg]
     *
     * @return current angle
     */
    float get_deg();

    /**
     * @brief Set servo free
     *
     * @return raw position
     */
    int set_free();

    /**
     * @brief Set the pos object
     *
     * @param _tar_pos : target position
     * @return raw position
     */
    float set_pos(int _tar_pos);

    /**
     * @brief Set angle
     *
     * @param _tar_deg : target angle
     * @return current angle [deg]
     */
    float set_deg(float _tar_deg);

    /**
     * @brief Set target position
     *
     * @param target_pos: target position
     */
    void set_target(float target_pos);

    /**
     * @brief Set the strc
     *
     * @param _strc : strc
     */
    void set_strc(int _strc);

    /**
     * @brief Get current temperature.
     *
     */
    void get_temp();

    /**
     * @brief Get the current electrical current.
     *
     */
    void get_current();

    /**
     * @brief Set the move time object
     *
     * @param set_time : target time
     * @param tar_pos : target position
     */
    void set_move_time(int set_time, float tar_pos);

    /**
     * @brief Get current angle
     *
     * @return current angle
     */
    float get_now_deg();

    /**
     * @brief Set current angle
     *
     * @param current angle
     */
    void set_now_deg(float angle);

    /**
     * @brief Set target strc
     *
     * @param str : target strc
     */
    void set_tar_strc(int str);

    /**
     * @brief Get  error status
     *
     * @return error status
     */
    int get_error_status();

    /**
     * @brief Get mode
     *
     * @return mode
     */
    int get_mode();

    /**
     * @brief Get posing finish flag
     *
     * @return posing finish flag
     */
    int get_posing_finish_flag();

private:
    /*********************************************************
     * Private variables
     *********************************************************/
    typedef enum _ENUM_ERROR_STATUS
    {
        _ERROR_STATUS_NORMAL = 0,
        _ERROR_STATUS_ERROR
    } _ERROR_STATUS;
    _ERROR_STATUS _error_status; /*!< error status */

    typedef enum _ENUM_POSING_FINISH_STATUS
    {
        _POSING_NOT_FINISHED = 0,
        _POSING_FINISHED
    } _POSING_FINISH_STATUS;
    _POSING_FINISH_STATUS _posing_finish_flag; /*!< posing finished flag */

    int _ID   = 0;    /*!< servo ID */
    int _mode = FREE; /*!< KRS Servo mode */

    IcsHardSerialClass *_servo;   /*!< servo object */
    MoveAverage<10> _ave_tar_pos; /*!< moving average */

    int _now_strc       = -1;      /*!<  */
    int _tar_strc       = -1;      /*!<  */
    int _now_temp       = 0;       /*!< current servo temperature */
    int _now_current    = 0;       /*!< current electrical current */
    int _raw_pos        = 0;       /*!< raw position */
    int _ini_pos        = 0;       /*!< initial position */
    float _now_deg      = 0;       /*!< current angle [deg] */
    float _tar_deg      = FREE;    /*!< target angle [deg] */
    float _posing_scale = 0.03375; /*!< posing scale */

    unsigned long _move_start_time = 0; /*!< start time of moving */
    unsigned int _move_tar_time    = 0; /*!< target time of moving */
    float _move_start_pos          = 0; /*!< start position of moving */

    TimeCheck _moving_timer; /*!< timer for moving */

private:
    /*********************************************************
     * Constant
     *********************************************************/
    const int _SET_FREE_COUNT_MAX = 10; /*!<  */
    const int _IS_INVALID         = -1; /*!<  */

    const int _MAX_DEGREE = 360;  /*!<  */
    const int _MIN_DEGREE = -360; /*!<  */

    const int _DEG_SCALE = 10; /*!<  */

    const int _TARGET_POSITION_MIN = 3500;  /*!<  */
    const int _TARGET_POSITION_MAX = 11500; /*!<  */

    const float _TARGET_POS_MATCH_LIMIT = 1.0; /*!<  */

    const int _SERVO_STRC_MIN = 1;   /*!<  */
    const int _SERVO_STRC_MAX = 127; /*!<  */

    const int _SERVO_TEMPERATURE_X_MIN = 30;  /*!<  */
    const int _SERVO_TEMPERATURE_X_MAX = 75;  /*!<  */
    const int _SERVO_TEMPERATURE_Y_MIN = 100; /*!<  */
    const int _SERVO_TEMPERATURE_Y_MAX = 70;  /*!<  */

    const int _SERVO_CURRENT_LOWER_X_MIN = 0;     /*!<  */
    const int _SERVO_CURRENT_LOWER_X_MAX = 20;    /*!<  */
    const int _SERVO_CURRENT_LOWER_Y_MIN = -100;  /*!<  */
    const int _SERVO_CURRENT_LOWER_Y_MAX = -2000; /*!<  */

    const int _SERVO_CURRENT_UPPER_X_MIN = 64;   /*!<  */
    const int _SERVO_CURRENT_UPPER_X_MAX = 84;   /*!<  */
    const int _SERVO_CURRENT_UPPER_Y_MIN = 100;  /*!<  */
    const int _SERVO_CURRENT_UPPER_Y_MAX = 2000; /*!<  */

    const float _ANGLE_CHANGED_LIMIT = 0.1; /*!<  */

    const float _DEG_TO_POS_FACTOR     = 100.0; /*!<  */
    const float _MOVING_TAR_POS_FACTOR = 10.0;  /*!<  */

    const int _SEND_TAR_POS_X_MIN = 0;  /*!<  */
    const int _SEND_TAR_POS_X_MAX = 10; /*!<  */
    const int _SEND_TAR_POS_Y_MIN = 1;  /*!<  */
    const int _SEND_TAR_POS_Y_MAX = 10; /*!<  */
};

} // namespace arm_unit
} // namespace maid_robot_system

#endif
