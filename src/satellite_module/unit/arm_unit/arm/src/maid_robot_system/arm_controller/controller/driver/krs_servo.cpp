/**
 * @file krs_servo.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Handle KRS servo
 * @version 0.23.2
 * @date 2023-02-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "maid_robot_system/arm_controller/controller/driver/krs_servo.hpp"

namespace maid_robot_system
{
namespace arm_unit
{

KRSServo::KRSServo()
{
}

void KRSServo::setup(int id, int initial_pos, float scale, IcsHardSerialClass *krs)
{
    this->_ID = id;

    this->_servo        = krs;
    this->_ini_pos      = initial_pos;
    this->_posing_scale = this->_posing_scale * scale;
    this->_ave_tar_pos.reset(0);
    this->_moving_timer.update();
    if (this->_ID > 0) {
        for (int b = 0; b > this->_SET_FREE_COUNT_MAX; b++) {
            if (set_free() == this->_IS_INVALID) {
                this->_error_status = _ERROR_STATUS_ERROR;
            }
        }
    }
}

float KRSServo::get_deg()
{
    if (this->_ID > 0) {
        if (this->_error_status == _ERROR_STATUS_NORMAL) {
            set_free();
            this->_now_deg = (this->_raw_pos - this->_ini_pos) * this->_posing_scale;
            if (this->_now_deg > this->_MAX_DEGREE || this->_now_deg < this->_MIN_DEGREE) {
                this->_now_deg      = _ERROR;
                this->_error_status = _ERROR_STATUS_ERROR;
            }
        } else {
            return _ERROR;
        }
    }
    this->_ave_tar_pos.set(this->_now_deg * this->_DEG_SCALE) / (double)this->_DEG_SCALE;
    return this->_now_deg;
}

int KRSServo::set_free()
{
    if (this->_ID > 0) {
        if (this->_error_status == _ERROR_STATUS_NORMAL) {
            this->_mode               = FREE;
            this->_posing_finish_flag = _POSING_FINISHED;
            this->_raw_pos            = this->_servo->setFree(this->_ID);
            if (this->_raw_pos == -1) {
                this->_error_status = _ERROR_STATUS_ERROR;
            }
        } else {
            return _ERROR;
        }
    }
    return this->_raw_pos;
}

int KRSServo::check_unlock_error()
{
    if (this->_ID > 0) {
        if (this->_error_status == _ERROR_STATUS_ERROR) {
            if (set_free() != -1) {
                this->_now_deg      = (this->_raw_pos - this->_ini_pos) * this->_posing_scale;
                this->_error_status = _ERROR_STATUS_NORMAL;
                if (this->_now_deg > this->_MAX_DEGREE || this->_now_deg < this->_MIN_DEGREE) {
                    this->_now_deg      = _ERROR;
                    this->_error_status = _ERROR_STATUS_ERROR;
                }
            } else {
                this->_error_status = _ERROR_STATUS_ERROR;
            }
        }
    }
    return this->_error_status;
}

float KRSServo::set_pos(int _tar_pos)
{
    if (this->_ID > 0) {
        if (this->_error_status == _ERROR_STATUS_NORMAL) {
            this->_mode    = MOVING;
            _tar_pos       = constrain(_tar_pos, this->_TARGET_POSITION_MIN, this->_TARGET_POSITION_MAX);
            this->_raw_pos = this->_servo->setPos(this->_ID, _tar_pos);
            if (this->_raw_pos == -1) {
                this->_error_status = _ERROR_STATUS_ERROR;
            }
        } else {
            return _ERROR;
        }
    }
    return this->_raw_pos;
}

float KRSServo::set_deg(float target_deg)
{
    if (this->_error_status == _ERROR_STATUS_NORMAL) {
        int target = target_deg / this->_posing_scale + this->_ini_pos;
        set_pos(target);
        this->_now_deg = (this->_raw_pos - this->_ini_pos) * this->_posing_scale;
    } else {
        return _ERROR;
    }
    return this->_now_deg;
}

void KRSServo::set_target(float target_pos)
{
    if (abs(abs(target_pos) - PASS) < this->_TARGET_POS_MATCH_LIMIT) {        /* If this is the command to ignore, ignore it. */
    } else if (abs(abs(target_pos) - FREE) < this->_TARGET_POS_MATCH_LIMIT) { /* the value 777 is to set free. */
        this->_mode = FREE;
    } else {
        this->_mode    = MOVING;
        this->_tar_deg = target_pos;
    }
}

void KRSServo::set_strc(int _strc)
{
    if (this->_ID > 0) {
        if (this->_error_status == _ERROR_STATUS_NORMAL) {
            if (_strc != this->_now_strc) {
                this->_now_strc = _strc;
                _strc           = constrain(_strc, this->_SERVO_STRC_MIN, this->_SERVO_STRC_MAX);
                this->_servo->setStrc(this->_ID, _strc);
            }
        }
    }
}

void KRSServo::get_temp()
{
    if (this->_ID > 0) {
        if (this->_error_status == _ERROR_STATUS_NORMAL) {
            this->_now_temp = map(this->_servo->getTmp(this->_ID),
                                  this->_SERVO_TEMPERATURE_X_MIN,
                                  this->_SERVO_TEMPERATURE_X_MAX,
                                  this->_SERVO_TEMPERATURE_Y_MIN,
                                  this->_SERVO_TEMPERATURE_Y_MAX);
        }
    }
}
void KRSServo::get_current()
{
    if (this->_ID > 0) {
        if (this->_error_status == _ERROR_STATUS_NORMAL) {
            this->_now_current = this->_servo->getCur(this->_ID);
            if (this->_now_current > this->_SERVO_CURRENT_UPPER_X_MIN) {
                this->_now_current = map(this->_now_current,
                                         this->_SERVO_CURRENT_UPPER_X_MIN,
                                         this->_SERVO_CURRENT_UPPER_X_MAX,
                                         this->_SERVO_CURRENT_UPPER_Y_MIN,
                                         this->_SERVO_CURRENT_UPPER_Y_MAX);
            } else {
                this->_now_current = map(this->_now_current,
                                         this->_SERVO_CURRENT_LOWER_X_MIN,
                                         this->_SERVO_CURRENT_LOWER_X_MAX,
                                         this->_SERVO_CURRENT_LOWER_Y_MIN,
                                         this->_SERVO_CURRENT_LOWER_Y_MAX);
            }
        }
    }
}

float KRSServo::servo_move_time()
{
    /* Move along with "S" curve in specified time. */
    unsigned long moving_time = this->_moving_timer.get_elapsed_time();
    if (this->_move_tar_time < moving_time) {
        this->_posing_finish_flag = _POSING_FINISHED;
    }
    if (this->_posing_finish_flag == _POSING_FINISHED) {
        return this->_tar_deg;
    }

    float delta_deg = this->_tar_deg - this->_move_start_pos;
    float send_tar_pos;

    if (abs(delta_deg) > this->_ANGLE_CHANGED_LIMIT) {
        float moving_tar_pos = (this->_move_start_pos + delta_deg * this->_moving_timer.get_s_curve_flag(this->_move_tar_time)) * this->_DEG_TO_POS_FACTOR;

        if (this->_move_start_pos < this->_tar_deg) {
            moving_tar_pos = constrain(moving_tar_pos, this->_move_start_pos * this->_DEG_TO_POS_FACTOR, this->_tar_deg * this->_DEG_TO_POS_FACTOR) / this->_DEG_TO_POS_FACTOR;
        } else {
            moving_tar_pos = constrain(moving_tar_pos, this->_tar_deg * this->_DEG_TO_POS_FACTOR, this->_move_start_pos * this->_DEG_TO_POS_FACTOR) / this->_DEG_TO_POS_FACTOR;
        }

        send_tar_pos = this->_ave_tar_pos.set(moving_tar_pos * this->_MOVING_TAR_POS_FACTOR) / this->_MOVING_TAR_POS_FACTOR;

        if (this->_tar_strc == -1) {
            int move_strc = map(abs(send_tar_pos - this->_now_deg), this->_SEND_TAR_POS_X_MIN, this->_SEND_TAR_POS_X_MAX, this->_SEND_TAR_POS_Y_MIN, this->_SEND_TAR_POS_Y_MAX);
            set_strc(move_strc);
        }
    } else {
        send_tar_pos = this->_tar_deg;
    }

    return send_tar_pos;
}

void KRSServo::set_move_time(int set_time, float tar_pos)
{
    if (this->_error_status == _ERROR_STATUS_NORMAL) {
        if (tar_pos != FREE && tar_pos != PASS) {
            this->_mode               = MOVING;
            this->_posing_finish_flag = _POSING_NOT_FINISHED;
            this->_move_tar_time      = set_time;

            this->_moving_timer.update();
            this->_move_start_pos = this->_now_deg;
            set_target(tar_pos);
        } else {
            if (tar_pos == FREE) {
                this->_mode = FREE;
            }
            if (tar_pos == PASS) {
            }
        }
    }
}

float KRSServo::get_now_deg()
{
    return this->_now_deg;
}

void KRSServo::set_now_deg(float angle)
{
    this->_now_deg = angle;
}

void KRSServo::set_tar_strc(int str)
{
    this->_tar_strc = str;
}

int KRSServo::get_error_status()
{
    return (int)this->_error_status;
}

int KRSServo::get_mode()
{
    return this->_mode;
}

int KRSServo::get_posing_finish_flag()
{
    return (int)this->_posing_finish_flag;
}

} // namespace arm_unit
} // namespace maid_robot_system
