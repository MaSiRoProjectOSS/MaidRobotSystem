/**
 * @file control_waist.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief control the waist
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "wheel_controller/controller/control_waist.hpp"

#include "common/move_average.hpp"

namespace mobility_unit
{
namespace controller
{

ControlWaist::ControlWaist(uint8_t pin_pitch)
{
    this->_pin_pitch          = pin_pitch;
    this->_request_pitch      = 0;
    this->_current_pitch_leg  = 0;
    this->_current_pitch_gyro = 0;
    this->_correction_data    = 0;
    this->_monitoring.from_scratch();
}

bool ControlWaist::_begin()
{
    bool result = true;
    if (NUM_DIGITAL_PINS <= this->_pin_pitch) {
        result = false;
    }
    return result;
}
bool ControlWaist::_end()
{
    bool result = true;
    return result;
}
bool ControlWaist::_enable()
{
    // TODO : Waistのモーター制御は誤作動するため停止中
    return false;
}

bool ControlWaist::_calculate()
{
    static MoveAverage<10> average; /*!< moving average */

    bool result = true;
    // TODO : 一度に補正値を与えているため、補正値をいれたほうが良い。
    // TODO : レビュー後にドキュメントへ下記の項目は移動すること
    /* 案 :
        * バランス補正モード : 最初は早く動作して目的付近で補正
        * 水平維持モード : requestは無視して水平を維持
        * リズムモード : 環境音(音楽など）にしたがって揺らす
    */

    if (true == this->_monitoring.check_time_over(this->MONITORING_INTERVAL_MS)) {
        // todo : センサー値の更新がないなら、送信データも更新しない。
    } else {
        this->_correction_data =                                                        //
                average.set(map((this->_current_pitch_leg + this->_current_pitch_gyro), // current value
                                this->_SEND_IN_MIN,                                     // from range : low
                                this->_SEND_IN_MAX,                                     // from range : High
                                this->_SEND_OUT_MIN,                                    // to range : low
                                this->_SEND_OUT_MAX                                     // to range : High
                                )
                            + this->_request_pitch //
                            + this->_PITCH_OFFSET);
    }
    return result;
}

bool ControlWaist::_is_error()
{
    return false;
}

void ControlWaist::send()
{
    if (true == this->is_enable()) {
        analogWrite(this->_pin_pitch, this->_correction_data);
    }
}

void ControlWaist::set_data_from_sensor(int center_pitch_pos)
{
    this->_current_pitch_leg = center_pitch_pos;
    this->_monitoring.update();
}
void ControlWaist::set_data_from_CAN(int request_pitch)
{
    this->_request_pitch = map(request_pitch,        // current value
                               this->_PITCH_IN_MIN,  // from range : low
                               this->_PITCH_IN_MAX,  // from range : High
                               this->_PITCH_OUT_MIN, // to range : low
                               this->_PITCH_OUT_MAX  // to range : High
    );
}
void ControlWaist::set_data_from_gyro(CoordinateEuler gyro)
{
    // TODO : なんの計算のしているか確認する。
    this->_current_pitch_gyro = (gyro.pitch / 1.5) * 2.0 * 100.0;
}

int ControlWaist::get_target_pitch()
{
    return this->_correction_data;
}

} // namespace controller
} // namespace mobility_unit
