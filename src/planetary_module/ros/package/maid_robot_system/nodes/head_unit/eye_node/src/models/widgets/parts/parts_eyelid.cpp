/**
 * @file parts_eyelid.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/parts/parts_eyelid.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace maid_robot_system
{
#define EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX 30

/**
 * @brief
 *
 * @param time_current
 * @param time_check
 * @param add_Value
 * @return uint
 */
uint PartsEyelid::get_ms_time(int time_current, int time_check, int add_Value)
{
    return (uint)((((time_current - time_check) / (eye_blink_time / 2.0)) * EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX) + add_Value);
}

/**
 * @brief
 *
 * @param elapsed
 */
void PartsEyelid::calc_animation(int elapsed)
{
    static MIENS previous_emotion = NEXT_EMOTION_INIT;
    static bool start_animation   = false;
    this->left_eye.set_elapsed(elapsed);
    this->right_eye.set_elapsed(elapsed);

    if (this->lib_animation == 0) {
        switch (next_eye_emotion) {
            case miens_wink_left:
                if (miens_wink_left == previous_emotion) {
                    eye_emotion = miens_normal;
                } else {
                    start_animation  = true;
                    eye_emotion      = next_eye_emotion;
                    previous_emotion = eye_emotion;
                    this->set_on_the_way();
                }

                break;

            case miens_wink_right:
                if (miens_wink_right == previous_emotion) {
                    eye_emotion = miens_normal;
                } else {
                    start_animation  = true;
                    eye_emotion      = next_eye_emotion;
                    previous_emotion = eye_emotion;
                    this->set_on_the_way();
                }

                break;

            default:
                eye_emotion      = next_eye_emotion;
                previous_emotion = eye_emotion;
                break;
        }

        if (true == thinking) {
            eye_blink_time = this->set_eye_blink_time(blink_type::BLINK_TYPE_MIN_MAX);
            this->left_eye.wink(eye_blink_time);
            this->right_eye.wink(eye_blink_time);
            thinking = false;
        } else {
            if (this->elapsed_next <= elapsed) {
                this->elapsed_next = elapsed + this->set_eye_blink_time(blink_type::BLINK_TYPE_LONG);
                start_animation    = true;
            }
        }

        if ((true == flag_EmotionKeep) || (true == start_animation)) {
            qt_wink_anime_start_time = elapsed;
            this->set_eye_blink(blink_type::BLINK_TYPE_MIN_MAX, false);
            start_animation = false;
        }
    } else {
        if (this->lib_animation_flag == 0) {
            this->lib_animation = get_ms_time(elapsed, qt_wink_anime_start_time, 1.0);
        }

        if (this->lib_animation > 29 && this->lib_animation_flag == 0) {
            if ((false == flag_EmotionKeep) || (eye_emotion != next_eye_emotion)) {
                this->lib_animation_flag = 1;
                qt_wink_anime_start_time = elapsed;
            }

            this->lib_animation = 29;
        }

        if (this->lib_animation_flag == 1) {
            this->lib_animation = get_ms_time(elapsed, qt_wink_anime_start_time, EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX - 1);
        }

        if (this->lib_animation > 58) {
            this->lib_animation      = 0;
            this->lib_animation_flag = 0;
        }
    }

    if (this->lib_animation < 29) {
        send_animation = this->lib_animation;
    } else {
        send_animation = 58 - this->lib_animation;
    }
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool PartsEyelid::enable_motion()
{
    if ((true == this->left_eye.enable_motion()) && (true == this->right_eye.enable_motion())) {
        return true;
    } else {
        return false;
    }
}

void PartsEyelid::_reset_position(StParameter param)
{
    /**
     *  設定する値は下図に示すように描画位置の左上座標
     *    これにモニタの設置キャリブレーション値を追加して位置の補正を行う
     *
     *  |-----------------------|-----------------------|-----------------------|-----------------------|
     *  |                                                                                               |
     *  |        (R)                                            (L)                                     |
     *  |         |---------------------------|                  |---------------------------|          |
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  -         |                           |                  |                           |          -
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  -         |             +             |                  |             +             |          -
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  -         |                           |                  |                           |          -
     *  |         |                           |                  |                           |          |
     *  |         |                           |                  |                           |          |
     *  |         |---------------------------|                  |---------------------------|          |
     *  |                                                                                               |
     *  |                                                                                               |
     *  |-----------------------|-----------------------|-----------------------|-----------------------|
     *
     *  Rの位置計算 = (ウィンドウサイズ * ( 1.0 / 4.0 ) ) - (eyelidのイメージサイズ / 2.0)
     *  Lの位置計算 = (ウィンドウサイズ * ( 3.0 / 4.0 ) ) - (eyelidのイメージサイズ / 2.0)
     */
    /* eyelid の中心座標 */
    this->left_eye.pos_center.set((param.screen_size.width * 0.75) + param.left_eye.eyelid.x, (param.screen_size.height * 0.25) + param.left_eye.eyelid.y, 0);
    this->right_eye.pos_center.set((param.screen_size.width * 0.25) + param.right_eye.eyelid.x, (param.screen_size.height * 0.25) + param.right_eye.eyelid.y, 0);
    /* eyelid の描画開始座標 */
    this->right_eye.pos.set_axis(this->right_eye.pos_center.x - (param.right_eye.eyelid.width * 0.5),   // R_x: -86.000000
                                 this->right_eye.pos_center.y - (param.right_eye.eyelid.height * 0.5)); // R_y: -280.000000
    this->left_eye.pos.set_axis(this->left_eye.pos_center.x - (param.left_eye.eyelid.width * 0.5),      // L_x: 1535.000000
                                this->left_eye.pos_center.y - (param.left_eye.eyelid.height * 0.5));    // L_y: -270.000000
}

/**
 * @brief
 *
 */
void PartsEyelid::set_on_the_way()
{
    winkValue = func_rand(3.0, 4.1);
    winkValue = func_rand(3.0, 4.1);
}

/**
 * @brief
 *
 */
void PartsEyelid::cycle()
{
}

QPixmap PartsEyelid::get_eye_id_right()
{
    return this->_get_eye_id(false);
}
QPixmap PartsEyelid::get_eye_id_left()
{
    return this->_get_eye_id(true);
}
QPixmap PartsEyelid::_get_eye_id(bool is_left)
{
    if (send_animation >= EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX) {
        send_animation = 0;
    }

    int index_elapsed = send_animation;
    int select_lid    = (int)INDEX_LIP_NORMAL;

    switch (eye_emotion) {
        case MIENS::miens_close_left:
            index_elapsed = (int)((is_left) ? send_animation : (send_animation / winkValue));
            select_lid    = (int)INDEX_LIP_SMILE;
            break;

        case MIENS::miens_close_right:
            index_elapsed = (int)((is_left) ? (send_animation / winkValue) : send_animation);
            select_lid    = (int)INDEX_LIP_SMILE;
            break;

        case MIENS::miens_wink_left:
            index_elapsed = (int)((is_left) ? send_animation : (send_animation / 4));
            select_lid    = (int)((is_left) ? INDEX_LIP_NORMAL : INDEX_LIP_SMILE);
            break;

        case MIENS::miens_wink_right:
            index_elapsed = (int)((is_left) ? (send_animation / 4) : send_animation);
            select_lid    = (int)((is_left) ? INDEX_LIP_SMILE : INDEX_LIP_NORMAL);
            break;

        case MIENS::miens_smile:
            index_elapsed = send_animation;
            select_lid    = (int)INDEX_LIP_SMILE;
            break;

        case MIENS::miens_normal:
        case MIENS::miens_close:
        default:
            index_elapsed = send_animation;
            select_lid    = (int)INDEX_LIP_NORMAL;
            break;
            break;
    }
    if (is_left) {
        if ((int)this->left_eye.store.size() == 0) {
            return this->_blank;
        } else {
            if ((int)this->left_eye.store.size() <= select_lid) {
                select_lid = 0;
            }
            if (this->left_eye.store[select_lid].max <= index_elapsed) {
                index_elapsed = 0;
            }
            return this->left_eye.store[select_lid].data[index_elapsed];
        }
    } else {
        if (this->right_eye.store.size() == 0) {
            return this->_blank;
        } else {
            if ((int)this->right_eye.store.size() <= select_lid) {
                select_lid = 0;
            }
            if (this->right_eye.store[select_lid].max <= index_elapsed) {
                index_elapsed = 0;
            }
            return this->right_eye.store[select_lid].data[index_elapsed];
        }
    }
}

/**
 * @brief
 *
 * @param eye_emotion
 */
void PartsEyelid::set_emotion(MIENS eye_emotion)
{
    bool flagChange  = (next_eye_emotion == eye_emotion) ? false : true;
    next_eye_emotion = eye_emotion;

    switch (next_eye_emotion) {
        case MIENS::miens_smile:
        case MIENS::miens_close_left:
        case MIENS::miens_close_right:
        case MIENS::miens_close:
            flag_EmotionKeep = true;
            break;

        case MIENS::miens_normal:
        case MIENS::miens_wink_left:
        case MIENS::miens_wink_right:
        default:
            flag_EmotionKeep = false;
            break;
    }

#if DEBUG_OUTPUT_BEHAVIOR
    char text_emotion[100];
    if (true == flagChange) {
        switch (next_eye_emotion) {
            case MIENS::miens_smile:
                sprintf(text_emotion, "smile");
                break;

            case MIENS::miens_close_left:
                sprintf(text_emotion, "lose_left");
                break;

            case MIENS::miens_close_right:
                sprintf(text_emotion, "close_right");
                break;

            case MIENS::miens_close:
                sprintf(text_emotion, "close");
                break;

            case MIENS::miens_normal:
                sprintf(text_emotion, "normal");
                break;

            case MIENS::miens_wink_left:
                sprintf(text_emotion, "wink_left");
                break;

            case MIENS::miens_wink_right:
                sprintf(text_emotion, "wink_right");
                break;

            default:
                sprintf(text_emotion, "unknow");
                break;
        }

        printf("Emotion = %s\n", text_emotion);
    }
#endif
}

void PartsEyelid::set_cycle(uint elapsed)
{
}

void PartsEyelid::load(StParameter param)
{
    this->_set_image(param);
}

void PartsEyelid::set_param(StParameter param)
{
    // TODO
    this->eye_blink_time_offset = param.blink_time_offset;
    this->elapsed_next          = 0;

    this->color.setRgb(param.eyelid_color.r, param.eyelid_color.g, param.eyelid_color.b);
    this->_reset_position(param);
#if DEBUG_OUTPUT_WIDGET
    printf("=============== Eyelid Postion ==============\n");
    printf(" Right (x,y) = (%f,%f)\n", this->right_eye.pos.x, this->right_eye.pos.y);
    printf(" Left (x,y) = (%f,%f)\n", this->left_eye.pos.x, this->left_eye.pos.y);
    printf("=============================================\n");
#endif
}
void PartsEyelid::_set_image(StParameter param)
{
    this->eye_blink_time_quickly = param.blink_time_quickly;
    this->eye_blink_time_min     = param.blink_time_min;   // ms
    this->eye_blink_time_max     = param.blink_time_max;   // ms
    this->eye_blink_time_limit   = param.blink_time_limit; // ms

    std::string f_name;
#if DEBUG_OUTPUT_LOAD_IMAGE
    printf("  ---- LOAD [Eyelid] ----\n");
#endif
    if (0 < param.left_eye.image.eyelid.size()) {
        this->left_eye.store.clear();
        for (size_t i = 0; i < param.left_eye.image.eyelid.size(); ++i) {
            StImageMap list;
            list.data.clear();
            list.id = param.left_eye.image.eyelid[i].id;
            for (size_t j = 0; j < param.left_eye.image.eyelid[i].files.size(); ++j) {
                f_name = param.left_eye.image.eyelid[i].files[j];
                if (true == std::filesystem::exists(f_name)) {
                    QPixmap buff(f_name.c_str(), nullptr, param.imageFlag);
                    QMatrix rotate_angle;
                    if (true == param.left_eye.image.eyelid[i].mirror) {
                        buff = buff.transformed(QTransform().scale(-1, 1));
                    }
                    buff = buff.scaled(param.left_eye.eyelid.width, param.left_eye.eyelid.height, Qt::IgnoreAspectRatio);

                    rotate_angle.rotate(param.left_eye.eyelid.angle);
                    buff = buff.transformed(rotate_angle);
                    list.data.push_back(buff);
                }
            }
            list.max = (int)list.data.size();
            if (0 < list.max) {
                this->left_eye.store.push_back(list);
            }
        }
    }

    if (0 < param.right_eye.image.eyelid.size()) {
        this->right_eye.store.clear();
        for (size_t i = 0; i < param.right_eye.image.eyelid.size(); ++i) {
            StImageMap list;
            list.data.clear();
            list.id = param.right_eye.image.eyelid[i].id;
            for (size_t j = 0; j < param.right_eye.image.eyelid[i].files.size(); ++j) {
                f_name = param.right_eye.image.eyelid[i].files[j];
                if (true == std::filesystem::exists(f_name)) {
                    QPixmap buff(f_name.c_str(), nullptr, param.imageFlag);
                    QMatrix rotate_angle;
                    if (true == param.right_eye.image.eyelid[i].mirror) {
                        buff = buff.transformed(QTransform().scale(-1, 1));
                    }
                    buff = buff.scaled(param.right_eye.eyelid.width, param.right_eye.eyelid.height, Qt::IgnoreAspectRatio);

                    rotate_angle.rotate(param.right_eye.eyelid.angle);
                    buff = buff.transformed(rotate_angle);
                    list.data.push_back(buff);
                }
            }
            list.max = (int)list.data.size();
            if (0 < list.max) {
                this->right_eye.store.push_back(list);
            }
        }
    }
}

/**
 * @brief Construct a new Ctrl Eyelid:: Ctrl Eyelid object
 *
 */
PartsEyelid::PartsEyelid()
{
}

/**
 * @brief
 *
 */
void StEyelid::fin_wink()
{
    _start_time      = 0xFFFFFFFF;
    _wink            = false;
    _elapsedIndex    = 0;
    _flag_next_blink = false;
}
/**
 * @brief
 *
 * @return StEyelid::
 */
StEyelid::StEyelid()
{
    fin_wink();
}

/**
 * @brief
 *
 * @return uint
 */
uint StEyelid::get_elapsed()
{
    return _elapsedIndex;
}

/**
 * @brief
 *
 * @param wink_time_millisecond
 */
void StEyelid::open_eye(uint wink_time_millisecond)
{
    // open eye during one wink
    if (true == _wink) {
        _start_time            = (_start_time + _wink_time_millisecond) - wink_time_millisecond;
        _wink_time_millisecond = wink_time_millisecond;
    }
}

/**
 * @brief
 *
 * @param wink_time_millisecond
 */
void StEyelid::wink(uint wink_time_millisecond)
{
    if (false == _wink) {
        if (0 == _elapsedIndex) {
            _start_time = _current_time;
        }

        _wink_time_millisecond = wink_time_millisecond;
        _wink                  = true;
    } else {
        open_eye(wink_time_millisecond);
    }
}

/**
 * @brief
 *
 * @param current_time
 */
void StEyelid::set_elapsed(uint current_time)
{
#if 1

#else
    int elapsedIndex = _elapsedIndex;
    _current_time    = current_time;

    if (_start_time < current_time) {
        elapsedIndex = (int)((current_time - _start_time) / _wink_time_millisecond);

        if (EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX <= elapsedIndex) {
            if ((EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX * 2) > elapsedIndex) {
                elapsedIndex = EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX - elapsedIndex;
            } else {
                // finished wink
                fin_wink();
            }
        }
    }

    _elapsedIndex = elapsedIndex;

    if (true == enable_motion()) {
        if (false == _flag_next_blink) {
            _eye_blink_time  = this->set_eye_blink_time(blink_type::BLINK_TYPE_LONG) + current_time;
            _flag_next_blink = true;
        }

        if (_eye_blink_time < current_time) {
            wink(default_wink_time_millisecond);
        }
    }

#endif
}

void PartsEyelid::set_eye_blink(blink_type eye_emotion, bool start_flag)
{
    this->eye_blink_time = this->set_eye_blink_time(eye_emotion);
    this->lib_animation++;

    if (true == start_flag) {
        this->left_eye.wink(this->eye_blink_time);
        this->right_eye.wink(this->eye_blink_time);
        this->thinking = true;
    }
}

double PartsEyelid::set_eye_blink_time(blink_type type)
{
    switch (type) {
        case blink_type::BLINK_TYPE_QUICKLY:
            return func_rand(this->eye_blink_time_min - this->eye_blink_time_quickly, this->eye_blink_time_max - this->eye_blink_time_quickly);

        case blink_type::BLINK_TYPE_MIN_MAX:
            return func_rand(this->eye_blink_time_min, this->eye_blink_time_max);

        case blink_type::BLINK_TYPE_LONG:
        default:
            return func_rand(this->eye_blink_time_offset + this->eye_blink_time_min, this->eye_blink_time_offset + this->eye_blink_time_limit);
    }
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool StEyelid::enable_motion()
{
    return !_wink;
}

} // namespace maid_robot_system
