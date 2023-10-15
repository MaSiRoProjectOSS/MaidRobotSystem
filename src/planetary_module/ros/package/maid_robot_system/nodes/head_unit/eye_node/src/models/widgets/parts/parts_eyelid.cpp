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

// =============================
// Constructor
// =============================
PartsEyelid::PartsEyelid()
{
}
PartsEyelid::~PartsEyelid()
{
}

// =============================
// PUBLIC : Function
// =============================
int PartsEyelid::calc_animation(int elapsed)
{
    static MIENS previous_emotion    = NEXT_EMOTION_INIT;
    static bool start_animation      = false;
    static int wink_anime_start_time = 0;
    static int flag_animation        = 0;
    this->left_eye.set_elapsed(elapsed);
    this->right_eye.set_elapsed(elapsed);

    if (this->_lib_animation == 0) {
        switch (this->_next_eye_emotion) {
            case miens_wink_left:
                if (miens_wink_left == previous_emotion) {
                    this->eye_emotion = miens_normal;
                } else {
                    start_animation   = true;
                    this->eye_emotion = this->_next_eye_emotion;
                    previous_emotion  = this->eye_emotion;
                    this->set_on_the_way();
                }

                break;

            case miens_wink_right:
                if (miens_wink_right == previous_emotion) {
                    this->eye_emotion = miens_normal;
                } else {
                    start_animation   = true;
                    this->eye_emotion = this->_next_eye_emotion;
                    previous_emotion  = this->eye_emotion;
                    this->set_on_the_way();
                }

                break;

            default:
                this->eye_emotion = this->_next_eye_emotion;
                previous_emotion  = this->eye_emotion;
                break;
        }

        if (true == this->_thinking) {
            this->_eye_blink_time = this->set_eye_blink_time(blink_type::BLINK_TYPE_MIN_MAX);
            this->left_eye.wink(this->_eye_blink_time);
            this->right_eye.wink(this->_eye_blink_time);
            this->_thinking = false;
        } else {
            if (this->_elapsed_next <= elapsed) {
                this->_elapsed_next = elapsed + this->set_eye_blink_time(blink_type::BLINK_TYPE_LONG);
                start_animation     = true;
            }
        }

        if ((true == this->_flag_EmotionKeep) || (true == start_animation)) {
            wink_anime_start_time = elapsed;
            this->set_eye_blink(blink_type::BLINK_TYPE_MIN_MAX, false);
            start_animation = false;
        }
    } else {
        if (flag_animation == 0) {
            this->_lib_animation = get_ms_time(elapsed, wink_anime_start_time, 1.0);
        }

        if (this->_lib_animation > 29 && flag_animation == 0) {
            if ((false == this->_flag_EmotionKeep) || (eye_emotion != this->_next_eye_emotion)) {
                flag_animation        = 1;
                wink_anime_start_time = elapsed;
            }

            this->_lib_animation = 29;
        }

        if (flag_animation == 1) {
            this->_lib_animation = get_ms_time(elapsed, wink_anime_start_time, EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX - 1);
        }

        if (this->_lib_animation > 58) {
            this->_lib_animation = 0;
            flag_animation       = 0;
        }
    }

    if (this->_lib_animation < 29) {
        this->_send_animation = this->_lib_animation;
    } else {
        this->_send_animation = 58 - this->_lib_animation;
    }
    return this->_send_animation;
}

void PartsEyelid::load(StParameter param)
{
    this->_eye_blink_time_quickly = param.blink_time_quickly;
    this->_eye_blink_time_min     = param.blink_time_min;   // ms
    this->_eye_blink_time_max     = param.blink_time_max;   // ms
    this->_eye_blink_time_limit   = param.blink_time_limit; // ms

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

// =============================
// PUBLIC : Setter
// =============================
void PartsEyelid::set_param(StParameter param)
{
    // TODO
    this->_eye_blink_time_offset = param.blink_time_offset;
    this->_elapsed_next          = 0;

    this->color.setRgb(param.eyelid_color.r, param.eyelid_color.g, param.eyelid_color.b);
    this->_reset_position(param);
#if DEBUG_OUTPUT_WIDGET
    printf("=============== Eyelid Postion ==============\n");
    printf("  position\n");
    printf("     Left (x,y) = (%f,%f)\n", this->left_eye.pos.x, this->left_eye.pos.y);
    printf("     Right (x,y) = (%f,%f)\n", this->right_eye.pos.x, this->right_eye.pos.y);
    printf("  center\n");
    printf("   Left (x,y) = (%f,%f)\n", this->left_eye.pos_center.x, this->left_eye.pos_center.y);
    printf("   Right (x,y) = (%f,%f)\n", this->right_eye.pos_center.x, this->right_eye.pos_center.y);
    printf("=============================================\n");
#endif
}

void PartsEyelid::set_eye_blink(blink_type eye_emotion, bool start_flag)
{
    this->_eye_blink_time = this->set_eye_blink_time(eye_emotion);
    this->_lib_animation++;

    if (true == start_flag) {
        this->left_eye.wink(this->_eye_blink_time);
        this->right_eye.wink(this->_eye_blink_time);
        this->_thinking = true;
    }
}

double PartsEyelid::set_eye_blink_time(blink_type type)
{
    switch (type) {
        case blink_type::BLINK_TYPE_QUICKLY:
            return func_rand(this->_eye_blink_time_min - this->_eye_blink_time_quickly, this->_eye_blink_time_max - this->_eye_blink_time_quickly);

        case blink_type::BLINK_TYPE_MIN_MAX:
            return func_rand(this->_eye_blink_time_min, this->_eye_blink_time_max);

        case blink_type::BLINK_TYPE_LONG:
        default:
            return func_rand(this->_eye_blink_time_offset + this->_eye_blink_time_min, this->_eye_blink_time_offset + this->_eye_blink_time_limit);
    }
}

void PartsEyelid::set_emotion(MIENS eye_emotion)
{
    bool flagChange         = (this->_next_eye_emotion == eye_emotion) ? false : true;
    this->_next_eye_emotion = eye_emotion;

    switch (this->_next_eye_emotion) {
        case MIENS::miens_smile:
        case MIENS::miens_close_left:
        case MIENS::miens_close_right:
        case MIENS::miens_close:
            this->_flag_EmotionKeep = true;
            break;

        case MIENS::miens_normal:
        case MIENS::miens_wink_left:
        case MIENS::miens_wink_right:
        default:
            this->_flag_EmotionKeep = false;
            break;
    }

#if DEBUG_OUTPUT_BEHAVIOR
    char text_emotion[100];
    if (true == flagChange) {
        switch (this->_next_eye_emotion) {
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

void PartsEyelid::set_on_the_way()
{
    this->_wink_value = func_rand(3.0, 4.1);
}

// =============================
// PUBLIC : Getter
// =============================
bool PartsEyelid::enable_motion()
{
    if ((true == this->left_eye.enable_motion()) && (true == this->right_eye.enable_motion())) {
        return true;
    } else {
        return false;
    }
}
QPixmap PartsEyelid::get_eye_id_right()
{
    return this->_get_eye_id(false);
}
QPixmap PartsEyelid::get_eye_id_left()
{
    return this->_get_eye_id(true);
}
uint PartsEyelid::get_ms_time(int time_current, int time_check, int add_Value)
{
    return (uint)((((time_current - time_check) / (this->_eye_blink_time / 2.0)) * EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX) + add_Value);
}

// =============================
// PRIVATE : Function
// =============================
QPixmap PartsEyelid::_get_eye_id(bool is_left)
{
    if (this->_send_animation >= EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX) {
        this->_send_animation = 0;
    }

    int index_elapsed = this->_send_animation;
    int select_lid    = (int)INDEX_LIP_NORMAL;

    switch (eye_emotion) {
        case MIENS::miens_close_left:
            index_elapsed = (int)((is_left) ? this->_send_animation : (this->_send_animation / this->_wink_value));
            select_lid    = (int)INDEX_LIP_SMILE;
            break;

        case MIENS::miens_close_right:
            index_elapsed = (int)((is_left) ? (this->_send_animation / this->_wink_value) : this->_send_animation);
            select_lid    = (int)INDEX_LIP_SMILE;
            break;

        case MIENS::miens_wink_left:
            index_elapsed = (int)((is_left) ? this->_send_animation : (this->_send_animation / 4));
            select_lid    = (int)((is_left) ? INDEX_LIP_NORMAL : INDEX_LIP_SMILE);
            break;

        case MIENS::miens_wink_right:
            index_elapsed = (int)((is_left) ? (this->_send_animation / 4) : this->_send_animation);
            select_lid    = (int)((is_left) ? INDEX_LIP_SMILE : INDEX_LIP_NORMAL);
            break;

        case MIENS::miens_smile:
            index_elapsed = this->_send_animation;
            select_lid    = (int)INDEX_LIP_SMILE;
            break;

        case MIENS::miens_normal:
        case MIENS::miens_close:
        default:
            index_elapsed = this->_send_animation;
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
    this->left_eye.pos_center.set((param.screen_size.width * 0.75) + param.left_eye.eyelid.x, (param.screen_size.height * 0.5) + param.left_eye.eyelid.y, 0);
    this->right_eye.pos_center.set((param.screen_size.width * 0.25) + param.right_eye.eyelid.x, (param.screen_size.height * 0.5) + param.right_eye.eyelid.y, 0);
    /* eyelid の描画開始座標 */
    this->right_eye.pos.set_axis(this->right_eye.pos_center.x - (param.right_eye.eyelid.width * 0.5),   //
                                 this->right_eye.pos_center.y - (param.right_eye.eyelid.height * 0.5)); //
    this->left_eye.pos.set_axis(this->left_eye.pos_center.x - (param.left_eye.eyelid.width * 0.5),      //
                                this->left_eye.pos_center.y - (param.left_eye.eyelid.height * 0.5));    //
}

} // namespace maid_robot_system
