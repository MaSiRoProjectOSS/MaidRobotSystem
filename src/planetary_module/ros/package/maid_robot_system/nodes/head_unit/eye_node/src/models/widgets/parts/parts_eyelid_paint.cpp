/**
 * @file parts_eyelid.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/parts/parts_eyelid.hpp"

namespace maid_robot_system
{
// =============================
// PRIVATE : Function
// =============================
void PartsEyelid::update_background(QPainter &painter, St2DRectangle screen_size)
{
    painter.fillRect(screen_size.x, //
                     screen_size.y,
                     screen_size.width,
                     screen_size.height,
                     this->_eyelid_color);

    // TODO : Write with polygons.    Frame is -1
    painter.fillRect((int)this->left_eye.rect.x, //
                     (int)this->left_eye.rect.y,
                     (int)this->left_eye.rect.width,
                     (int)this->left_eye.rect.height,
                     this->left_eye.ciliary_color);
    painter.fillRect((int)this->right_eye.rect.x, //
                     (int)this->right_eye.rect.y,
                     (int)this->right_eye.rect.width,
                     (int)this->right_eye.rect.height,
                     this->right_eye.ciliary_color);
}
void PartsEyelid::update(QPainter &painter)
{
    if (true == this->right_eye.exit_eyelid) {
        painter.drawPixmap(this->right_eye.rect.x, //
                           this->right_eye.rect.y,
                           this->right_eye.rect.width,
                           this->right_eye.rect.height,
                           this->_get_eye_id(false));
    }
    if (true == this->left_eye.exit_eyelid) {
        painter.drawPixmap(this->left_eye.rect.x, //
                           this->left_eye.rect.y,
                           this->left_eye.rect.width,
                           this->left_eye.rect.height,
                           this->_get_eye_id(true));
    }
}

int PartsEyelid::calculate(int elapsed)
{
    static MIENS previous_emotion    = NEXT_EMOTION_INIT;
    static bool start_animation      = false;
    static int wink_anime_start_time = 0;
    static int flag_animation        = 0;
    this->left_eye.set_elapsed(elapsed);
    this->right_eye.set_elapsed(elapsed);

    if (this->_block_time <= elapsed) {
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

            if ((true == this->_flag_keep) || (true == start_animation)) {
                wink_anime_start_time = elapsed;
                this->set_eye_blink(blink_type::BLINK_TYPE_MIN_MAX, false);
                start_animation = false;
            }
        } else {
            if (flag_animation == 0) {
                this->_lib_animation = get_ms_time(elapsed, wink_anime_start_time, 1.0);
            }

            if (this->_lib_animation > 29 && flag_animation == 0) {
                if ((false == this->_flag_keep) || (this->eye_emotion != this->_next_eye_emotion)) {
                    flag_animation        = 1;
                    wink_anime_start_time = elapsed;
                }

                this->_lib_animation = 29;
            }

            if (flag_animation == 1) {
                this->_lib_animation = get_ms_time(elapsed, wink_anime_start_time, this->_store_size - 1);
            }

            if (this->_lib_animation > 58) {
                this->_lib_animation = 0;
                flag_animation       = 0;
            }
        }
    }

    this->_progress = (int)(((30 - this->_lib_animation) / 30.0) * 100);
    if (this->_lib_animation < 29) {
        this->_send_animation = this->_lib_animation;
    } else {
        this->_send_animation = 58 - this->_lib_animation;
    }

    return this->_progress;
}

// =============================
// PRIVATE : Function
// =============================
QPixmap PartsEyelid::_get_eye_id(bool is_left)
{
    if (this->_send_animation >= this->_store_size) {
        this->_send_animation = 0;
    }

    int index_elapsed = this->_send_animation;
    int select_lid    = (int)INDEX_LIP_NORMAL;

    switch (this->eye_emotion) {
        case MIENS::miens_keep_normal:
            index_elapsed = 0;
            select_lid    = (int)INDEX_LIP_NORMAL;
            break;
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

} // namespace maid_robot_system
