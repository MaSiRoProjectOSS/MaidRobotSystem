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
void PartsEyelid::init(MIENS start_emotion, MIENS next_emotion, int transition_time)
{
    switch (start_emotion) {
        case MIENS::miens_smile:
        case MIENS::miens_close_left:
        case MIENS::miens_close_right:
        case MIENS::miens_close:
        case MIENS::miens_wink_left:
        case MIENS::miens_wink_right:
            this->_lib_animation = 29;
            break;
        case MIENS::miens_normal:
        default:
            break;
    }

    this->not_accepted(transition_time);
    this->set_emotion(next_emotion);
}
void PartsEyelid::closing()
{
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
    int buf_size   = 1000;
    bool flag_size = false;

    if (0 < param.left_eye.image.eyelid.size()) {
        this->left_eye.exit_eyelid = false;
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
                flag_size = true;
                buf_size  = std::min(list.max, list.max);
                this->left_eye.store.push_back(list);
            }
        }
        if (0 < (int)this->left_eye.store.size()) {
            this->left_eye.exit_eyelid = true;
        }
    }

    if (0 < param.right_eye.image.eyelid.size()) {
        this->right_eye.exit_eyelid = false;
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
                flag_size = true;
                buf_size  = std::min(list.max, list.max);
                this->right_eye.store.push_back(list);
            }
        }
        if (0 < (int)this->right_eye.store.size()) {
            this->right_eye.exit_eyelid = true;
        }
    }
    this->_store_size = (false == flag_size) ? 0 : buf_size;
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
    this->left_eye.ciliary_color.setRgb(param.left_eye.ciliary_color.r, param.left_eye.ciliary_color.g, param.left_eye.ciliary_color.b);
    this->right_eye.ciliary_color.setRgb(param.right_eye.ciliary_color.r, param.right_eye.ciliary_color.g, param.right_eye.ciliary_color.b);
    this->_reset_position(param);
#if DEBUG_OUTPUT_WIDGET
    printf("=============== Eyelid Postion ==============\n");
    printf("  rect\n");
    printf("     Left  (x,y), (w,h) = (%6.1f,%6.1f), (%6.1f,%6.1f)\n", this->left_eye.rect.x, this->left_eye.rect.y, this->left_eye.rect.width, this->left_eye.rect.height);
    printf("     Right (x,y), (w,h) = (%6.1f,%6.1f), (%6.1f,%6.1f)\n", this->right_eye.rect.x, this->right_eye.rect.y, this->right_eye.rect.width, this->right_eye.rect.height);
    printf("  center\n");
    printf("     Left  (x,y) = (%6.1f,%6.1f)\n", this->left_eye.pos_center.x, this->left_eye.pos_center.y);
    printf("     Right (x,y) = (%6.1f,%6.1f)\n", this->right_eye.pos_center.x, this->right_eye.pos_center.y);

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

void PartsEyelid::not_accepted(int elapsed)
{
    this->_block_time = elapsed;
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
uint PartsEyelid::get_ms_time(int time_current, int time_check, int add_Value)
{
    return (uint)((((time_current - time_check) / (this->_eye_blink_time / 2.0)) * this->_store_size) + add_Value);
}

void PartsEyelid::_reset_position(StParameter param)
{
    /**
     * The value to be set is the upper left coordinate of the drawing position as shown in the figure below
     *   Add the monitor's installation calibration value to this value to correct the position.
     *  postion_right = (window_size * ( 1.0 / 4.0 ) ) - (image_size / 2.0)
     *  postion_left  = (window_size * ( 3.0 / 4.0 ) ) - (image_size / 2.0)
     *
     *  |------------|------------|------------|------------|
     *  |    (R)                      (L)                   |
     *  |     |-------------|          |---------------|    |
     *  -     |             |          |               |    -
     *  |     |             |          |               |    |
     *  |     |             |          |               |    |
     *  -     |      +      |          |       +       |    -
     *  |     |             |          |               |    |
     *  |     |             |          |               |    |
     *  -     |             |          |               |    -
     *  |     |-------------|          |---------------|    |
     *  |                                                   |
     *  |------------|------------|------------|------------|
     */
    /* center position */
    this->left_eye.pos_center.set((param.screen_size.width * 0.75) + param.left_eye.eyelid.x, //
                                  (param.screen_size.height * 0.5) + param.left_eye.eyelid.y,
                                  0);
    this->right_eye.pos_center.set((param.screen_size.width * 0.25) + param.right_eye.eyelid.x, //
                                   (param.screen_size.height * 0.5) + param.right_eye.eyelid.y,
                                   0);
    /* Drawing range */
    this->left_eye.rect.set(this->left_eye.pos_center.x - (param.left_eye.eyelid.width * 0.5),
                            this->left_eye.pos_center.y - (param.left_eye.eyelid.height * 0.5),
                            param.left_eye.eyelid.width,
                            param.left_eye.eyelid.height);
    this->right_eye.rect.set(this->right_eye.pos_center.x - (param.right_eye.eyelid.width * 0.5),
                             this->right_eye.pos_center.y - (param.right_eye.eyelid.height * 0.5),
                             param.right_eye.eyelid.width,
                             param.right_eye.eyelid.height);
}

} // namespace maid_robot_system
