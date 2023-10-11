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
    left.set_elapsed(elapsed);
    right.set_elapsed(elapsed);

    if (this->lib_animation == 0) {
        switch (next_eye_emotion) {
            case miens_wink_left:
                if (miens_wink_left == previous_emotion) {
                    eye_emotion = miens_normal;
                } else {
                    start_animation  = true;
                    eye_emotion      = next_eye_emotion;
                    previous_emotion = eye_emotion;
                    set_onTheWay();
                }

                break;

            case miens_wink_right:
                if (miens_wink_right == previous_emotion) {
                    eye_emotion = miens_normal;
                } else {
                    start_animation  = true;
                    eye_emotion      = next_eye_emotion;
                    previous_emotion = eye_emotion;
                    set_onTheWay();
                }

                break;

            default:
                eye_emotion      = next_eye_emotion;
                previous_emotion = eye_emotion;
                break;
        }

        if (true == thinking) {
            eye_blink_time = this->set_eye_blink_time(blink_type::BLINK_TYPE_MIN_MAX);
            left.wink(eye_blink_time);
            right.wink(eye_blink_time);
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
    if ((true == left.enable_motion()) && (true == right.enable_motion())) {
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
    right.pos_center.set((param.screen_size.width * 0.25) + param.r_x, (param.screen_size.height * 0.25) + param.r_y);
    left.pos_center.set((param.screen_size.width * 0.75) + param.l_x, (param.screen_size.height * 0.25) + param.l_y);
    /* eyelid の描画開始座標 */
    right.pos.set(right.pos_center.x - (param.eyelid_size_x * 0.5),  // R_x: -86.000000
                  right.pos_center.y - (param.eyelid_size_y * 0.5)); // R_y: -280.000000
    left.pos.set(left.pos_center.x - (param.eyelid_size_x * 0.5),    // L_x: 1535.000000
                 left.pos_center.y - (param.eyelid_size_y * 0.5));   // L_y: -270.000000
}

/**
 * @brief
 *
 */
void PartsEyelid::set_onTheWay()
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

/**
 * @brief
 *
 * @param target
 * @return QPixmap
 */
QPixmap PartsEyelid::get_eye_id(ENUM_TARGET_EYE target)
{
    if (send_animation >= EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX) {
        send_animation = 0;
    }

    int index_elapsed          = send_animation;
    INDEX_LIP_IMAGE select_lid = INDEX_LIP_NORMAL;

    switch (eye_emotion) {
        case miens_close_left:
            index_elapsed = (int)((TARGET_EYE_LEFT == target) ? send_animation : (send_animation / winkValue));
            select_lid    = INDEX_LIP_SMILE;
            break;

        case miens_close_right:
            index_elapsed = (int)((TARGET_EYE_LEFT == target) ? (send_animation / winkValue) : send_animation);
            select_lid    = INDEX_LIP_SMILE;
            break;

        case miens_wink_left:
            index_elapsed = (int)((TARGET_EYE_LEFT == target) ? send_animation : (send_animation / 4));
            select_lid    = (TARGET_EYE_LEFT == target) ? INDEX_LIP_NORMAL : INDEX_LIP_SMILE;
            break;

        case miens_wink_right:
            index_elapsed = (int)((TARGET_EYE_LEFT == target) ? (send_animation / 4) : send_animation);
            select_lid    = (TARGET_EYE_LEFT == target) ? INDEX_LIP_SMILE : INDEX_LIP_NORMAL;
            break;

        case miens_normal:
        case miens_close:
            index_elapsed = send_animation;
            select_lid    = INDEX_LIP_NORMAL;
            break;

        case miens_smile:
        default:
            index_elapsed = send_animation;
            select_lid    = INDEX_LIP_SMILE;
            break;
            break;
    }

    switch (select_lid) {
        case INDEX_LIP_NORMAL:
            if (TARGET_EYE_LEFT == target) {
                return this->L_eyelid[index_elapsed];
            } else {
                return this->R_eyelid[index_elapsed];
            }

            break;

        case INDEX_LIP_SMILE:
        default:
            if (TARGET_EYE_LEFT == target) {
                return this->L_smile_lid[index_elapsed];
            } else {
                return this->R_smile_lid[index_elapsed];
            }

            break;
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
    char text_emotion[100];

    switch (next_eye_emotion) {
        case miens_smile:
        case miens_close_left:
        case miens_close_right:
        case miens_close:
            flag_EmotionKeep = true;
            break;

        case miens_normal:
        case miens_wink_left:
        case miens_wink_right:
        default:
            flag_EmotionKeep = false;
            break;
    }

    if (true == flagChange) {
        switch (next_eye_emotion) {
            case miens_smile:
                sprintf(text_emotion, "smile");
                break;

            case miens_close_left:
                sprintf(text_emotion, "lose_left");
                break;

            case miens_close_right:
                sprintf(text_emotion, "close_right");
                break;

            case miens_close:
                sprintf(text_emotion, "close");
                break;

            case miens_normal:
                sprintf(text_emotion, "normal");
                break;

            case miens_wink_left:
                sprintf(text_emotion, "wink_left");
                break;

            case miens_wink_right:
                sprintf(text_emotion, "wink_right");
                break;

            default:
                sprintf(text_emotion, "unknow");
                break;
        }

        printf("Emotion = %s\n", text_emotion);
    }
}
/**
 * @brief
 *
 * @param elapsed
 */
void PartsEyelid::set_cycle(uint elapsed)
{
}

void PartsEyelid::set_param(StParameter param)
{
    this->eye_blink_time_offset = param.eye_blink_time_offset;
    this->elapsed_next          = 0;

    // TODO
    printf("  ---- LOAD [Eyelid] ----\n");
    this->_set_image(param);
    this->_reset_position(param);
#if DEBUG_OUTPUT_WIDGET
    printf("=============== Eyelid Postion ==============\n");
    printf(" Right (x,y) = (%f,%f)\n", this->right.pos.x, this->right.pos.y);
    printf(" Left (x,y) = (%f,%f)\n", this->left.pos.x, this->left.pos.y);
    printf("=============================================\n");
#endif
    ////////////////
}
void PartsEyelid::_set_image(StParameter param)
{
    this->eye_blink_time_quickly      = param.eye_blink_time_quickly;
    this->eye_blink_time_min          = param.eye_blink_time_min;   // ms
    this->eye_blink_time_max          = param.eye_blink_time_max;   // ms
    this->eye_blink_time_limit        = param.eye_blink_time_limit; // ms
    Qt::ImageConversionFlag imageFlag = Qt::NoOpaqueDetection;
    imageFlag                         = Qt::OrderedAlphaDither;
    /* ============================================= */
    // wink anime  pre load///////////////////////////////////////////////
    printf("    register - %s\n", "Buffering image for eyelid");
    /* ============================================= */
    std::stringstream ss;
    std::string f_name;

    for (int buf_c = 0; buf_c < EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX; buf_c++) {
        ss.str("");
        ss.clear();
        ss << WORKSPACECONFIG_PATH_IMG_EYE << param.name << "eyelid/default/"
           << "Scene1_" << buf_c << ".png";
        f_name = ss.str();
        QPixmap buff(f_name.c_str(), nullptr, imageFlag);
        QMatrix rotate_angle_eyelid;
        rotate_angle_eyelid.rotate(-90);
        // left
        QMatrix rotate_angle_eyelid_l;
        rotate_angle_eyelid_l.rotate(-param.l_angle);
        this->L_eyelid[buf_c] = buff.transformed(rotate_angle_eyelid);
        this->L_eyelid[buf_c] = this->L_eyelid[buf_c].scaled(param.eyelid_size_x, param.eyelid_size_y, Qt::IgnoreAspectRatio);
        this->L_eyelid[buf_c] = this->L_eyelid[buf_c].transformed(rotate_angle_eyelid_l);
        // right
        QMatrix rotate_angle_eyelid_r;
        rotate_angle_eyelid_r.rotate(-param.r_angle);
        this->R_eyelid[buf_c] = buff.transformed(rotate_angle_eyelid);
        this->R_eyelid[buf_c] = this->R_eyelid[buf_c].transformed(QTransform().scale(-1, 1));
        this->R_eyelid[buf_c] = this->R_eyelid[buf_c].scaled(param.eyelid_size_x, param.eyelid_size_y, Qt::IgnoreAspectRatio);
        this->R_eyelid[buf_c] = this->R_eyelid[buf_c].transformed(rotate_angle_eyelid_r);
    }

    /* ============================================= */
    // smile anime  pre load///////////////////////////////////////////////
    printf("    register - %s\n", "Buffering image for eyelid with smile");

    /* ============================================= */
    for (int buf_c = 0; buf_c < EYE_WIDGET_EYELID_IMAGE_ARRAY_MAX; buf_c++) {
        ss.str("");
        ss.clear();
        ss << WORKSPACECONFIG_PATH_IMG_EYE << param.name << "eyelid/smile/"
           << "eye_smile1_" << buf_c << ".png";
        f_name = ss.str();
        QPixmap buff(f_name.c_str(), nullptr, imageFlag);
        QMatrix rotate_angle_smile_lid;
        rotate_angle_smile_lid.rotate(-90);
        // left
        QMatrix rotate_angle_smile_lid_l;
        rotate_angle_smile_lid_l.rotate(-param.l_angle);
        this->L_smile_lid[buf_c] = buff.transformed(rotate_angle_smile_lid);
        this->L_smile_lid[buf_c] = this->L_smile_lid[buf_c].scaled(param.eyelid_size_x, param.eyelid_size_y, Qt::IgnoreAspectRatio);
        this->L_smile_lid[buf_c] = this->L_smile_lid[buf_c].transformed(rotate_angle_smile_lid_l);
        // right
        QMatrix rotate_angle_smile_lid_r;
        rotate_angle_smile_lid_r.rotate(-param.r_angle);
        this->R_smile_lid[buf_c] = buff.transformed(rotate_angle_smile_lid);
        this->R_smile_lid[buf_c] = this->R_smile_lid[buf_c].transformed(QTransform().scale(-1, 1));
        this->R_smile_lid[buf_c] = this->R_smile_lid[buf_c].scaled(param.eyelid_size_x, param.eyelid_size_y, Qt::IgnoreAspectRatio);
        this->R_smile_lid[buf_c] = this->R_smile_lid[buf_c].transformed(rotate_angle_smile_lid_r);
    }
}

/**
 * @brief Construct a new Ctrl Eyelid:: Ctrl Eyelid object
 *
 */
PartsEyelid::PartsEyelid()
{
}

/* ============================================= */
/* ============================================= */
/* ============================================= */
/* ============================================= */
/* ============================================= */
/* ============================================= */
/* ============================================= */
/* ============================================= */
/* ============================================= */
/* ============================================= */
/* ============================================= */

/**
 * @brief
 *
 * @param current_time
 */
void StEyelid::debug_old_calc(uint current_time)
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
    debug_old_calc(current_time);
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
        this->left.wink(this->eye_blink_time);
        this->right.wink(this->eye_blink_time);
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
