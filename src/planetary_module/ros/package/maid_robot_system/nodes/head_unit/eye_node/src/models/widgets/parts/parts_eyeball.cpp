/**
 * @file parts_eyeball.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/parts/parts_eyeball.hpp"

#include <filesystem>

namespace maid_robot_system
{
#ifndef M_PI
#define M_PI 3.14159265359
#endif

/**
 * @brief 位置のデフォルト化
 *
 */
void PartsEyeball::set_default()
{
    this->right_eye.target.set(0, 0, 0);
    this->left_eye.target.set(0, 0, 0);
    this->set_dimensions(EYEBALL_DIMENSIONS_DEFAULT);
}

/**
 * @brief ひとみの大きさ設定
 *
 * @param value 倍率
 */
void PartsEyeball::set_dimensions(float value)
{
    /* 入力値が小さい場合は、最小値に設定 */
    if (EYEBALL_DIMENSIONS_MIN > value) {
        request_dimensions = EYEBALL_DIMENSIONS_MIN;
    }

    /* 入力値が大きい場合は、最大値に設定 */
    else if (EYEBALL_DIMENSIONS_MAX < value) {
        request_dimensions = EYEBALL_DIMENSIONS_MAX;
    }

    /* 入力値を設定 */
    else {
        request_dimensions = value;
    }
}

/**
 * @brief 外周リングの描画
 *
 */
void PartsEyeball::draw_outside()
{
#if DRAW_CORNEA_OUTSIDE
    /* ============================================= */
    // left
    /* ============================================= */
    if (0 < (int)this->left_eye.store.cornea_outside.size()) {
        this->left_eye.store.matrix_cornea_outside.reset();
        this->left_eye.store.matrix_cornea_outside.scale((this->left_eye.draw_postion.width - 20) / this->left_eye.size.x, //
                                                         (this->left_eye.draw_postion.height + 60) / this->left_eye.size.y);
        this->left_eye.store.matrix_cornea_outside.rotate(this->left_eye.cornea_outside_angle);
        this->left_eye.cornea_outside      = this->left_eye.store.cornea_outside[this->get_index()].data[0].transformed(this->left_eye.store.matrix_cornea_outside);
        this->left_eye.draw_cornea_anime.x = this->left_eye.eyeball_center.x() - (this->left_eye.cornea_outside.width() / 2.0);
        this->left_eye.draw_cornea_anime.y = this->left_eye.eyeball_center.y() - (this->left_eye.cornea_outside.height() / 2.0);
    }
    /* ============================================= */
    // right
    /* ============================================= */
    if (0 < (int)this->right_eye.store.cornea_outside.size()) {
        this->right_eye.store.matrix_cornea_outside.reset();
        this->right_eye.store.matrix_cornea_outside.scale((this->right_eye.draw_postion.width - 20) / this->right_eye.size.x, //
                                                          (this->right_eye.draw_postion.height + 60) / this->right_eye.size.y);
        this->right_eye.store.matrix_cornea_outside.rotate(this->right_eye.cornea_outside_angle);
        this->right_eye.cornea_outside      = this->right_eye.store.cornea_outside[this->get_index()].data[0].transformed(this->right_eye.store.matrix_cornea_outside);
        this->right_eye.draw_cornea_anime.x = this->right_eye.eyeball_center.x() - (this->right_eye.cornea_outside.width() / 2.0);
        this->right_eye.draw_cornea_anime.y = this->right_eye.eyeball_center.y() - (this->right_eye.cornea_outside.height() / 2.0);
    }
#endif
}

/**
 * @brief 内周リングの描画
 *
 */
void PartsEyeball::draw_inside()
{
#if DRAW_CORNEA_INSIDE
    /* ============================================= */
    // left
    /* ============================================= */
    if (0 < (int)this->right_eye.store.cornea_inside.size()) {
        this->left_eye.store.matrix_cornea_inside.reset();
        this->left_eye.store.matrix_cornea_inside.rotate(this->left_eye.cornea_inside_angle);
        this->left_eye.cornea_inside        = this->left_eye.store.cornea_inside[this->get_index()].data[0].transformed(this->left_eye.store.matrix_cornea_inside);
        this->left_eye.draw_cornea_anime2.x = this->left_eye.eyeball_center.x() - (this->left_eye.cornea_inside.width() / 2.0);
        this->left_eye.draw_cornea_anime2.y = this->left_eye.eyeball_center.y() - (this->left_eye.cornea_inside.height() / 2.0);
    }
    /* ============================================= */
    // right
    /* ============================================= */
    if (0 < (int)this->right_eye.store.cornea_inside.size()) {
        this->right_eye.store.matrix_cornea_inside.reset();
        this->right_eye.store.matrix_cornea_inside.rotate(this->right_eye.cornea_inside_angle);
        this->right_eye.cornea_inside        = this->right_eye.store.cornea_inside[this->get_index()].data[0].transformed(this->right_eye.store.matrix_cornea_inside);
        this->right_eye.draw_cornea_anime2.x = this->right_eye.eyeball_center.x() - (this->right_eye.cornea_inside.width() / 2.0);
        this->right_eye.draw_cornea_anime2.y = this->right_eye.eyeball_center.y() - (this->right_eye.cornea_inside.height() / 2.0);
    }
#endif
}

int PartsEyeball::get_index()
{
    switch (this->current_cornea_state) {
        case CorneaState::Receiving:
            return 1;

        case CorneaState::Normal:
        default:
            return 0;
    }
}

void PartsEyeball::set_state_cornea(CorneaState state)
{
    this->request_cornea_state = state;
}

void PartsEyeball::load(StParameter param)
{
    this->_set_image(param);
}

void PartsEyeball::set_param(StParameter param)
{
    // TODO
    this->left_eye.calibration_angle      = param.left_eye.eyeball.angle * (M_PI / 180);
    this->right_eye.calibration_angle     = param.right_eye.eyeball.angle * (M_PI / 180);
    this->left_eye.calibration_angle_cos  = std::abs(std::cos(this->left_eye.calibration_angle));
    this->left_eye.calibration_angle_sin  = std::abs(std::sin(this->left_eye.calibration_angle));
    this->right_eye.calibration_angle_cos = std::abs(std::cos(this->right_eye.calibration_angle));
    this->right_eye.calibration_angle_sin = std::abs(std::sin(this->right_eye.calibration_angle));

    this->right_eye.setting(param.right_eye.eyeball.width, param.right_eye.eyeball.height, param.right_eye.eyeball_center);
    this->left_eye.setting(param.left_eye.eyeball.width, param.left_eye.eyeball.height, param.left_eye.eyeball_center);
#if DEBUG_OUTPUT_WIDGET
    printf("============== Eyeball center ===============\n");
    printf(" Right (x,y) = (%f,%f)\n", param.right_eye.eyeball_center.x, param.right_eye.eyeball_center.y);
    printf(" Left (x,y) = (%f,%f)\n", param.left_eye.eyeball_center.x, param.left_eye.eyeball_center.y);
    printf("=============================================\n");
    printf("------------ Calibration Postion ------------\n");
    printf(" Size (x,y) = (%d,%d)\n", param.left_eye.eyelid.width, param.left_eye.eyelid.height);
    printf(" Size (x,y) = (%d,%d)\n", param.right_eye.eyelid.width, param.right_eye.eyelid.height);
    printf(" Pos r(x,y) = (%f,%f)\n", param.left_eye.eyeball.x, param.left_eye.eyeball.y);
    printf(" Pos l(x,y) = (%f,%f)\n", param.right_eye.eyeball.x, param.right_eye.eyeball.y);
    printf("---------------------------------------------\n");
#endif
    /* ============================================= */
}
void PartsEyeball::_set_image(StParameter param)
{
    if (true == std::filesystem::is_directory(param.path)) {
        char buffer_path[255];
        std::string f_name;
        /* ============================================= */
#if DEBUG_OUTPUT_LOAD_IMAGE
        printf("  ---- LOAD [Vitreous] ----\n");
#endif
        if (0 < param.left_eye.image.eyeball.size()) {
            this->left_eye.store.vitreous.clear();
            for (size_t i = 0; i < param.left_eye.image.eyeball.size(); ++i) {
                StImageMap list;
                list.data.clear();
                list.id = param.left_eye.image.eyeball[i].id;
                for (size_t j = 0; j < param.left_eye.image.eyeball[i].files.size(); ++j) {
                    f_name = param.left_eye.image.eyeball[i].files[j];
                    if (true == std::filesystem::exists(f_name)) {
                        QPixmap buff(f_name.c_str(), nullptr, param.imageFlag);
                        QMatrix matrix_eyeball;
                        if (true == param.left_eye.image.eyeball[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix_eyeball.rotate(param.left_eye.eyeball.angle + param.left_eye.eyelid.angle);
                        buff = buff.scaled(param.left_eye.eyeball.width, param.left_eye.eyeball.height, Qt::IgnoreAspectRatio);

                        buff = buff.transformed(matrix_eyeball);
                        list.data.push_back(buff);
                    }
                }
                list.max = (int)list.data.size();
                if (0 < list.max) {
                    this->left_eye.store.vitreous.push_back(list);
                }
            }
        }
        if (0 < param.right_eye.image.eyeball.size()) {
            this->right_eye.store.vitreous.clear();
            for (size_t i = 0; i < param.right_eye.image.eyeball.size(); ++i) {
                StImageMap list;
                list.data.clear();
                list.id = param.right_eye.image.eyeball[i].id;
                for (size_t j = 0; j < param.right_eye.image.eyeball[i].files.size(); ++j) {
                    f_name = param.right_eye.image.eyeball[i].files[j];
                    if (true == std::filesystem::exists(f_name)) {
                        QPixmap buff(f_name.c_str(), nullptr, param.imageFlag);
                        QMatrix matrix_eyeball;
                        if (true == param.right_eye.image.eyeball[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix_eyeball.rotate(param.right_eye.eyeball.angle + param.right_eye.eyelid.angle);
                        buff = buff.scaled(param.right_eye.eyeball.width, param.right_eye.eyeball.height, Qt::IgnoreAspectRatio);

                        buff = buff.transformed(matrix_eyeball);
                        list.data.push_back(buff);
                    }
                }
                list.max = (int)list.data.size();
                if (0 < list.max) {
                    this->right_eye.store.vitreous.push_back(list);
                }
            }
        }

        /* ============================================= */
#if DRAW_CORNEA_OUTSIDE
#if DEBUG_OUTPUT_LOAD_IMAGE
        printf("  ---- LOAD [Cornea-outside] ----\n");
#endif
        if (0 < param.left_eye.image.cornea_outside.size()) {
            this->left_eye.store.cornea_outside.clear();
            for (size_t i = 0; i < param.left_eye.image.cornea_outside.size(); ++i) {
                StImageMap list;
                list.data.clear();
                list.id = param.left_eye.image.cornea_outside[i].id;
                for (size_t j = 0; j < param.left_eye.image.cornea_outside[i].files.size(); ++j) {
                    f_name = param.left_eye.image.cornea_outside[i].files[j];
                    if (true == std::filesystem::exists(f_name)) {
                        QPixmap buff(f_name.c_str(), nullptr, param.imageFlag);
                        QMatrix matrix_cornea_outside;
                        if (true == param.left_eye.image.cornea_outside[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix_cornea_outside.rotate(param.left_eye.eyeball.angle + param.left_eye.eyelid.angle);
                        buff = buff.scaled((param.left_eye.eyeball.width * param.left_eye.cornea_outside.scale.x),
                                           (param.left_eye.eyeball.height * param.left_eye.cornea_outside.scale.y),
                                           Qt::IgnoreAspectRatio);

                        buff = buff.transformed(matrix_cornea_outside);
                        list.data.push_back(buff);
                    }
                }
                list.max = (int)list.data.size();
                if (0 < list.max) {
                    this->left_eye.store.cornea_outside.push_back(list);
                }
            }
        }
        if (0 < param.right_eye.image.cornea_outside.size()) {
            this->right_eye.store.cornea_outside.clear();
            for (size_t i = 0; i < param.right_eye.image.cornea_outside.size(); ++i) {
                StImageMap list;
                list.data.clear();
                list.id = param.right_eye.image.cornea_outside[i].id;
                for (size_t j = 0; j < param.right_eye.image.cornea_outside[i].files.size(); ++j) {
                    f_name = param.right_eye.image.cornea_outside[i].files[j];
                    if (true == std::filesystem::exists(f_name)) {
                        QPixmap buff(f_name.c_str(), nullptr, param.imageFlag);
                        QMatrix matrix_cornea_outside;
                        if (true == param.right_eye.image.cornea_outside[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix_cornea_outside.rotate(param.right_eye.eyeball.angle + param.right_eye.eyelid.angle);
                        buff = buff.scaled((param.right_eye.eyeball.width * param.right_eye.cornea_outside.scale.x),
                                           (param.right_eye.eyeball.height * param.right_eye.cornea_outside.scale.y),
                                           Qt::IgnoreAspectRatio);

                        buff = buff.transformed(matrix_cornea_outside);
                        list.data.push_back(buff);
                    }
                }
                list.max = (int)list.data.size();
                if (0 < list.max) {
                    this->right_eye.store.cornea_outside.push_back(list);
                }
            }
        }
#endif
#if DRAW_CORNEA_INSIDE
#if DEBUG_OUTPUT_LOAD_IMAGE
        printf("  ---- LOAD [Cornea-inside] ----\n");
#endif
        if (0 < param.left_eye.image.cornea_inside.size()) {
            this->left_eye.store.cornea_inside.clear();
            for (size_t i = 0; i < param.left_eye.image.cornea_inside.size(); ++i) {
                StImageMap list;
                list.data.clear();
                list.id = param.left_eye.image.cornea_inside[i].id;
                for (size_t j = 0; j < param.left_eye.image.cornea_inside[i].files.size(); ++j) {
                    f_name = param.left_eye.image.cornea_inside[i].files[j];
                    if (true == std::filesystem::exists(f_name)) {
                        QPixmap buff(f_name.c_str(), nullptr, param.imageFlag);
                        QMatrix matrix_cornea_inside;
                        if (true == param.left_eye.image.cornea_inside[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix_cornea_inside.rotate(param.left_eye.eyeball.angle + param.left_eye.eyelid.angle);
                        buff = buff.scaled((param.left_eye.eyeball.width * param.left_eye.cornea_inside.scale.x),
                                           (param.left_eye.eyeball.height * param.left_eye.cornea_inside.scale.y),
                                           Qt::IgnoreAspectRatio);

                        buff = buff.transformed(matrix_cornea_inside);
                        list.data.push_back(buff);
                    }
                }
                list.max = (int)list.data.size();
                if (0 < list.max) {
                    this->left_eye.store.cornea_inside.push_back(list);
                }
            }
        }
        if (0 < param.right_eye.image.cornea_inside.size()) {
            this->right_eye.store.cornea_inside.clear();
            for (size_t i = 0; i < param.right_eye.image.cornea_inside.size(); ++i) {
                StImageMap list;
                list.data.clear();
                list.id = param.right_eye.image.cornea_inside[i].id;
                for (size_t j = 0; j < param.right_eye.image.cornea_inside[i].files.size(); ++j) {
                    f_name = param.right_eye.image.cornea_inside[i].files[j];
                    if (true == std::filesystem::exists(f_name)) {
                        QPixmap buff(f_name.c_str(), nullptr, param.imageFlag);
                        QMatrix matrix_cornea_inside;
                        if (true == param.right_eye.image.cornea_inside[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix_cornea_inside.rotate(param.right_eye.eyeball.angle + param.right_eye.eyelid.angle);
                        buff = buff.scaled((param.right_eye.eyeball.width * param.right_eye.cornea_inside.scale.x),
                                           (param.right_eye.eyeball.height * param.right_eye.cornea_inside.scale.y),
                                           Qt::IgnoreAspectRatio);

                        buff = buff.transformed(matrix_cornea_inside);
                        list.data.push_back(buff);
                    }
                }
                list.max = (int)list.data.size();
                if (0 < list.max) {
                    this->right_eye.store.cornea_inside.push_back(list);
                }
            }
        }
#endif
    }
}

/**
 * @brief 描画位置の設定
 *
 * @param send_animation
 */
void PartsEyeball::calc_draw_pixel(int elapsed, int send_animation)
{
    this->current_cornea_state = this->request_cornea_state;
    float buf_dimensions       = dimensions;

    if (request_dimensions < buf_dimensions) {
        buf_dimensions = request_dimensions - EYEBALL_DIMENSIONS_INCREASE;

        if (request_dimensions > buf_dimensions) {
            buf_dimensions = request_dimensions;
        }
    }

    else if (request_dimensions > buf_dimensions) {
        buf_dimensions = request_dimensions + EYEBALL_DIMENSIONS_INCREASE;

        if (request_dimensions < buf_dimensions) {
            buf_dimensions = request_dimensions;
        }
    }

    dimensions = buf_dimensions;
    /**/
    /* ============================================= */
    // left
    /* ============================================= */
    this->left_eye.set_draw_pixel(send_animation, dimensions, this->left_eye.calibration_angle_cos, this->left_eye.calibration_angle_sin);
    left_eye.eyeball_center.setX(this->left_eye.draw_postion.x + (this->left_eye.draw_postion.width / 2.0));
    left_eye.eyeball_center.setY(this->left_eye.draw_postion.y + (this->left_eye.draw_postion.height / 2.0));
    if (0 < (int)this->left_eye.store.vitreous.size()) {
        this->left_eye.eyeball = this->left_eye.store.vitreous[0].data[0];
    }
    /* ============================================= */
    // right
    /* ============================================= */
    this->right_eye.set_draw_pixel(send_animation, dimensions, this->right_eye.calibration_angle_cos, this->right_eye.calibration_angle_sin);
    right_eye.eyeball_center.setX(this->right_eye.draw_postion.x + (this->right_eye.draw_postion.width / 2.0));
    right_eye.eyeball_center.setY(this->right_eye.draw_postion.y + (this->right_eye.draw_postion.height / 2.0));
    if (0 < (int)this->right_eye.store.vitreous.size()) {
        this->right_eye.eyeball = this->right_eye.store.vitreous[0].data[0];
    }
}

/**
 * @brief Construct a new Ctrl Eyeball:: Ctrl Eyeball object
 *
 */
PartsEyeball::PartsEyeball()
{
}

} // namespace maid_robot_system
