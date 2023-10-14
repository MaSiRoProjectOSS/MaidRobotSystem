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
    set_dimensions(EYEBALL_DIMENSIONS_DEFAULT);
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
    this->matrix_cornea_outside_left.reset();
    this->matrix_cornea_outside_left.scale((this->right_eye.draw_postion.width - 20) / this->right_eye.size.x, (this->right_eye.draw_postion.height + 60) / this->right_eye.size.y);
    this->matrix_cornea_outside_left.rotate(-this->right_eye.cornea_outside_angle);
    this->cornea_outside_left          = this->cornea_outside_origin[this->get_index()].transformed(this->matrix_cornea_outside_left);
    this->left_eye.draw_cornea_anime.x = this->eyeball_center_left.x() - (this->cornea_outside_left.width() / 2.0);
    this->left_eye.draw_cornea_anime.y = this->eyeball_center_left.y() - (this->cornea_outside_left.height() / 2.0);
    /* ============================================= */
    // right
    /* ============================================= */
    this->matrix_cornea_outside_right.reset();
    this->matrix_cornea_outside_right.scale((this->right_eye.draw_postion.width - 20) / this->right_eye.size.x,
                                            (this->right_eye.draw_postion.height + 60) / this->right_eye.size.y);
    this->matrix_cornea_outside_right.rotate(-this->right_eye.cornea_outside_angle);
    this->cornea_outside_right          = this->cornea_outside_origin[this->get_index()].transformed(this->matrix_cornea_outside_right);
    this->right_eye.draw_cornea_anime.x = this->eyeball_center_right.x() - (this->cornea_outside_right.width() / 2.0);
    this->right_eye.draw_cornea_anime.y = this->eyeball_center_right.y() - (this->cornea_outside_right.height() / 2.0);
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
    this->matrix_cornea_inside_left.reset();
    this->matrix_cornea_inside_left.rotate(-this->right_eye.cornea_inside_angle);
    this->cornea_outside_left           = this->cornea_inside_origin[this->get_index()].transformed(this->matrix_cornea_inside_left);
    this->left_eye.draw_cornea_anime2.x = this->eyeball_center_left.x() - (this->cornea_outside_left.width() / 2.0);
    this->left_eye.draw_cornea_anime2.y = this->eyeball_center_left.y() - (this->cornea_outside_left.height() / 2.0);
    /* ============================================= */
    // right
    /* ============================================= */
    this->matrix_cornea_inside_right.reset();
    this->matrix_cornea_inside_right.rotate(-this->right_eye.cornea_inside_angle);
    this->cornea_inside_right            = this->cornea_inside_origin[this->get_index()].transformed(this->matrix_cornea_inside_right);
    this->right_eye.draw_cornea_anime2.x = this->eyeball_center_right.x() - (this->cornea_inside_right.width() / 2.0);
    this->right_eye.draw_cornea_anime2.y = this->eyeball_center_right.y() - (this->cornea_inside_right.height() / 2.0);
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
#if DEBUG_OUTPUT_LOAD_IMAGE
        printf("  ---- LOAD [Eyeball] ----\n");
#endif
        ////////////////
        // eyeball_origin left
        sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/eyeball/eyeball_normally.png");
        if (true == std::filesystem::exists(buffer_path)) {
            QMatrix matrix_eyeball_l;
            matrix_eyeball_l.rotate(param.left_eye.eyeball.angle + param.left_eye.eyelid.angle);
            this->eyeball_origin_l = QPixmap(buffer_path, nullptr, param.imageFlag);
            this->eyeball_origin_l = this->eyeball_origin_l.scaled(param.left_eye.eyeball.width, param.left_eye.eyeball.height, Qt::IgnoreAspectRatio);
            this->eyeball_origin_l = this->eyeball_origin_l.transformed(matrix_eyeball_l);
        }
        // eyeball_origin right
        sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/eyeball/eyeball_normally.png");
        if (true == std::filesystem::exists(buffer_path)) {
            QMatrix matrix_eyeball_r;
            matrix_eyeball_r.rotate(param.right_eye.eyeball.angle + param.right_eye.eyelid.angle);
            this->eyeball_origin_r = QPixmap(buffer_path, nullptr, param.imageFlag);
            this->eyeball_origin_r = this->eyeball_origin_r.scaled(param.right_eye.eyeball.width, param.right_eye.eyeball.height, Qt::IgnoreAspectRatio);
            this->eyeball_origin_r = this->eyeball_origin_r.transformed(matrix_eyeball_r);
        }

#if DRAW_CORNEA_OUTSIDE
#if DEBUG_OUTPUT_LOAD_IMAGE
        printf("  ---- LOAD [Cornea-outside] ----\n");
#endif
        QMatrix matrix_cornea_outside;
        matrix_cornea_outside.rotate(0);

        for (int i = 0; i < 2; i++) {
            switch (i) {
                case 1:
                    sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/cornea/cornea_order.png");
                    if (true == std::filesystem::exists(buffer_path)) {
                        this->cornea_outside_origin[i] = QPixmap(buffer_path, nullptr, param.imageFlag);
                    }
                    break;

                default:
                    sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/cornea/cornea_normally.png");
                    if (true == std::filesystem::exists(buffer_path)) {
                        this->cornea_outside_origin[i] = QPixmap(buffer_path, nullptr, param.imageFlag);
                    }
                    break;
            }

            this->cornea_outside_origin[i] = this->cornea_outside_origin[i].scaled(param.right_eye.eyeball.width, param.right_eye.eyeball.height, Qt::IgnoreAspectRatio);
            this->cornea_outside_origin[i] = this->cornea_outside_origin[i].transformed(matrix_cornea_outside);
        }

#endif
#if DRAW_CORNEA_INSIDE
#if DEBUG_OUTPUT_LOAD_IMAGE
        printf("  ---- LOAD [Cornea-inside] ----\n");
#endif
        QMatrix matrix_cornea_inside;
        matrix_cornea_inside.rotate(0);

        for (int i = 0; i < 2; i++) {
            switch (i) {
                case 1:
                    sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/cornea/cornea_order.png");
                    if (true == std::filesystem::exists(buffer_path)) {
                        this->cornea_inside_origin[i] = QPixmap(buffer_path, nullptr, param.imageFlag);
                    }
                    break;

                default:
                    sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/cornea/cornea_normally.png");
                    if (true == std::filesystem::exists(buffer_path)) {
                        this->cornea_inside_origin[i] = QPixmap(buffer_path, nullptr, param.imageFlag);
                    }
                    break;
            }

            this->cornea_inside_origin[i] = this->cornea_inside_origin[i].scaled(param.right_eye.eyeball.width, param.right_eye.eyeball.height, Qt::IgnoreAspectRatio);
            this->cornea_inside_origin[i] = this->cornea_inside_origin[i].transformed(matrix_cornea_inside);
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
    this->left_eye.set_draw_pixel(send_animation, dimensions, this->calibration_l_angle_cos, this->calibration_l_angle_sin);
    this->right_eye.set_draw_pixel(send_animation, dimensions, this->calibration_r_angle_cos, this->calibration_r_angle_sin);
    eyeball_center_right.setX(this->right_eye.draw_postion.x + (this->right_eye.draw_postion.width / 2.0));
    eyeball_center_right.setY(this->right_eye.draw_postion.y + (this->right_eye.draw_postion.height / 2.0));
    eyeball_center_left.setX(this->left_eye.draw_postion.x + (this->left_eye.draw_postion.width / 2.0));
    eyeball_center_left.setY(this->left_eye.draw_postion.y + (this->left_eye.draw_postion.height / 2.0));
}

/**
 * @brief Construct a new Ctrl Eyeball:: Ctrl Eyeball object
 *
 */
PartsEyeball::PartsEyeball()
{
}

/**
 * @brief 初期化
 *
 */
void PartsEyeball::initialize(double param_calibration_l_angle, double param_calibration_r_angle)
{
    this->calibration_l_angle     = param_calibration_l_angle * (M_PI / 180);
    this->calibration_r_angle     = param_calibration_r_angle * (M_PI / 180);
    this->calibration_l_angle_cos = std::abs(std::cos(this->calibration_l_angle));
    this->calibration_l_angle_sin = std::abs(std::sin(this->calibration_l_angle));
    this->calibration_r_angle_cos = std::abs(std::cos(this->calibration_r_angle));
    this->calibration_r_angle_sin = std::abs(std::sin(this->calibration_r_angle));
}

} // namespace maid_robot_system
