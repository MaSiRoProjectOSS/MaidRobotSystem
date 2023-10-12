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
    right.target.set(0, 0);
    left.target.set(0, 0);
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
#if DRAW_PUPIL_OUTSIDE
    pupil_outside = pupil_outside_origin[this->get_index()];
    matrix_pupil_outside.reset();
    matrix_pupil_outside.scale((right.draw_postion.width - 20) / right.size.x, (right.draw_postion.height + 60) / right.size.y);
    matrix_pupil_outside.rotate(right.pupil_outside_angle);
    pupil_outside = pupil_outside.transformed(matrix_pupil_outside);
    /* ============================================= */
    // pupil outside postion
    right.draw_pupil_anime.x = eyeball_center_right.x() - (pupil_outside.width() / 2.0);
    right.draw_pupil_anime.y = eyeball_center_right.y() - (pupil_outside.height() / 2.0);
    left.draw_pupil_anime.x  = eyeball_center_left.x() - (pupil_outside.width() / 2.0);
    left.draw_pupil_anime.y  = eyeball_center_left.y() - (pupil_outside.height() / 2.0);
#endif
}

/**
 * @brief 内周リングの描画
 *
 */
void PartsEyeball::draw_inside()
{
#if DRAW_PUPIL_INSIDE
    pupil_inside = pupil_inside_origin[this->get_index()];
    matrix_pupil_inside.reset();
    matrix_pupil_inside.rotate(right.pupil_inside_angle);
    pupil_inside = pupil_inside.transformed(matrix_pupil_inside);
    /* ============================================= */
    // pupil inside postion
    right.draw_pupil_anime2.x = eyeball_center_right.x() - (pupil_inside.width() / 2.0);
    right.draw_pupil_anime2.y = eyeball_center_right.y() - (pupil_inside.height() / 2.0);
    left.draw_pupil_anime2.x  = eyeball_center_left.x() - (pupil_inside.width() / 2.0);
    left.draw_pupil_anime2.y  = eyeball_center_left.y() - (pupil_inside.height() / 2.0);
#endif
}

int PartsEyeball::get_index()
{
    switch (this->current_pupil_state) {
        case PupilState::Receiving:
            return 1;

        case PupilState::Normal:
        default:
            return 0;
    }
}

void PartsEyeball::set_state_pupil(PupilState state)
{
    this->request_pupil_state = state;
}

void PartsEyeball::set_param(StParameter param)
{
    this->_set_image(param);
    // TODO
    right.setting(param.eyeball_size_x, param.eyeball_size_y, param.eyeball_center_right);
    left.setting(param.eyeball_size_x, param.eyeball_size_y, param.eyeball_center_left);
#if DEBUG_OUTPUT_WIDGET
    printf("============== Eyeball center ===============\n");
    printf(" Right (x,y) = (%f,%f)\n", param.eyeball_center_right.x, param.eyeball_center_right.y);
    printf(" Left (x,y) = (%f,%f)\n", param.eyeball_center_left.x, param.eyeball_center_left.y);
    printf("=============================================\n");
    printf("------------ Calibration Postion ------------\n");
    printf(" Size (x,y) = (%d,%d)\n", param.eyelid_size_x, param.eyelid_size_y);
    printf(" Pos r(x,y) = (%f,%f)\n", param.eyeball_position_l_x, param.eyeball_position_l_y);
    printf(" Pos l(x,y) = (%f,%f)\n", param.eyeball_position_r_x, param.eyeball_position_r_y);
    printf("---------------------------------------------\n");
#endif
    /* ============================================= */
}
void PartsEyeball::_set_image(StParameter param)
{
    if (true == std::filesystem::is_directory(param.path)) {
        char buffer_path[255];
        printf("  ---- LOAD [Eyeball] ----\n");
        ////////////////
        // eyeball_origin left
        sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/eyeball/eyeball_normally.png");
        if (true == std::filesystem::exists(buffer_path)) {
            QMatrix matrix_eyeball_l;
            matrix_eyeball_l.rotate(-param.l_angle - param.eyeball_angle);
            this->eyeball_origin_l = QPixmap(buffer_path, nullptr, param.imageFlag);
            this->eyeball_origin_l = this->eyeball_origin_l.scaled(param.eyeball_size_x, param.eyeball_size_y, Qt::IgnoreAspectRatio);
            this->eyeball_origin_l = this->eyeball_origin_l.transformed(matrix_eyeball_l);
        }
        // eyeball_origin right
        sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/eyeball/eyeball_normally.png");
        if (true == std::filesystem::exists(buffer_path)) {
            QMatrix matrix_eyeball_r;
            matrix_eyeball_r.rotate(-param.r_angle + param.eyeball_angle);
            this->eyeball_origin_r = QPixmap(buffer_path, nullptr, param.imageFlag);
            this->eyeball_origin_r = this->eyeball_origin_r.scaled(param.eyeball_size_x, param.eyeball_size_y, Qt::IgnoreAspectRatio);
            this->eyeball_origin_r = this->eyeball_origin_r.transformed(matrix_eyeball_r);
        }

#if DRAW_PUPIL_OUTSIDE
        printf("  ---- LOAD [Pupil-outside] ----\n");
        // pupil_outside_origin
        this->matrix_pupil_outside.rotate(0);

        for (int i = 0; i < 2; i++) {
            switch (i) {
                case 1:
                    sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/pupil/pupil_order.png");
                    if (true == std::filesystem::exists(buffer_path)) {
                        this->pupil_outside_origin[i] = QPixmap(buffer_path, nullptr, param.imageFlag);
                    }
                    break;

                default:
                    sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/pupil/pupil_normally.png");
                    if (true == std::filesystem::exists(buffer_path)) {
                        this->pupil_outside_origin[i] = QPixmap(buffer_path, nullptr, param.imageFlag);
                    }
                    break;
            }

            this->pupil_outside_origin[i] = this->pupil_outside_origin[i].scaled(this->pupil_ling_size_outside, this->pupil_ling_size_outside, Qt::IgnoreAspectRatio);
            this->pupil_outside_origin[i] = this->pupil_outside_origin[i].transformed(this->matrix_pupil_outside);
        }

#endif
#if DRAW_PUPIL_INSIDE
        printf("  ---- LOAD [Pupil-inside] ----\n");
        // pupil_inside_origin
        this->matrix_pupil_inside.rotate(0);

        for (int i = 0; i < 2; i++) {
            switch (i) {
                case 1:
                    sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/pupil/pupil_order.png");
                    if (true == std::filesystem::exists(buffer_path)) {
                        this->pupil_inside_origin[i] = QPixmap(buffer_path, nullptr, param.imageFlag);
                    }
                    break;

                default:
                    sprintf(buffer_path, "%s/%s", param.path.c_str(), "eye/pupil/pupil_normally.png");
                    if (true == std::filesystem::exists(buffer_path)) {
                        this->pupil_inside_origin[i] = QPixmap(buffer_path, nullptr, param.imageFlag);
                    }
                    break;
            }

            this->pupil_inside_origin[i] = this->pupil_inside_origin[i].scaled(this->pupil_ling_size_inside, this->pupil_ling_size_inside, Qt::IgnoreAspectRatio);
            this->pupil_inside_origin[i] = this->pupil_inside_origin[i].transformed(this->matrix_pupil_inside);
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
    this->current_pupil_state = this->request_pupil_state;
    float buf_dimensions      = dimensions;

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
    left.set_draw_pixel(send_animation, dimensions, this->calibration_l_angle_cos, this->calibration_l_angle_sin);
    right.set_draw_pixel(send_animation, dimensions, this->calibration_r_angle_cos, this->calibration_r_angle_sin);
    eyeball_center_right.setX(right.draw_postion.x + (right.draw_postion.width / 2.0));
    eyeball_center_right.setY(right.draw_postion.y + (right.draw_postion.height / 2.0));
    eyeball_center_left.setX(left.draw_postion.x + (left.draw_postion.width / 2.0));
    eyeball_center_left.setY(left.draw_postion.y + (left.draw_postion.height / 2.0));
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
void PartsEyeball::Initialize(double param_calibration_l_angle, double param_calibration_r_angle)
{
    this->calibration_l_angle     = param_calibration_l_angle * (M_PI / 180);
    this->calibration_r_angle     = param_calibration_r_angle * (M_PI / 180);
    this->calibration_l_angle_cos = std::abs(std::cos(this->calibration_l_angle));
    this->calibration_l_angle_sin = std::abs(std::sin(this->calibration_l_angle));
    this->calibration_r_angle_cos = std::abs(std::cos(this->calibration_r_angle));
    this->calibration_r_angle_sin = std::abs(std::sin(this->calibration_r_angle));
}

} // namespace maid_robot_system
