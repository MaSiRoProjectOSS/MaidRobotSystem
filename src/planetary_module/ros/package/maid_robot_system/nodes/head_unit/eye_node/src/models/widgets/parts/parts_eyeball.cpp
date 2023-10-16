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

// =============================
// Constructor
// =============================
PartsEyeball::PartsEyeball()
{
}
PartsEyeball::~PartsEyeball()
{
}

// =============================
// PUBLIC : Function
// =============================
void PartsEyeball::calculate(int elapsed, int send_animation)
{
    ENUM_STATE state     = this->_request_cornea_state;
    float buf_dimensions = this->_dimensions;

    if (this->_request_dimensions < buf_dimensions) {
        buf_dimensions = this->_request_dimensions - EYEBALL_DIMENSIONS_INCREASE;

        if (this->_request_dimensions > buf_dimensions) {
            buf_dimensions = this->_request_dimensions;
        }
    } else if (this->_request_dimensions > buf_dimensions) {
        buf_dimensions = this->_request_dimensions + EYEBALL_DIMENSIONS_INCREASE;

        if (this->_request_dimensions < buf_dimensions) {
            buf_dimensions = this->_request_dimensions;
        }
    }

    this->_dimensions = buf_dimensions;

    // left
    this->left_eye.set_draw_pixel(send_animation, this->_dimensions, this->left_eye.calibration_angle_cos, this->left_eye.calibration_angle_sin);
    left_eye.eyeball_center.setX(this->left_eye.draw_postion.x + (this->left_eye.draw_postion.width / 2.0));
    left_eye.eyeball_center.setY(this->left_eye.draw_postion.y + (this->left_eye.draw_postion.height / 2.0));
    // right
    this->right_eye.set_draw_pixel(send_animation, this->_dimensions, this->right_eye.calibration_angle_cos, this->right_eye.calibration_angle_sin);
    right_eye.eyeball_center.setX(this->right_eye.draw_postion.x + (this->right_eye.draw_postion.width / 2.0));
    right_eye.eyeball_center.setY(this->right_eye.draw_postion.y + (this->right_eye.draw_postion.height / 2.0));

    /* ============================================= */
    // drawing
    /* ============================================= */
    this->_drawing(state);
}

void PartsEyeball::load(StParameter param)
{
    if (true == std::filesystem::is_directory(param.path)) {
        char buffer_path[255];
        std::string f_name;
        /* ============================================= */
#if DEBUG_OUTPUT_LOAD_IMAGE
        printf("  ---- LOAD [Vitreous] ----\n");
#endif
        if (0 < param.left_eye.image.eyeball.size()) {
            this->left_eye.store.exit_vitreous = false;
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
            if (0 < (int)this->left_eye.store.vitreous.size()) {
                this->left_eye.store.exit_vitreous = true;
            }
        }
        if (0 < param.right_eye.image.eyeball.size()) {
            this->right_eye.store.exit_vitreous = false;
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
            if (0 < (int)this->right_eye.store.vitreous.size()) {
                this->right_eye.store.exit_vitreous = true;
            }
        }

        /* ============================================= */
#if DRAW_CORNEA_OUTSIDE
#if DEBUG_OUTPUT_LOAD_IMAGE
        printf("  ---- LOAD [Cornea-outside] ----\n");
#endif
        if (0 < param.left_eye.image.cornea_outside.size()) {
            this->left_eye.store.exit_cornea_outside = false;
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
            if (true == param.left_eye.cornea_outside.enable) {
                if (0 < (int)this->left_eye.store.cornea_outside.size()) {
                    this->left_eye.store.exit_cornea_outside = true;
                }
            }
        }
        if (0 < param.right_eye.image.cornea_outside.size()) {
            this->right_eye.store.exit_cornea_outside = false;
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
            if (true == param.right_eye.cornea_outside.enable) {
                if (0 < (int)this->right_eye.store.cornea_outside.size()) {
                    this->right_eye.store.exit_cornea_outside = true;
                }
            }
        }
#endif
#if DRAW_CORNEA_INSIDE
#if DEBUG_OUTPUT_LOAD_IMAGE
        printf("  ---- LOAD [Cornea-inside] ----\n");
#endif
        if (0 < param.left_eye.image.cornea_inside.size()) {
            this->left_eye.store.exit_cornea_inside = false;
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
            if (true == param.left_eye.cornea_inside.enable) {
                if (0 < (int)this->left_eye.store.cornea_inside.size()) {
                    this->left_eye.store.exit_cornea_inside = true;
                }
            }
        }
        if (0 < param.right_eye.image.cornea_inside.size()) {
            this->right_eye.store.exit_cornea_inside = false;
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
            if (true == param.right_eye.cornea_inside.enable) {
                if (0 < (int)this->right_eye.store.cornea_inside.size()) {
                    this->right_eye.store.exit_cornea_inside = true;
                }
            }
        }
#endif
    }
}

// =============================
// PUBLIC : Setter
// =============================
void PartsEyeball::set_param(StParameter param)
{
    // TODO
    this->left_eye.color.setRgb(param.ciliary_color.r, param.ciliary_color.g, param.ciliary_color.b);
    this->right_eye.color.setRgb(param.ciliary_color.r, param.ciliary_color.g, param.ciliary_color.b);

    this->left_eye.rect_ciliary.setWidth(param.left_eye.eyelid.width);
    this->left_eye.rect_ciliary.setHeight(param.left_eye.eyelid.height);
    this->right_eye.rect_ciliary.setWidth(param.right_eye.eyelid.width);
    this->right_eye.rect_ciliary.setHeight(param.right_eye.eyelid.height);

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
    printf(" Right (x,y) = (%6.1f,%6.1f)\n", param.right_eye.eyeball_center.x, param.right_eye.eyeball_center.y);
    printf(" Left  (x,y) = (%6.1f,%6.1f)\n", param.left_eye.eyeball_center.x, param.left_eye.eyeball_center.y);

    printf("------------ Calibration Postion ------------\n");
    printf(" Size (x,y) = (%d,%d)\n", param.left_eye.eyelid.width, param.left_eye.eyelid.height);
    printf(" Size (x,y) = (%d,%d)\n", param.right_eye.eyelid.width, param.right_eye.eyelid.height);
    printf(" Pos r(x,y) = (%d,%d)\n", param.left_eye.eyeball.x, param.left_eye.eyeball.y);
    printf(" Pos l(x,y) = (%d,%d)\n", param.right_eye.eyeball.x, param.right_eye.eyeball.y);
#endif
    /* ============================================= */
}

void PartsEyeball::set_dimensions(float value)
{
    if (EYEBALL_DIMENSIONS_MIN > value) {
        this->_request_dimensions = EYEBALL_DIMENSIONS_MIN;
    } else if (EYEBALL_DIMENSIONS_MAX < value) {
        this->_request_dimensions = EYEBALL_DIMENSIONS_MAX;
    } else {
        this->_request_dimensions = value;
    }
}

void PartsEyeball::set_state_cornea(CorneaState state)
{
    this->_request_cornea_state = state;
}

void PartsEyeball::set_default()
{
    this->right_eye.target.set(0, 0, 0);
    this->left_eye.target.set(0, 0, 0);
    this->set_dimensions(EYEBALL_DIMENSIONS_DEFAULT);
}

// =============================
// PRIVATE : Function
// =============================
void PartsEyeball::_drawing(ENUM_STATE state)
{
    /* ============================================= */
    // left
    /* ============================================= */
    if (true == this->left_eye.store.exit_vitreous) {
        this->left_eye.eyeball = this->left_eye.store.vitreous[0].data[0];
    }
    /* ============================================= */
    // right
    /* ============================================= */
    if (true == this->right_eye.store.exit_vitreous) {
        this->right_eye.eyeball = this->right_eye.store.vitreous[0].data[0];
    }

#if DRAW_CORNEA_OUTSIDE
    /* ============================================= */
    // left
    /* ============================================= */
    if (true == this->left_eye.store.exit_cornea_outside) {
        this->left_eye.store.matrix_cornea_outside.reset();
        this->left_eye.store.matrix_cornea_outside.scale((this->left_eye.draw_postion.width - 20) / this->left_eye.size.x, //
                                                         (this->left_eye.draw_postion.height + 60) / this->left_eye.size.y);
        this->left_eye.store.matrix_cornea_outside.rotate(this->left_eye.cornea_outside_angle);
        this->left_eye.cornea_outside      = this->left_eye.store.cornea_outside[this->_get_index(state)].data[0].transformed(this->left_eye.store.matrix_cornea_outside);
        this->left_eye.draw_cornea_anime.x = this->left_eye.eyeball_center.x() - (this->left_eye.cornea_outside.width() / 2.0);
        this->left_eye.draw_cornea_anime.y = this->left_eye.eyeball_center.y() - (this->left_eye.cornea_outside.height() / 2.0);
    }
    /* ============================================= */
    // right
    /* ============================================= */
    if (true == this->right_eye.store.exit_cornea_outside) {
        this->right_eye.store.matrix_cornea_outside.reset();
        this->right_eye.store.matrix_cornea_outside.scale((this->right_eye.draw_postion.width - 20) / this->right_eye.size.x, //
                                                          (this->right_eye.draw_postion.height + 60) / this->right_eye.size.y);
        this->right_eye.store.matrix_cornea_outside.rotate(this->right_eye.cornea_outside_angle);
        this->right_eye.cornea_outside      = this->right_eye.store.cornea_outside[this->_get_index(state)].data[0].transformed(this->right_eye.store.matrix_cornea_outside);
        this->right_eye.draw_cornea_anime.x = this->right_eye.eyeball_center.x() - (this->right_eye.cornea_outside.width() / 2.0);
        this->right_eye.draw_cornea_anime.y = this->right_eye.eyeball_center.y() - (this->right_eye.cornea_outside.height() / 2.0);
    }
#endif

#if DRAW_CORNEA_INSIDE
    /* ============================================= */
    // left
    /* ============================================= */
    if (true == this->left_eye.store.exit_cornea_inside) {
        this->left_eye.store.matrix_cornea_inside.reset();
        this->left_eye.store.matrix_cornea_inside.rotate(this->left_eye.cornea_inside_angle);
        this->left_eye.cornea_inside        = this->left_eye.store.cornea_inside[this->_get_index(state)].data[0].transformed(this->left_eye.store.matrix_cornea_inside);
        this->left_eye.draw_cornea_anime2.x = this->left_eye.eyeball_center.x() - (this->left_eye.cornea_inside.width() / 2.0);
        this->left_eye.draw_cornea_anime2.y = this->left_eye.eyeball_center.y() - (this->left_eye.cornea_inside.height() / 2.0);
    }
    /* ============================================= */
    // right
    /* ============================================= */
    if (true == this->right_eye.store.exit_cornea_inside) {
        this->right_eye.store.matrix_cornea_inside.reset();
        this->right_eye.store.matrix_cornea_inside.rotate(this->right_eye.cornea_inside_angle);
        this->right_eye.cornea_inside        = this->right_eye.store.cornea_inside[this->_get_index(state)].data[0].transformed(this->right_eye.store.matrix_cornea_inside);
        this->right_eye.draw_cornea_anime2.x = this->right_eye.eyeball_center.x() - (this->right_eye.cornea_inside.width() / 2.0);
        this->right_eye.draw_cornea_anime2.y = this->right_eye.eyeball_center.y() - (this->right_eye.cornea_inside.height() / 2.0);
    }
#endif
}

int PartsEyeball::_get_index(ENUM_STATE state)
{
    int result = 0;
    switch (state) {
        case ENUM_STATE::Receiving:
            result = 1;
            break;
        case ENUM_STATE::Normal:
        default:
            result = 0;
            break;
    }
    return result;
}

} // namespace maid_robot_system
