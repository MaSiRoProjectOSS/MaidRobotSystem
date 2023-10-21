/**
 * @file parts_eyeball.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "widgets/parts/parts_eyeball.hpp"

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
void PartsEyeball::init()
{
}
void PartsEyeball::closing()
{
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
#if DEBUG_OUTPUT_LOAD_IMAGE
                printf("    left_eye/vitreous\n");
#endif
            }
            if (false == this->left_eye.store.exit_vitreous) {
                this->left_eye.eyeball = this->_blank;
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
#if DEBUG_OUTPUT_LOAD_IMAGE
                    printf("    right_eye/vitreous\n");
#endif
                }
            }
            if (0 < (int)this->right_eye.store.vitreous.size()) {
                this->right_eye.store.exit_vitreous = true;
            }
            if (false == this->right_eye.store.exit_vitreous) {
                this->right_eye.eyeball = this->_blank;
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
                        QMatrix matrix;
                        if (true == param.left_eye.image.cornea_outside[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix.rotate(param.left_eye.eyeball.angle + param.left_eye.eyelid.angle);
                        buff = buff.scaled(param.left_eye.eyeball.width, param.left_eye.eyeball.height, Qt::KeepAspectRatio);

                        buff = buff.transformed(matrix);
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
#if DEBUG_OUTPUT_LOAD_IMAGE
                    printf("    left_eye/cornea_outside\n");
#endif
                }
            }
            if (false == this->left_eye.store.exit_cornea_outside) {
                this->left_eye.cornea_outside = this->_blank;
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
                        QMatrix matrix;
                        if (true == param.right_eye.image.cornea_outside[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix.rotate(param.right_eye.eyeball.angle + param.right_eye.eyelid.angle);
                        buff = buff.scaled(param.right_eye.eyeball.width, param.left_eye.eyeball.height, Qt::KeepAspectRatio);

                        buff = buff.transformed(matrix);
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
#if DEBUG_OUTPUT_LOAD_IMAGE
                    printf("    right_eye/cornea_outside\n");
#endif
                }
            }
            if (false == this->right_eye.store.exit_cornea_outside) {
                this->left_eye.cornea_outside = this->_blank;
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
                        QMatrix matrix;
                        if (true == param.left_eye.image.cornea_inside[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix.rotate(param.left_eye.eyeball.angle + param.left_eye.eyelid.angle);
                        buff = buff.scaled(param.left_eye.eyeball.width, param.left_eye.eyeball.height, Qt::KeepAspectRatio);

                        buff = buff.transformed(matrix);
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
#if DEBUG_OUTPUT_LOAD_IMAGE
                    printf("    left_eye/cornea_inside\n");
#endif
                }
            }
            if (false == this->left_eye.store.exit_cornea_inside) {
                this->left_eye.cornea_inside = this->_blank;
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
                        QMatrix matrix;
                        if (true == param.right_eye.image.cornea_inside[i].mirror) {
                            buff = buff.transformed(QTransform().scale(-1, 1));
                        }

                        matrix.rotate(param.right_eye.eyeball.angle + param.right_eye.eyelid.angle);
                        buff = buff.scaled(param.right_eye.eyeball.width, param.left_eye.eyeball.height, Qt::KeepAspectRatio);

                        buff = buff.transformed(matrix);
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
#if DEBUG_OUTPUT_LOAD_IMAGE
                    printf("    right_eye/cornea_inside\n");
#endif
                }
            }
            if (false == this->right_eye.store.exit_cornea_inside) {
                this->right_eye.cornea_inside = this->_blank;
            }
        }
#endif
    }
}

// =============================
// PUBLIC : Setter
// =============================
void PartsEyeball::set_param(StParameter param, StVector left_eye_center, StVector right_eye_center)
{
    this->left_eye.descend  = param.left_eye.eyeball_descend;
    this->right_eye.descend = param.right_eye.eyeball_descend;
    this->left_eye.setting(param.left_eye, left_eye_center.x + param.left_eye.eyeball.x, left_eye_center.y + param.left_eye.eyeball.y);
    this->right_eye.setting(param.right_eye, right_eye_center.x + param.right_eye.eyeball.x, right_eye_center.y + param.right_eye.eyeball.y);

    this->left_eye.speed_min_cornea_outside = param.left_eye.cornea_outside.speed_min;
    this->left_eye.speed_min_cornea_inside  = param.left_eye.cornea_inside.speed_min;
    this->left_eye.speed_max_cornea_outside = param.left_eye.cornea_outside.speed_max;
    this->left_eye.speed_max_cornea_inside  = param.left_eye.cornea_inside.speed_max;

    this->right_eye.speed_min_cornea_outside = param.right_eye.cornea_outside.speed_min;
    this->right_eye.speed_min_cornea_inside  = param.right_eye.cornea_inside.speed_min;
    this->right_eye.speed_max_cornea_outside = param.right_eye.cornea_outside.speed_max;
    this->right_eye.speed_max_cornea_inside  = param.right_eye.cornea_inside.speed_max;

    this->left_eye.scale_cornea_outside.set(param.left_eye.cornea_outside.scale.x, param.left_eye.cornea_outside.scale.y, 0);
    this->left_eye.scale_cornea_inside.set(param.left_eye.cornea_inside.scale.x, param.left_eye.cornea_inside.scale.y, 0);
    this->right_eye.scale_cornea_outside.set(param.right_eye.cornea_outside.scale.x, param.right_eye.cornea_outside.scale.y, 0);
    this->right_eye.scale_cornea_inside.set(param.right_eye.cornea_inside.scale.x, param.right_eye.cornea_inside.scale.y, 0);

#if DEBUG_OUTPUT_WIDGET
    printf("=============== Eyeball Postion ==============\n");
    printf("  Left\n");
    printf("    Center (x,y) = (%3.1f,%3.1f)\n", this->left_eye.center.x, this->left_eye.center.y);
    printf("    Size   (w,h) = (%3.1f,%3.1f)\n", this->left_eye.size.width, this->left_eye.size.height);
    printf("  Right\n");
    printf("    Center (x,y) = (%3.1f,%3.1f)\n", this->right_eye.center.x, this->right_eye.center.y);
    printf("    Size   (w,h) = (%3.1f,%3.1f)\n", this->right_eye.size.width, this->right_eye.size.height);
#endif
    /* ============================================= */
}

void PartsEyeball::set_dimensions(float dimensions)
{
    if (EYEBALL_DIMENSIONS_MIN > dimensions) {
        this->_request_dimensions = EYEBALL_DIMENSIONS_MIN;
    } else if (EYEBALL_DIMENSIONS_MAX < dimensions) {
        this->_request_dimensions = EYEBALL_DIMENSIONS_MAX;
    } else {
        this->_request_dimensions = dimensions;
    }
}

int PartsEyeball::set_state_cornea(int id)
{
    this->_request_cornea_id = id;
    return this->_request_cornea_id;
}
int PartsEyeball::set_state_eyeball(int id)
{
    this->_request_eyeball_id = id;
    return this->_request_eyeball_id;
}

void PartsEyeball::set_default()
{
    this->set_dimensions(EYEBALL_DIMENSIONS_DEFAULT);
}

} // namespace maid_robot_system
