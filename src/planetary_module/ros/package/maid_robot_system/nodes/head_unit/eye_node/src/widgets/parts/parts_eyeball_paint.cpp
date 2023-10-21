/**
 * @file parts_eyeball.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "widgets/parts/parts_eyeball.hpp"

#include <QRectF>

namespace maid_robot_system
{
// =============================
// PUBLIC : Function
// =============================
void PartsEyeball::update_background(QPainter &painter)
{
}
void PartsEyeball::update_eyeball(QPainter &painter)
{
    if (true == this->right_eye.store.exit_vitreous) {
        painter.drawPixmap(this->right_eye.draw_postion.x,
                           this->right_eye.draw_postion.y,
                           this->right_eye.draw_postion.width,
                           this->right_eye.draw_postion.height,
                           this->right_eye.eyeball);
    }
    if (true == this->left_eye.store.exit_vitreous) {
        painter.drawPixmap(this->left_eye.draw_postion.x,
                           this->left_eye.draw_postion.y,
                           this->left_eye.draw_postion.width,
                           this->left_eye.draw_postion.height,
                           this->left_eye.eyeball);
    }
}
void PartsEyeball::update_cornea_outside(QPainter &painter)
{
    if (true == this->right_eye.store.exit_cornea_outside) {
        painter.drawPixmap(this->right_eye.target_cornea_outside, this->right_eye.cornea_outside, this->right_eye.source_cornea_outside);
    }
    if (true == this->left_eye.store.exit_cornea_outside) {
        painter.drawPixmap(this->left_eye.target_cornea_outside, this->left_eye.cornea_outside, this->left_eye.source_cornea_outside);
    }
}
void PartsEyeball::update_cornea_inside(QPainter &painter)
{
    if (true == this->right_eye.store.exit_cornea_inside) {
        painter.drawPixmap(this->right_eye.target_cornea_inside, this->right_eye.cornea_inside, this->right_eye.source_cornea_inside);
    }
    if (true == this->left_eye.store.exit_cornea_inside) {
        painter.drawPixmap(this->left_eye.target_cornea_inside, this->left_eye.cornea_inside, this->left_eye.source_cornea_inside);
    }
}

void PartsEyeball::calculate(int progress, int elapsed)
{
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
    this->left_eye.set_draw_pixel(progress, this->_dimensions, this->left_eye.calibration_angle_cos, this->left_eye.calibration_angle_sin);
    left_eye.eyeball_center.x = (this->left_eye.draw_postion.x + (this->left_eye.draw_postion.width / 2.0));
    left_eye.eyeball_center.y = (this->left_eye.draw_postion.y + (this->left_eye.draw_postion.height / 2.0));
    // right
    this->right_eye.set_draw_pixel(progress, this->_dimensions, this->right_eye.calibration_angle_cos, this->right_eye.calibration_angle_sin);
    right_eye.eyeball_center.x = (this->right_eye.draw_postion.x + (this->right_eye.draw_postion.width / 2.0));
    right_eye.eyeball_center.y = (this->right_eye.draw_postion.y + (this->right_eye.draw_postion.height / 2.0));

    /* ============================================= */
    // drawing
    /* ============================================= */

    static int previous_eyeball = -1;
    static int previous_cornea  = -1;

    if (previous_eyeball != this->_request_eyeball_id) {
        if (0 > this->_request_eyeball_id) {
            this->_request_eyeball_id = std::max(0, previous_eyeball);
        } else {
            previous_eyeball = this->_request_eyeball_id;
        }
        if (true == this->left_eye.store.exit_vitreous) {
            this->left_eye.eyeball = this->left_eye.store.vitreous[0].data[0];
#if DEBUG_OUTPUT_LOAD_IMAGE
            printf("<reload> left_eye/eyeball\n");
#endif
        }
        if (true == this->right_eye.store.exit_vitreous) {
            this->right_eye.eyeball = this->right_eye.store.vitreous[0].data[0];
#if DEBUG_OUTPUT_LOAD_IMAGE
            printf("<reload> right_eye/eyeball\n");
#endif
        }
    }
    if (previous_cornea != this->_request_cornea_id) {
        if (0 > this->_request_cornea_id) {
            this->_request_cornea_id = std::max(0, previous_cornea);
        } else {
            previous_cornea = this->_request_cornea_id;
        }
#if DRAW_CORNEA_OUTSIDE
        if (true == this->left_eye.store.exit_cornea_outside) {
            for (size_t i = 0; i < this->left_eye.store.cornea_outside.size(); ++i) {
                if (previous_cornea == this->left_eye.store.cornea_outside[i].id) {
                    this->left_eye.store.buf_cornea_outside = this->left_eye.store.cornea_outside[i].data[0];

                    this->left_eye.size_cornea_outside.set(((this->left_eye.store.buf_cornea_outside.width() * this->left_eye.scale_cornea_outside.x) / 2.0), //
                                                           ((this->left_eye.store.buf_cornea_outside.height() * this->left_eye.scale_cornea_outside.y) / 2.0),
                                                           this->left_eye.store.buf_cornea_outside.width() * this->left_eye.scale_cornea_outside.x,
                                                           this->left_eye.store.buf_cornea_outside.height() * this->left_eye.scale_cornea_outside.y);
                    this->left_eye.store.rect_cornea_outside.set(this->left_eye.store.buf_cornea_outside.width() / 2.0, //
                                                                 this->left_eye.store.buf_cornea_outside.height() / 2.0,
                                                                 this->left_eye.store.buf_cornea_outside.width(),
                                                                 this->left_eye.store.buf_cornea_outside.height());

#if DEBUG_OUTPUT_LOAD_IMAGE
                    printf("<reload> left_eye/cornea_outside\n");
#endif
                    break;
                }
            }
        }
        if (true == this->right_eye.store.exit_cornea_outside) {
            this->right_eye.cornea_outside = this->right_eye.store.cornea_outside[0].data[0];
            for (size_t i = 0; i < this->right_eye.store.cornea_outside.size(); ++i) {
                if (previous_cornea == this->right_eye.store.cornea_outside[i].id) {
                    this->right_eye.store.buf_cornea_outside = this->right_eye.store.cornea_outside[i].data[0];

                    this->right_eye.size_cornea_outside.set(((this->right_eye.store.buf_cornea_outside.width() * this->right_eye.scale_cornea_outside.x) / 2.0), //
                                                            ((this->right_eye.store.buf_cornea_outside.height() * this->right_eye.scale_cornea_outside.y) / 2.0),
                                                            this->right_eye.store.buf_cornea_outside.width() * this->right_eye.scale_cornea_outside.x,
                                                            this->right_eye.store.buf_cornea_outside.height() * this->right_eye.scale_cornea_outside.y);
                    this->right_eye.store.rect_cornea_outside.set(this->right_eye.store.buf_cornea_outside.width() / 2.0, //
                                                                  this->right_eye.store.buf_cornea_outside.height() / 2.0,
                                                                  this->right_eye.store.buf_cornea_outside.width(),
                                                                  this->right_eye.store.buf_cornea_outside.height());

#if DEBUG_OUTPUT_LOAD_IMAGE
                    printf("<reload> right_eye/cornea_outside\n");
#endif
                    break;
                }
            }
        }
#endif

#if DRAW_CORNEA_INSIDE
        if (true == this->left_eye.store.exit_cornea_inside) {
            for (size_t i = 0; i < this->left_eye.store.cornea_inside.size(); ++i) {
                if (previous_cornea == this->left_eye.store.cornea_inside[i].id) {
                    this->left_eye.store.buf_cornea_inside = this->left_eye.store.cornea_inside[i].data[0];
                    this->left_eye.size_cornea_inside.set(((this->left_eye.store.buf_cornea_inside.width() * this->left_eye.scale_cornea_inside.x) / 2.0), //
                                                          ((this->left_eye.store.buf_cornea_inside.height() * this->left_eye.scale_cornea_inside.y) / 2.0),
                                                          this->left_eye.store.buf_cornea_inside.width() * this->left_eye.scale_cornea_inside.x,
                                                          this->left_eye.store.buf_cornea_inside.height() * this->left_eye.scale_cornea_inside.y);

                    this->left_eye.store.rect_cornea_inside.set(this->left_eye.store.buf_cornea_inside.width() / 2.0, //
                                                                this->left_eye.store.buf_cornea_inside.height() / 2.0,
                                                                this->left_eye.store.buf_cornea_inside.width(),
                                                                this->left_eye.store.buf_cornea_inside.height());

#if DEBUG_OUTPUT_LOAD_IMAGE
                    printf("<reload> left_eye/cornea_inside\n");
#endif
                    break;
                }
            }
        }
        if (true == this->right_eye.store.exit_cornea_inside) {
            for (size_t i = 0; i < this->right_eye.store.cornea_inside.size(); ++i) {
                if (previous_cornea == this->right_eye.store.cornea_inside[i].id) {
                    this->right_eye.store.buf_cornea_inside = this->right_eye.store.cornea_inside[i].data[0];
                    this->right_eye.size_cornea_inside.set(((this->right_eye.store.buf_cornea_inside.width() * this->right_eye.scale_cornea_inside.x) / 2.0), //
                                                           ((this->right_eye.store.buf_cornea_inside.height() * this->right_eye.scale_cornea_inside.y) / 2.0),
                                                           this->right_eye.store.buf_cornea_inside.width() * this->right_eye.scale_cornea_inside.x,
                                                           this->right_eye.store.buf_cornea_inside.height() * this->right_eye.scale_cornea_inside.y);

                    this->right_eye.store.rect_cornea_inside.set(this->right_eye.store.buf_cornea_inside.width() / 2.0, //
                                                                 this->right_eye.store.buf_cornea_inside.height() / 2.0,
                                                                 this->right_eye.store.buf_cornea_inside.width(),
                                                                 this->right_eye.store.buf_cornea_inside.height());

#if DEBUG_OUTPUT_LOAD_IMAGE
                    printf("<reload> right_eye/cornea_inside\n");
#endif
                    break;
                }
            }
        }
#endif
    }

#if DRAW_CORNEA_OUTSIDE
    /* ============================================= */
    // left
    /* ============================================= */
    if (true == this->left_eye.store.exit_cornea_outside) {
        this->left_eye.store.matrix_cornea_outside.rotate(this->left_eye.speed_cornea_outside);
        this->left_eye.cornea_outside = this->left_eye.store.buf_cornea_outside.transformed(this->left_eye.store.matrix_cornea_outside);
        this->left_eye.target_cornea_outside.setRect(this->left_eye.eyeball_center.x - (this->left_eye.size_cornea_outside.x * this->_dimensions), //
                                                     this->left_eye.eyeball_center.y - (this->left_eye.size_cornea_outside.y * this->_dimensions),
                                                     (this->left_eye.size_cornea_outside.width * this->_dimensions),
                                                     (this->left_eye.size_cornea_outside.height * this->_dimensions));

        this->left_eye.source_cornea_outside.setRect((this->left_eye.cornea_outside.width() / 2.0) - this->left_eye.store.rect_cornea_outside.x, //
                                                     (this->left_eye.cornea_outside.height() / 2.0) - this->left_eye.store.rect_cornea_outside.y,
                                                     this->left_eye.store.rect_cornea_outside.width,
                                                     this->left_eye.store.rect_cornea_outside.height);
    }
    /* ============================================= */
    // right
    /* ============================================= */
    if (true == this->right_eye.store.exit_cornea_outside) {
        this->right_eye.store.matrix_cornea_outside.rotate(this->right_eye.speed_cornea_outside);
        this->right_eye.cornea_outside = this->right_eye.store.buf_cornea_outside.transformed(this->right_eye.store.matrix_cornea_outside);
        this->right_eye.target_cornea_outside.setRect(this->right_eye.eyeball_center.x - (this->right_eye.size_cornea_outside.x * this->_dimensions), //
                                                      this->right_eye.eyeball_center.y - (this->right_eye.size_cornea_outside.y * this->_dimensions),
                                                      (this->right_eye.size_cornea_outside.width * this->_dimensions),
                                                      (this->right_eye.size_cornea_outside.height * this->_dimensions));

        this->right_eye.source_cornea_outside.setRect((this->right_eye.cornea_outside.width() / 2.0) - this->right_eye.store.rect_cornea_outside.x, //
                                                      (this->right_eye.cornea_outside.height() / 2.0) - this->right_eye.store.rect_cornea_outside.y,
                                                      this->right_eye.store.rect_cornea_outside.width,
                                                      this->right_eye.store.rect_cornea_outside.height);
    }
#endif

#if DRAW_CORNEA_INSIDE
    /* ============================================= */
    // left
    /* ============================================= */
    if (true == this->left_eye.store.exit_cornea_inside) {
        this->left_eye.store.matrix_cornea_inside.rotate(this->left_eye.speed_cornea_inside);
        this->left_eye.cornea_inside = this->left_eye.store.buf_cornea_inside.transformed(this->left_eye.store.matrix_cornea_inside);
        this->left_eye.target_cornea_inside.setRect(this->left_eye.eyeball_center.x - (this->left_eye.size_cornea_inside.x * this->_dimensions), //
                                                    this->left_eye.eyeball_center.y - (this->left_eye.size_cornea_inside.y * this->_dimensions),
                                                    (this->left_eye.size_cornea_inside.width * this->_dimensions),
                                                    (this->left_eye.size_cornea_inside.height * this->_dimensions));

        this->left_eye.source_cornea_inside.setRect((this->left_eye.cornea_inside.width() / 2.0) - this->left_eye.store.rect_cornea_inside.x, //
                                                    (this->left_eye.cornea_inside.height() / 2.0) - this->left_eye.store.rect_cornea_inside.y,
                                                    this->left_eye.store.rect_cornea_inside.width,
                                                    this->left_eye.store.rect_cornea_inside.height);
    }
    /* ============================================= */
    // right
    /* ============================================= */
    if (true == this->right_eye.store.exit_cornea_inside) {
        this->right_eye.store.matrix_cornea_inside.rotate(this->right_eye.speed_cornea_inside);
        this->right_eye.cornea_inside = this->right_eye.store.buf_cornea_inside.transformed(this->right_eye.store.matrix_cornea_inside);
        this->right_eye.target_cornea_inside.setRect(this->right_eye.eyeball_center.x - (this->right_eye.size_cornea_inside.x * this->_dimensions), //
                                                     this->right_eye.eyeball_center.y - (this->right_eye.size_cornea_inside.y * this->_dimensions),
                                                     (this->right_eye.size_cornea_inside.width * this->_dimensions),
                                                     (this->right_eye.size_cornea_inside.height * this->_dimensions));

        this->right_eye.source_cornea_inside.setRect((this->right_eye.cornea_inside.width() / 2.0) - this->right_eye.store.rect_cornea_inside.x, //
                                                     (this->right_eye.cornea_inside.height() / 2.0) - this->right_eye.store.rect_cornea_inside.y,
                                                     this->right_eye.store.rect_cornea_inside.width,
                                                     this->right_eye.store.rect_cornea_inside.height);
    }
#endif
}
} // namespace maid_robot_system
