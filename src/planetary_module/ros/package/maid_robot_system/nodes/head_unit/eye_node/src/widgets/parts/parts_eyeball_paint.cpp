/**
 * @file parts_eyeball.cpp
 * @brief
 * @date 2020-03-27
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "widgets/parts/parts_eyeball.hpp"

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
        painter.drawPixmap(this->right_eye.draw_cornea_anime.x, this->right_eye.draw_cornea_anime.y, this->right_eye.cornea_outside);
    }
    if (true == this->left_eye.store.exit_cornea_outside) {
        painter.drawPixmap(this->left_eye.draw_cornea_anime.x, this->left_eye.draw_cornea_anime.y, this->left_eye.cornea_outside);
    }
}
void PartsEyeball::update_cornea_inside(QPainter &painter)
{
    if (true == this->right_eye.store.exit_cornea_inside) {
        painter.drawPixmap(this->right_eye.draw_cornea_anime2.x, this->right_eye.draw_cornea_anime2.y, this->right_eye.cornea_inside);
    }
    if (true == this->left_eye.store.exit_cornea_inside) {
        painter.drawPixmap(this->left_eye.draw_cornea_anime2.x, this->left_eye.draw_cornea_anime2.y, this->left_eye.cornea_inside);
    }
}

void PartsEyeball::calculate(int progress, int elapsed)
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
    this->left_eye.set_draw_pixel(progress, this->_dimensions, this->left_eye.calibration_angle_cos, this->left_eye.calibration_angle_sin);
    left_eye.eyeball_center.setX(this->left_eye.draw_postion.x + (this->left_eye.draw_postion.width / 2.0));
    left_eye.eyeball_center.setY(this->left_eye.draw_postion.y + (this->left_eye.draw_postion.height / 2.0));
    // right
    this->right_eye.set_draw_pixel(progress, this->_dimensions, this->right_eye.calibration_angle_cos, this->right_eye.calibration_angle_sin);
    right_eye.eyeball_center.setX(this->right_eye.draw_postion.x + (this->right_eye.draw_postion.width / 2.0));
    right_eye.eyeball_center.setY(this->right_eye.draw_postion.y + (this->right_eye.draw_postion.height / 2.0));

    /* ============================================= */
    // drawing
    /* ============================================= */
    this->_drawing(state);
}

} // namespace maid_robot_system
