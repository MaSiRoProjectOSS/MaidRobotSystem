/**
 * @file model_gl.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/model_implement.hpp"

namespace maid_robot_system
{
bool ModelImplement::event(QEvent *e)
{
    return true;
}

void ModelImplement::paintEvent(QPaintEvent *event)
{
}

void ModelImplement::resizeEvent(QResizeEvent *event)
{
}

} // namespace maid_robot_system
