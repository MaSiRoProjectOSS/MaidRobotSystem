/**
 * @file model_implement.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "models/model_implement.hpp"
#include "models/widgets/eye_widget.hpp"

#include <filesystem>

namespace maid_robot_system
{
EyeWidget *g_widget;

bool ModelImplement::calculate()
{
    bool result = true;
    // g_widget ->UpdateScreen();
    //this->app->processEvents();
    return result;
}

void ModelImplement::open(int argc, char **argv)
{
    this->app = new QApplication(argc, argv);
    QCursor::setPos(0, 0);
    QCursor cursor(Qt::BlankCursor);
    this->app->setOverrideCursor(cursor);
    this->app->changeOverrideCursor(cursor);
    // g_widget = new EyeWidget();
}

void ModelImplement::closing()
{
    this->app->closeAllWindows();
    // g_widget   ->Closing();
}

// =============================
// Constructor
// =============================
ModelImplement::ModelImplement()
{
}
ModelImplement::~ModelImplement()
{
}

} // namespace maid_robot_system
