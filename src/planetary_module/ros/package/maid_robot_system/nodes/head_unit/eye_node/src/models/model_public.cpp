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

#include <QPair>
#include <filesystem>

namespace maid_robot_system
{
EyeWidget *g_widget;

bool ModelImplement::calculate()
{
    bool result = true;
    g_widget->UpdateScreen();
    this->app->processEvents();
    return result;
}

bool ModelImplement::open(int argc, char **argv)
{
    bool result = true;
    this->app   = new QApplication(argc, argv);
    // format
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
#if 0
    format.setVersion(3, 2);
#else
    format.setVersion(2, 0);
#endif
    QPair<int, int> ver = format.version();
    format.setProfile(QSurfaceFormat::CoreProfile);
    //QSurfaceFormat::setDefaultFormat(format);
    printf("Open GL ver.%d.%d\n", ver.first, ver.second);

    // Cursor
    QCursor::setPos(0, 0);
    QCursor cursor(Qt::BlankCursor);
    this->app->setOverrideCursor(cursor);
    this->app->changeOverrideCursor(cursor);

    // Widget
    g_widget = new EyeWidget();
    g_widget->show();
    return result;
}
bool ModelImplement::exec()
{
    return this->app->exec();
}

bool ModelImplement::closing()
{
    bool result = true;
    this->app->closeAllWindows();
    // g_widget   ->Closing();
    return result;
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
