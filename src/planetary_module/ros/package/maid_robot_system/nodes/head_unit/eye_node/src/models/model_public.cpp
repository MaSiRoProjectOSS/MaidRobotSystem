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

#ifndef NODE_OPEN_GL_VERSION_MAJOR
#define NODE_OPEN_GL_VERSION_MAJOR 3
#endif
#ifndef NODE_OPEN_GL_VERSION_MINOR
#define NODE_OPEN_GL_VERSION_MINOR 2
#endif

#ifndef NODE_OPEN_GL_FORMAT_DEPTH_BUFFER_SIZE
#define NODE_OPEN_GL_FORMAT_DEPTH_BUFFER_SIZE 24
#endif
#ifndef NODE_OPEN_GL_FORMAT_STENCIL_BUFFER_SIZE
#define NODE_OPEN_GL_FORMAT_STENCIL_BUFFER_SIZE 8
#endif

namespace maid_robot_system
{
EyeWidget *g_widget;

bool ModelImplement::_set_param()
{
    return g_widget->set_param(this->_param);
}

std::string ModelImplement::get_lap_time()
{
    return "";
}

void ModelImplement::set_msg_eye(int emotions, int pupil_effect, float size, float distance, float left_x, float left_y, float right_x, float right_y)
{
    g_widget->cmd_eye_input(emotions, pupil_effect, size, distance, left_x, left_y, right_x, right_y);
}

bool ModelImplement::calculate()
{
    bool result = true;
    g_widget->update_screen();
    this->app->processEvents();
    return result;
}

bool ModelImplement::open(int argc, char **argv)
{
    bool result = false;
    try {
        this->app = new QApplication(argc, argv);
        // format
        QSurfaceFormat format;
        format.setStereo(true);
        format.setDepthBufferSize(NODE_OPEN_GL_FORMAT_DEPTH_BUFFER_SIZE);
        format.setStencilBufferSize(NODE_OPEN_GL_FORMAT_STENCIL_BUFFER_SIZE);
        format.setVersion(NODE_OPEN_GL_VERSION_MAJOR, NODE_OPEN_GL_VERSION_MINOR);
        if (NODE_OPEN_GL_VERSION_MAJOR <= 2) {
            format.setProfile(QSurfaceFormat::NoProfile);
        } else {
            format.setProfile(QSurfaceFormat::CoreProfile);
        }
        QSurfaceFormat::setDefaultFormat(format);
#if DEBUG_OUTPUT_OPEN_GL
        QPair<int, int> ver = format.version();
        printf("Open GL ver.%d.%d\n", ver.first, ver.second);
        switch (format.profile()) {
            case QSurfaceFormat::CoreProfile:
                printf("   profile : CoreProfile\n");
                break;
            case QSurfaceFormat::NoProfile:
                printf("   profile : NoProfile\n");
                break;
            default:
                break;
        }
#endif

        // Cursor
        QCursor::setPos(0, 0);
        QCursor cursor(Qt::BlankCursor);
        this->app->setOverrideCursor(cursor);
        this->app->changeOverrideCursor(cursor);

        // Widget
        g_widget = new EyeWidget();
        g_widget->setFormat(format);
        result = true;
    } catch (...) {
    }

    return result;
}
bool ModelImplement::exec()
{
    g_widget->showFullScreen();
    // this->app->exec();
    return true;
}

bool ModelImplement::closing()
{
    bool result = true;
    g_widget->closing();
    this->app->closeAllWindows();
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
