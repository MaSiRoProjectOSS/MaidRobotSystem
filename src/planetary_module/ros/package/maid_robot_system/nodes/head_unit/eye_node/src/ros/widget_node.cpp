/**
 * @file widget_node.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "ros/widget_node.hpp"

#include <QPair>
#include <exception>
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
// =============================
// PUBLIC : Function
// =============================
bool WidgetNode::exec_start()
{
#if DEBUG_OUTPUT_WIDGET
    printf("[Widget] start...\n");
#endif
    this->_is_running = true;
    return this->_widget->exec_start();
}

void WidgetNode::respawn()
{
    this->_app->closeAllWindows();
}

int WidgetNode::running_exec()
{
    int result = 0;
    do {
        result = this->_app->exec();
        if (true == this->_is_running) {
            result = this->_widget->exec_start();
        }
    } while (true == this->_is_running);
    return result;
}

bool WidgetNode::closing()
{
    bool result       = false;
    this->_is_running = false;
    this->_widget->closing();
    this->_app->closeAllWindows();
#if DEBUG_OUTPUT_WIDGET
    printf("[Widget] closing...\n");
#endif
    result = true;
    return result;
}

// =============================
// Constructor
// =============================
WidgetNode::WidgetNode(std::string node_name, int argc, char **argv)
{
    try {
        this->_app = new QApplication(argc, argv);
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
        this->_app->setOverrideCursor(cursor);
        this->_app->changeOverrideCursor(cursor);

        // Widget
        this->_widget = new EyeWidget();
        this->_widget->setFormat(format);
    } catch (...) {
        throw new std::runtime_error("Failed to initialize widget.");
    }
}

bool WidgetNode::is_response()
{
    return this->_widget->is_response();
}

WidgetNode::~WidgetNode()
{
}

} // namespace maid_robot_system
