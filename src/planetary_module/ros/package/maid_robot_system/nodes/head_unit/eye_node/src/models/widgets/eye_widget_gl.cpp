/**
 * @file eye_widget_gl.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

namespace maid_robot_system
{
void EyeWidget::initializeGL()
{
    //   QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    //  f->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

#if 0
    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);
    //vboの作成
    glGenBuffers(1, &m_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(GLfloat), m_vertices.constData(), GL_STATIC_DRAW);
    //vertex shaderのコードで頂点座標のロケーションは0に指定済
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
#else

    initializeOpenGLFunctions(); //初期化
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    m_program = glCreateProgram();
    //vertex shaderの作成
    GLuint vshader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vshader, 1, &vshader_src, 0);
    glCompileShader(vshader);
    //fragment shaderの作成
    GLuint fshader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fshader, 1, &fshader_src, 0);
    glCompileShader(fshader);
    //プログラムにアタッチ
    glAttachShader(m_program, vshader);
    glAttachShader(m_program, fshader);
    //リンク
    glLinkProgram(m_program);
    //削除
    glDeleteShader(vshader);
    glDeleteShader(fshader);
#endif
}

void EyeWidget::resizeGL(int w, int h)
{
    //m_projection.setToIdentity();
    //m_projection.perspective(45.0f, w / float(h), 0.01f, 100.0f);
}

void EyeWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(m_program);
    //glBindVertexArray(m_vao);
    glDrawArrays(GL_TRIANGLES, 0, m_vertices.size() / 3); //ドローコール
}

} // namespace maid_robot_system
