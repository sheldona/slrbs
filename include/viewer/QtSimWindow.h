#pragma once

#if defined(USE_QT) && defined(USE_SYSTEM_QT)

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QTimer>

class QtSimWindow : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core {
    Q_OBJECT
public:
    explicit QtSimWindow(QWidget* parent = nullptr);
    ~QtSimWindow() override;

protected:
    // QOpenGLWidget overrides
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void initCube();

    QOpenGLShaderProgram m_program;  // simple colour shader
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    GLuint m_ebo = 0;

    float  m_angle = 0.0f;           // rotation angle (degrees)
    QTimer m_timer;                  // drives animation
};

#else
// Dummy class for when Qt is disabled
class QtSimWindow {
public:
    explicit QtSimWindow(void* = nullptr) {}
    ~QtSimWindow() = default;
};
#endif