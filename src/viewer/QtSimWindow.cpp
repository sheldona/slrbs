#if defined(USE_QT) && defined(USE_SYSTEM_QT)

#include "viewer/QtSimWindow.h"
#include <QMatrix4x4>

// ────────────────────────────────────────────────────────────────────────
//  Cube vertex data: 8 vertices, each with position (x,y,z) and colour (r,g,b)
// ────────────────────────────────────────────────────────────────────────
static const float kVertices[] = {
    // pos               // colour
    -1.f,-1.f,-1.f,   1.f,0.f,0.f,
     1.f,-1.f,-1.f,   0.f,1.f,0.f,
     1.f, 1.f,-1.f,   0.f,0.f,1.f,
    -1.f, 1.f,-1.f,   1.f,1.f,0.f,
    -1.f,-1.f, 1.f,   1.f,0.f,1.f,
     1.f,-1.f, 1.f,   0.f,1.f,1.f,
     1.f, 1.f, 1.f,   1.f,1.f,1.f,
    -1.f, 1.f, 1.f,   0.f,0.f,0.f
};

static const unsigned int kIndices[] = {
    0,1,2, 2,3,0,   // -Z face
    4,5,6, 6,7,4,   // +Z face
    0,1,5, 5,4,0,   // -Y face
    2,3,7, 7,6,2,   // +Y face
    0,3,7, 7,4,0,   // -X face
    1,2,6, 6,5,1    // +X face
};

// ────────────────────────────────────────────────────────────────────────
QtSimWindow::QtSimWindow(QWidget* parent)
    : QOpenGLWidget(parent) {

    // ~60 fps timer to update rotation and repaint
    connect(&m_timer, &QTimer::timeout, this, [this]() {
        m_angle += 1.0f;            // 1 degree per frame
        update();
    });
    m_timer.start(16); // ≈60 fps
}

QtSimWindow::~QtSimWindow() {
    makeCurrent();
    glDeleteBuffers(1, &m_vbo);
    glDeleteBuffers(1, &m_ebo);
    glDeleteVertexArrays(1, &m_vao);
    doneCurrent();
}

void QtSimWindow::initializeGL() {
    initializeOpenGLFunctions();

    // ---- compile a very small shader program -------------------------
    m_program.addShaderFromSourceCode(QOpenGLShader::Vertex,
        "#version 330 core\n"
        "layout(location = 0) in vec3 pos;\n"
        "layout(location = 1) in vec3 col;\n"
        "out vec3 vColour;\n"
        "uniform mat4 mvp;\n"
        "void main() {\n"
        "  vColour = col;\n"
        "  gl_Position = mvp * vec4(pos, 1.0);\n"
        "}"
    );
    m_program.addShaderFromSourceCode(QOpenGLShader::Fragment,
        "#version 330 core\n"
        "in vec3 vColour;\n"
        "out vec4 frag;\n"
        "void main() { frag = vec4(vColour, 1.0); }"
    );
    m_program.link();

    // ---- create VAO / VBO / EBO --------------------------------------
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kVertices), kVertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(kIndices), kIndices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glEnable(GL_DEPTH_TEST);
}

void QtSimWindow::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void QtSimWindow::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Build MVP (model‑view‑projection) matrix
    QMatrix4x4 model;
    model.rotate(m_angle, 0.5f, 1.0f, 0.0f);

    QMatrix4x4 view;
    view.translate(0.0f, 0.0f, -6.0f);

    QMatrix4x4 proj;
    proj.perspective(45.0f, float(width()) / float(height()), 0.1f, 100.0f);

    QMatrix4x4 mvp = proj * view * model;

    m_program.bind();
    m_program.setUniformValue("mvp", mvp);

    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr);
}

#endif // defined(USE_QT) && defined(USE_SYSTEM_QT)