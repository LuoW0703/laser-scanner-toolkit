#include "PointCloudView.h"

#include <QFile>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QSurfaceFormat>
#include <QTextStream>
#include <QVector4D>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace {
constexpr qsizetype kMaximumDisplayPoints = 150000;
constexpr double kZoomStep = 1.25;
constexpr double kMinimumZoom = 0.2;
constexpr double kMaximumZoom = 12.0;

struct RgbColor {
    float red;
    float green;
    float blue;
};

RgbColor heightColor(double value) {
    value = std::clamp(value, 0.0, 1.0);
    return {
        static_cast<float>(0.20 + 0.80 * value),
        static_cast<float>(
            0.35 + 0.55 * (1.0 - std::abs(2.0 * value - 1.0))),
        static_cast<float>(0.95 - 0.75 * value)
    };
}
}

PointCloudView::PointCloudView(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_dragButton(Qt::NoButton)
    , m_program(nullptr)
    , m_pointBuffer(QOpenGLBuffer::VertexBuffer)
    , m_referenceBuffer(QOpenGLBuffer::VertexBuffer)
    , m_buffersDirty(true)
    , m_yawDegrees(-38.0)
    , m_pitchDegrees(58.0)
    , m_zoom(1.0)
    , m_panX(0.0)
    , m_panY(0.0)
    , m_dataMinimum()
    , m_dataMaximum() {
    QSurfaceFormat surfaceFormat = format();
    surfaceFormat.setDepthBufferSize(24);
    surfaceFormat.setSamples(4);
    setFormat(surfaceFormat);

    setMinimumSize(280, 220);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setFocusPolicy(Qt::StrongFocus);
    setCursor(Qt::OpenHandCursor);
    setToolTip(QStringLiteral(
        "左键拖动旋转，右键拖动平移，滚轮缩放，双击复位"));
    m_referenceVertices = createReferenceVertices();
    clearPointCloud(QStringLiteral("等待三维点云"));
}

PointCloudView::~PointCloudView() {
    if (context()) {
        makeCurrent();
        m_pointBuffer.destroy();
        m_referenceBuffer.destroy();
        delete m_program;
        m_program = nullptr;
        doneCurrent();
    }
}

bool PointCloudView::setPointCloud(
    const QString& path, const QString& errorText) {
    QString error;
    if (!loadAsciiPly(path, error)) {
        clearPointCloud(
            errorText + QStringLiteral("\n") + path +
            (error.isEmpty() ? QString() : QStringLiteral("\n") + error));
        return false;
    }

    m_message.clear();
    m_buffersDirty = true;
    resetView();
    return true;
}

void PointCloudView::clearPointCloud(const QString& message) {
    m_vertices.clear();
    m_message = message;
    m_dataMinimum = QVector3D();
    m_dataMaximum = QVector3D();
    m_buffersDirty = true;
    resetView();
}

void PointCloudView::zoomIn() {
    applyZoom(kZoomStep);
}

void PointCloudView::zoomOut() {
    applyZoom(1.0 / kZoomStep);
}

void PointCloudView::resetView() {
    m_yawDegrees = -38.0;
    m_pitchDegrees = 58.0;
    m_zoom = 1.0;
    m_panX = 0.0;
    m_panY = 0.0;
    update();
}

void PointCloudView::fitView() {
    m_zoom = 1.0;
    m_panX = 0.0;
    m_panY = 0.0;
    update();
}

void PointCloudView::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.035f, 0.043f, 0.055f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_MULTISAMPLE);
    glEnable(0x8642);  // GL_PROGRAM_POINT_SIZE

    m_program = new QOpenGLShaderProgram();
    const char* vertexShader =
        "attribute highp vec3 position;\n"
        "attribute lowp vec3 color;\n"
        "uniform highp mat4 mvp;\n"
        "uniform highp float pointSize;\n"
        "varying lowp vec3 vertexColor;\n"
        "void main() {\n"
        "  gl_Position = mvp * vec4(position, 1.0);\n"
        "  gl_PointSize = pointSize;\n"
        "  vertexColor = color;\n"
        "}\n";
    const char* fragmentShader =
        "varying lowp vec3 vertexColor;\n"
        "void main() {\n"
        "  gl_FragColor = vec4(vertexColor, 1.0);\n"
        "}\n";

    if (!m_program->addShaderFromSourceCode(
            QOpenGLShader::Vertex, vertexShader) ||
        !m_program->addShaderFromSourceCode(
            QOpenGLShader::Fragment, fragmentShader)) {
        m_glError = m_program->log();
        return;
    }

    m_program->bindAttributeLocation("position", 0);
    m_program->bindAttributeLocation("color", 1);
    if (!m_program->link()) {
        m_glError = m_program->log();
        return;
    }

    if (!m_pointBuffer.create() || !m_referenceBuffer.create()) {
        m_glError = QStringLiteral("无法创建 OpenGL 顶点缓冲区");
        return;
    }
    m_buffersDirty = true;
}

void PointCloudView::resizeGL(int width, int height) {
    glViewport(0, 0, width, height);
}

void PointCloudView::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_program || !m_program->isLinked() || !m_glError.isEmpty()) {
        QPainter painter(this);
        painter.setPen(QColor(230, 100, 100));
        painter.drawText(
            rect().adjusted(16, 16, -16, -16),
            Qt::AlignCenter | Qt::TextWordWrap,
            QStringLiteral("OpenGL 初始化失败\n") + m_glError);
        return;
    }

    if (m_buffersDirty) {
        uploadBuffers();
    }

    QMatrix4x4 projection;
    const float aspect =
        height() > 0 ? static_cast<float>(width()) / height() : 1.0f;
    projection.perspective(42.0f, aspect, 0.05f, 50.0f);

    QMatrix4x4 view;
    view.translate(
        static_cast<float>(m_panX),
        static_cast<float>(m_panY),
        static_cast<float>(-4.2 / m_zoom));

    QMatrix4x4 model;
    model.rotate(static_cast<float>(m_pitchDegrees), 1.0f, 0.0f, 0.0f);
    model.rotate(static_cast<float>(m_yawDegrees), 0.0f, 0.0f, 1.0f);
    const QMatrix4x4 mvp = projection * view * model;

    m_program->bind();
    m_program->setUniformValue("mvp", mvp);
    m_program->setUniformValue(
        "pointSize",
        static_cast<GLfloat>(
            std::clamp(2.0 * devicePixelRatioF(), 1.5, 5.0)));
    m_program->enableAttributeArray(0);
    m_program->enableAttributeArray(1);

    m_referenceBuffer.bind();
    m_program->setAttributeBuffer(
        0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));
    m_program->setAttributeBuffer(
        1, GL_FLOAT, offsetof(Vertex, red), 3, sizeof(Vertex));
    glLineWidth(1.0f);
    glDrawArrays(GL_LINES, 0, m_referenceVertices.size());
    m_referenceBuffer.release();

    if (!m_vertices.isEmpty()) {
        m_pointBuffer.bind();
        m_program->setAttributeBuffer(
            0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));
        m_program->setAttributeBuffer(
            1, GL_FLOAT, offsetof(Vertex, red), 3, sizeof(Vertex));
        glDrawArrays(GL_POINTS, 0, m_vertices.size());
        m_pointBuffer.release();
    }

    m_program->disableAttributeArray(0);
    m_program->disableAttributeArray(1);
    m_program->release();

    QPainter painter(this);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    if (m_vertices.isEmpty()) {
        painter.setPen(QColor(153, 153, 153));
        painter.drawText(
            rect().adjusted(16, 16, -16, -16),
            Qt::AlignCenter | Qt::TextWordWrap, m_message);
        return;
    }

    painter.setPen(QColor(225, 225, 225));
    painter.drawText(
        QRect(10, 8, width() - 20, 24),
        Qt::AlignLeft | Qt::AlignVCenter,
        QStringLiteral(
            "%1 点 | OpenGL 深度场景 | 左键旋转 | 右键平移 | 滚轮缩放")
            .arg(m_vertices.size()));
    painter.setPen(QColor(210, 210, 210));
    painter.drawText(
        QRect(10, height() - 48, width() - 20, 20),
        Qt::AlignLeft | Qt::AlignVCenter,
        QStringLiteral("X 红色 | Y 绿色 | Z 蓝色 | 网格为 XY 平面"));
    painter.drawText(
        QRect(10, height() - 28, width() - 20, 20),
        Qt::AlignLeft | Qt::AlignVCenter,
        QStringLiteral(
            "X [%1, %2] mm | Y [%3, %4] mm | Z [%5, %6] mm")
            .arg(m_dataMinimum.x(), 0, 'f', 1)
            .arg(m_dataMaximum.x(), 0, 'f', 1)
            .arg(m_dataMinimum.y(), 0, 'f', 1)
            .arg(m_dataMaximum.y(), 0, 'f', 1)
            .arg(m_dataMinimum.z(), 0, 'f', 1)
            .arg(m_dataMaximum.z(), 0, 'f', 1));
}

void PointCloudView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton ||
        event->button() == Qt::RightButton) {
        m_dragButton = event->button();
        m_lastMousePosition = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QOpenGLWidget::mousePressEvent(event);
}

void PointCloudView::mouseMoveEvent(QMouseEvent* event) {
    if (m_dragButton == Qt::NoButton) {
        QOpenGLWidget::mouseMoveEvent(event);
        return;
    }

    const QPoint delta = event->pos() - m_lastMousePosition;
    m_lastMousePosition = event->pos();
    if (m_dragButton == Qt::LeftButton) {
        m_yawDegrees += delta.x() * 0.45;
        m_pitchDegrees = std::clamp(
            m_pitchDegrees + delta.y() * 0.45, -89.0, 89.0);
    } else {
        const double worldPerPixel =
            3.2 / std::max(1, std::min(width(), height())) / m_zoom;
        m_panX += delta.x() * worldPerPixel;
        m_panY -= delta.y() * worldPerPixel;
    }
    update();
    event->accept();
}

void PointCloudView::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == m_dragButton) {
        m_dragButton = Qt::NoButton;
        setCursor(Qt::OpenHandCursor);
        event->accept();
        return;
    }
    QOpenGLWidget::mouseReleaseEvent(event);
}

void PointCloudView::mouseDoubleClickEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        resetView();
        event->accept();
        return;
    }
    QOpenGLWidget::mouseDoubleClickEvent(event);
}

void PointCloudView::wheelEvent(QWheelEvent* event) {
    if (event->angleDelta().y() == 0) {
        QOpenGLWidget::wheelEvent(event);
        return;
    }
    applyZoom(event->angleDelta().y() > 0 ? kZoomStep : 1.0 / kZoomStep);
    event->accept();
}

bool PointCloudView::loadAsciiPly(const QString& path, QString& error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        error = QStringLiteral("无法打开点云文件");
        return false;
    }

    QTextStream stream(&file);
    if (stream.readLine().trimmed() != QStringLiteral("ply")) {
        error = QStringLiteral("不是有效的 PLY 文件");
        return false;
    }

    bool ascii = false;
    bool inVertexElement = false;
    bool headerEnded = false;
    qsizetype vertexCount = 0;
    QStringList properties;
    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (line == QStringLiteral("end_header")) {
            headerEnded = true;
            break;
        }

        const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
        if (parts.size() >= 2 && parts[0] == QStringLiteral("format")) {
            ascii = parts[1] == QStringLiteral("ascii");
        } else if (parts.size() >= 3 &&
                   parts[0] == QStringLiteral("element")) {
            inVertexElement = parts[1] == QStringLiteral("vertex");
            if (inVertexElement) {
                bool ok = false;
                vertexCount = parts[2].toLongLong(&ok);
                if (!ok || vertexCount <= 0) {
                    error = QStringLiteral("PLY 顶点数量无效");
                    return false;
                }
                properties.clear();
            }
        } else if (parts.size() >= 3 && inVertexElement &&
                   parts[0] == QStringLiteral("property")) {
            properties.append(parts.last());
        }
    }

    if (!headerEnded || !ascii) {
        error = QStringLiteral("仅支持完整的 ASCII PLY");
        return false;
    }

    const int xIndex = properties.indexOf(QStringLiteral("x"));
    const int yIndex = properties.indexOf(QStringLiteral("y"));
    const int zIndex = properties.indexOf(QStringLiteral("z"));
    const int redIndex = properties.indexOf(QStringLiteral("red"));
    const int greenIndex = properties.indexOf(QStringLiteral("green"));
    const int blueIndex = properties.indexOf(QStringLiteral("blue"));
    if (xIndex < 0 || yIndex < 0 || zIndex < 0) {
        error = QStringLiteral("PLY 缺少 x/y/z 坐标");
        return false;
    }

    struct RawPoint {
        double x;
        double y;
        double z;
        RgbColor color;
        bool hasColor;
    };
    QVector<RawPoint> rawPoints;
    const qsizetype stride =
        std::max<qsizetype>(
            1, (vertexCount + kMaximumDisplayPoints - 1) /
                   kMaximumDisplayPoints);
    rawPoints.reserve(
        std::min<qsizetype>(vertexCount, kMaximumDisplayPoints));

    double xMin = std::numeric_limits<double>::max();
    double xMax = std::numeric_limits<double>::lowest();
    double yMin = std::numeric_limits<double>::max();
    double yMax = std::numeric_limits<double>::lowest();
    double zMin = std::numeric_limits<double>::max();
    double zMax = std::numeric_limits<double>::lowest();

    for (qsizetype index = 0; index < vertexCount && !stream.atEnd(); ++index) {
        const QStringList values =
            stream.readLine().split(' ', Qt::SkipEmptyParts);
        if (values.size() < properties.size()) {
            error = QStringLiteral("PLY 顶点数据不完整");
            return false;
        }
        if (index % stride != 0) {
            continue;
        }

        bool xOk = false;
        bool yOk = false;
        bool zOk = false;
        const double x = values[xIndex].toDouble(&xOk);
        const double y = values[yIndex].toDouble(&yOk);
        const double z = values[zIndex].toDouble(&zOk);
        if (!xOk || !yOk || !zOk ||
            !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            error = QStringLiteral("PLY 包含无效坐标");
            return false;
        }

        const bool hasColor =
            redIndex >= 0 && greenIndex >= 0 && blueIndex >= 0;
        RgbColor color{};
        if (hasColor) {
            color = {
                static_cast<float>(
                    std::clamp(values[redIndex].toInt(), 0, 255) / 255.0),
                static_cast<float>(
                    std::clamp(values[greenIndex].toInt(), 0, 255) / 255.0),
                static_cast<float>(
                    std::clamp(values[blueIndex].toInt(), 0, 255) / 255.0)
            };
        }
        rawPoints.append({x, y, z, color, hasColor});
        xMin = std::min(xMin, x);
        xMax = std::max(xMax, x);
        yMin = std::min(yMin, y);
        yMax = std::max(yMax, y);
        zMin = std::min(zMin, z);
        zMax = std::max(zMax, z);
    }

    if (rawPoints.isEmpty()) {
        error = QStringLiteral("PLY 中没有可显示的点");
        return false;
    }

    const double centerX = (xMin + xMax) * 0.5;
    const double centerY = (yMin + yMax) * 0.5;
    const double centerZ = (zMin + zMax) * 0.5;
    const double radius =
        std::max({xMax - xMin, yMax - yMin, zMax - zMin, 1e-9}) * 0.5;
    const double zSpan = std::max(zMax - zMin, 1e-9);
    m_dataMinimum = QVector3D(
        static_cast<float>(xMin),
        static_cast<float>(yMin),
        static_cast<float>(zMin));
    m_dataMaximum = QVector3D(
        static_cast<float>(xMax),
        static_cast<float>(yMax),
        static_cast<float>(zMax));

    QVector<Vertex> loaded;
    loaded.reserve(rawPoints.size());
    for (const RawPoint& point : rawPoints) {
        const RgbColor color = point.hasColor
            ? point.color
            : heightColor((point.z - zMin) / zSpan);
        loaded.append({
            static_cast<float>((point.x - centerX) / radius),
            static_cast<float>((point.y - centerY) / radius),
            static_cast<float>((point.z - centerZ) / radius),
            color.red,
            color.green,
            color.blue
        });
    }
    m_vertices = std::move(loaded);
    return true;
}

void PointCloudView::applyZoom(double factor) {
    m_zoom = std::clamp(
        m_zoom * factor, kMinimumZoom, kMaximumZoom);
    update();
}

void PointCloudView::uploadBuffers() {
    if (!m_pointBuffer.isCreated() || !m_referenceBuffer.isCreated()) {
        return;
    }

    m_pointBuffer.bind();
    m_pointBuffer.allocate(
        m_vertices.constData(),
        static_cast<int>(m_vertices.size() * sizeof(Vertex)));
    m_pointBuffer.release();

    m_referenceBuffer.bind();
    m_referenceBuffer.allocate(
        m_referenceVertices.constData(),
        static_cast<int>(m_referenceVertices.size() * sizeof(Vertex)));
    m_referenceBuffer.release();
    m_buffersDirty = false;
}

QVector<PointCloudView::Vertex>
PointCloudView::createReferenceVertices() const {
    QVector<Vertex> vertices;
    auto addLine = [&vertices](
        float x1, float y1, float z1,
        float x2, float y2, float z2,
        float red, float green, float blue) {
        vertices.append({x1, y1, z1, red, green, blue});
        vertices.append({x2, y2, z2, red, green, blue});
    };

    constexpr float gridExtent = 1.2f;
    constexpr float gridZ = -1.05f;
    constexpr int gridSteps = 12;
    for (int index = -gridSteps; index <= gridSteps; ++index) {
        const float value =
            gridExtent * static_cast<float>(index) / gridSteps;
        const float intensity = index == 0 ? 0.34f : 0.16f;
        addLine(
            -gridExtent, value, gridZ,
            gridExtent, value, gridZ,
            intensity, intensity, intensity);
        addLine(
            value, -gridExtent, gridZ,
            value, gridExtent, gridZ,
            intensity, intensity, intensity);
    }

    addLine(0.0f, 0.0f, gridZ, 1.45f, 0.0f, gridZ, 1.0f, 0.18f, 0.18f);
    addLine(0.0f, 0.0f, gridZ, 0.0f, 1.45f, gridZ, 0.18f, 1.0f, 0.18f);
    addLine(0.0f, 0.0f, gridZ, 0.0f, 0.0f, 1.45f, 0.20f, 0.45f, 1.0f);
    return vertices;
}
