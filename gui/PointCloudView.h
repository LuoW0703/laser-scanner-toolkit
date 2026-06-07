#pragma once

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector>
#include <QVector3D>

class QOpenGLShaderProgram;

/**
 * OpenGL point-cloud viewer used by three-dimensional evidence records.
 *
 * Unlike the earlier painter projection, this widget sends vertices through
 * an actual perspective projection and OpenGL depth buffer. Grid lines,
 * coordinate axes, and cloud points therefore share the same 3D camera and
 * obey real front/back occlusion while the user rotates the scene.
 */
class PointCloudView : public QOpenGLWidget, protected QOpenGLFunctions {
public:
    explicit PointCloudView(QWidget* parent = nullptr);
    ~PointCloudView() override;

    bool setPointCloud(const QString& path, const QString& errorText);
    void clearPointCloud(const QString& message);

    void zoomIn();
    void zoomOut();
    void resetView();
    void fitView();

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    struct Vertex {
        float x;
        float y;
        float z;
        float red;
        float green;
        float blue;
    };

    bool loadAsciiPly(const QString& path, QString& error);
    void applyZoom(double factor);
    void uploadBuffers();
    QVector<Vertex> createReferenceVertices() const;

    QVector<Vertex> m_vertices;
    QVector<Vertex> m_referenceVertices;
    QString m_message;
    QString m_glError;
    QPoint m_lastMousePosition;
    Qt::MouseButton m_dragButton;

    QOpenGLShaderProgram* m_program;
    QOpenGLBuffer m_pointBuffer;
    QOpenGLBuffer m_referenceBuffer;
    bool m_buffersDirty;

    double m_yawDegrees;
    double m_pitchDegrees;
    double m_zoom;
    double m_panX;
    double m_panY;
    QVector3D m_dataMinimum;
    QVector3D m_dataMaximum;
};
