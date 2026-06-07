#pragma once

#include <QWidget>

class ImageView;
class PointCloudView;
class QStackedWidget;

/** Selects a 2D image viewer or a 3D PLY viewer from the evidence path. */
class EvidenceView : public QWidget {
public:
    explicit EvidenceView(QWidget* parent = nullptr);

    bool setContent(const QString& path, const QString& errorText);
    void clearContent(const QString& message);

    void zoomIn();
    void zoomOut();
    void resetView();
    void fitView();

private:
    bool showingPointCloud() const;

    QStackedWidget* m_stack;
    ImageView* m_imageView;
    PointCloudView* m_pointCloudView;
};
