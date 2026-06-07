#include "EvidenceView.h"

#include "ImageView.h"
#include "PointCloudView.h"

#include <QFileInfo>
#include <QStackedWidget>
#include <QVBoxLayout>

EvidenceView::EvidenceView(QWidget* parent)
    : QWidget(parent)
    , m_stack(new QStackedWidget(this))
    , m_imageView(new ImageView())
    , m_pointCloudView(new PointCloudView()) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_stack);
    m_stack->addWidget(m_imageView);
    m_stack->addWidget(m_pointCloudView);
    m_stack->setCurrentWidget(m_imageView);
}

bool EvidenceView::setContent(
    const QString& path, const QString& errorText) {
    if (QFileInfo(path).suffix().compare(
            QStringLiteral("ply"), Qt::CaseInsensitive) == 0) {
        m_stack->setCurrentWidget(m_pointCloudView);
        return m_pointCloudView->setPointCloud(path, errorText);
    }

    m_stack->setCurrentWidget(m_imageView);
    return m_imageView->setImage(path, errorText);
}

void EvidenceView::clearContent(const QString& message) {
    m_stack->setCurrentWidget(m_imageView);
    m_imageView->clearImage(message);
    m_pointCloudView->clearPointCloud(message);
}

void EvidenceView::zoomIn() {
    if (showingPointCloud()) {
        m_pointCloudView->zoomIn();
    } else {
        m_imageView->zoomIn();
    }
}

void EvidenceView::zoomOut() {
    if (showingPointCloud()) {
        m_pointCloudView->zoomOut();
    } else {
        m_imageView->zoomOut();
    }
}

void EvidenceView::resetView() {
    if (showingPointCloud()) {
        m_pointCloudView->resetView();
    } else {
        m_imageView->actualSize();
    }
}

void EvidenceView::fitView() {
    if (showingPointCloud()) {
        m_pointCloudView->fitView();
    } else {
        m_imageView->fitImage();
    }
}

bool EvidenceView::showingPointCloud() const {
    return m_stack->currentWidget() == m_pointCloudView;
}
