#include "ImageView.h"

#include <QColor>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QWheelEvent>

#include <algorithm>

namespace {
constexpr double kZoomStep = 1.25;
constexpr double kMinimumScale = 0.02;
constexpr double kMaximumScale = 50.0;
}

ImageView::ImageView(QWidget* parent)
    : QGraphicsView(parent)
    , m_pixmapItem(nullptr)
    , m_messageItem(nullptr)
    , m_fitToWindow(true) {
    setScene(new QGraphicsScene(this));
    setAlignment(Qt::AlignCenter);
    setBackgroundBrush(QColor(17, 17, 17));
    setFrameShape(QFrame::Box);
    setFrameShadow(QFrame::Plain);
    setStyleSheet("QGraphicsView { border:1px solid #333; }");
    setDragMode(QGraphicsView::ScrollHandDrag);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorViewCenter);
    setRenderHint(QPainter::SmoothPixmapTransform, true);
    setMinimumSize(280, 220);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setToolTip(QStringLiteral(
        "鼠标滚轮缩放，按住左键拖动，双击适应窗口"));
}

bool ImageView::setImage(const QString& path, const QString& errorText) {
    const QPixmap pixmap(path);
    if (pixmap.isNull()) {
        clearImage(errorText + QStringLiteral("\n") + path);
        return false;
    }

    scene()->clear();
    m_messageItem = nullptr;
    m_pixmapItem = scene()->addPixmap(pixmap);
    scene()->setSceneRect(m_pixmapItem->boundingRect());
    setDragMode(QGraphicsView::ScrollHandDrag);
    m_fitToWindow = true;
    fitImage();
    return true;
}

void ImageView::clearImage(const QString& message) {
    scene()->clear();
    m_pixmapItem = nullptr;
    resetTransform();
    setDragMode(QGraphicsView::NoDrag);
    m_messageItem = scene()->addText(message);
    m_messageItem->setDefaultTextColor(QColor(153, 153, 153));
    m_fitToWindow = true;
    centerMessage();
}

void ImageView::zoomIn() {
    zoomBy(kZoomStep);
}

void ImageView::zoomOut() {
    zoomBy(1.0 / kZoomStep);
}

void ImageView::actualSize() {
    if (!m_pixmapItem) {
        return;
    }

    resetTransform();
    centerOn(m_pixmapItem);
    m_fitToWindow = false;
}

void ImageView::fitImage() {
    if (!m_pixmapItem || viewport()->width() <= 1 || viewport()->height() <= 1) {
        return;
    }

    resetTransform();
    fitInView(m_pixmapItem, Qt::KeepAspectRatio);
    m_fitToWindow = true;
}

void ImageView::wheelEvent(QWheelEvent* event) {
    if (!m_pixmapItem || event->angleDelta().y() == 0) {
        QGraphicsView::wheelEvent(event);
        return;
    }

    zoomBy(event->angleDelta().y() > 0 ? kZoomStep : 1.0 / kZoomStep);
    event->accept();
}

void ImageView::resizeEvent(QResizeEvent* event) {
    QGraphicsView::resizeEvent(event);
    if (m_fitToWindow) {
        fitImage();
    } else if (m_messageItem) {
        centerMessage();
    }
}

void ImageView::mouseDoubleClickEvent(QMouseEvent* event) {
    if (m_pixmapItem && event->button() == Qt::LeftButton) {
        fitImage();
        event->accept();
        return;
    }
    QGraphicsView::mouseDoubleClickEvent(event);
}

void ImageView::zoomBy(double factor) {
    if (!m_pixmapItem) {
        return;
    }

    const double currentScale = transform().m11();
    const double targetScale =
        std::clamp(currentScale * factor, kMinimumScale, kMaximumScale);
    const double appliedFactor = targetScale / currentScale;
    if (appliedFactor != 1.0) {
        scale(appliedFactor, appliedFactor);
    }
    m_fitToWindow = false;
}

void ImageView::centerMessage() {
    if (!m_messageItem) {
        return;
    }

    const QRectF viewportRect(
        0.0, 0.0, viewport()->width(), viewport()->height());
    scene()->setSceneRect(viewportRect);
    const QRectF textRect = m_messageItem->boundingRect();
    m_messageItem->setPos(
        (viewportRect.width() - textRect.width()) / 2.0,
        (viewportRect.height() - textRect.height()) / 2.0);
}
