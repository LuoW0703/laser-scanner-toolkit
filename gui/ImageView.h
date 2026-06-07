#pragma once

#include <QGraphicsView>

class QGraphicsPixmapItem;
class QGraphicsTextItem;

/**
 * Interactive image preview for simulation evidence.
 *
 * The original pixels stay in a QGraphicsScene. Wheel zoom therefore does not
 * repeatedly resample the image, and ScrollHandDrag supplies left-button
 * panning whenever the enlarged image extends beyond the viewport.
 */
class ImageView : public QGraphicsView {
public:
    explicit ImageView(QWidget* parent = nullptr);

    bool setImage(const QString& path, const QString& errorText);
    void clearImage(const QString& message);

    void zoomIn();
    void zoomOut();
    void actualSize();
    void fitImage();

protected:
    void wheelEvent(QWheelEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;

private:
    void zoomBy(double factor);
    void centerMessage();

    QGraphicsPixmapItem* m_pixmapItem;
    QGraphicsTextItem* m_messageItem;
    bool m_fitToWindow;
};
