#ifndef IMAGEITEM_H
#define IMAGEITEM_H

#include <QGraphicsPixmapItem>
#include <QPainter>

class ImageItem: public QGraphicsPixmapItem
{
public:
    ImageItem();
    ~ImageItem();

    QRectF boundingRect() const Q_DECL_OVERRIDE;
    QPainterPath shape() const Q_DECL_OVERRIDE;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) Q_DECL_OVERRIDE;


    void setMyPixmap(QPixmap &pixmap,int width,int height);
};

#endif // IMAGEITEM_H
