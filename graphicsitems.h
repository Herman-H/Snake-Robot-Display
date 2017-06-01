#ifndef GRAPHICSITEMS_H
#define GRAPHICSITEMS_H

#include <QObject>
#include <QGraphicsItem>
#include <QGraphicsPolygonItem>
#include <QPen>
#include <QTransform>
#include "dimensions.h"

#define SCALE_ALL_FACTOR 400

class GraphicsSegmentItem : public QGraphicsItem
{
public:
    GraphicsSegmentItem(const int segment, const int numSegments) :
        m_segment(segment)
    {
        typedef robot_dimensions<SCALE_ALL_FACTOR> RD;
        typedef CenterOfMass_dimensions<SCALE_ALL_FACTOR> CMD;
        QGraphicsRectItem * segmentRect = new QGraphicsRectItem(-RD::segmentMCtoEdgeBackward(m_segment),
                                                                -RD::segmentMCtoEdgeLeft(m_segment),
                                                                RD::segmentMCtoEdgeBackward(m_segment)+RD::segmentMCtoEdgeForward(m_segment),
                                                                RD::segmentMCtoEdgeLeft(m_segment)+RD::segmentMCtoEdgeRight(m_segment));
        QPen segmentPen;
        segmentPen.setStyle(Qt::SolidLine);
        segmentPen.setBrush(Qt::black);
        segmentRect->setPen(segmentPen);
        QBrush segmentBrush;
        segmentBrush.setStyle(Qt::SolidPattern);
        segmentBrush.setColor(Qt::green);
        segmentRect->setBrush(segmentBrush);
        segmentRect->setParentItem(this);

        QGraphicsEllipseItem * cmCircle = new QGraphicsEllipseItem(-CMD::circleRadius(m_segment),
                                                                   -CMD::circleRadius(m_segment),
                                                                   2*CMD::circleRadius(m_segment),
                                                                   2*CMD::circleRadius(m_segment));
        QPen cmPen;
        cmPen.setStyle(Qt::SolidLine);
        cmPen.setBrush(Qt::red);
        cmCircle->setPen(cmPen);
        QBrush cmBrush;
        cmBrush.setStyle(Qt::SolidPattern);
        cmBrush.setColor(Qt::red);
        cmCircle->setBrush(cmBrush);
        cmCircle->setParentItem(this);

        if(segment != 0)
        {
            // the segment is not in the front, ie does have a joint at the front
            QGraphicsRectItem * jointFront = new QGraphicsRectItem(RD::segmentMCtoForwardJointBegin(m_segment),
                                                                  -0.5f*RD::jointForwardWidth(m_segment),
                                                                  RD::segmentMCtoForwardJointEnd(m_segment)-RD::segmentMCtoForwardJointBegin(m_segment),
                                                                  RD::jointForwardWidth(m_segment));
            QPen frontPen;
            frontPen.setStyle(Qt::SolidLine);
            frontPen.setBrush(Qt::black);
            jointFront->setPen(frontPen);
            QBrush frontBrush;
            frontBrush.setStyle(Qt::SolidPattern);
            frontBrush.setColor(Qt::white);
            jointFront->setBrush(frontBrush);
            jointFront->setParentItem(this);
        }
        if(segment != numSegments-1)
        {
            // the segment is not the last one, ie does have a joint at the back
            QGraphicsRectItem * jointBack = new QGraphicsRectItem(-RD::segmentMCtoBackwardJointEnd(m_segment),
                                                                  -0.5f*RD::jointBackwardWidth(m_segment),
                                                                  RD::segmentMCtoBackwardJointEnd(m_segment)-RD::segmentMCtoBackwardJointBegin(m_segment),
                                                                  RD::jointBackwardWidth(m_segment));
            QPen backPen;
            backPen.setStyle(Qt::SolidLine);
            backPen.setBrush(Qt::black);
            jointBack->setPen(backPen);
            QBrush backBrush;
            backBrush.setStyle(Qt::SolidPattern);
            backBrush.setColor(Qt::blue);
            jointBack->setBrush(backBrush);
            jointBack->setParentItem(this);
        }
    }
    ~GraphicsSegmentItem(){}

    QRectF boundingRect() const
    {
        typedef robot_dimensions<SCALE_ALL_FACTOR> RD;
        return QRectF(-RD::segmentMCtoBackwardJointEnd(m_segment),
                      -RD::segmentMCtoEdgeLeft(m_segment),
                      RD::segmentMCtoBackwardJointEnd(m_segment)+RD::segmentMCtoForwardJointEnd(m_segment),
                      RD::segmentMCtoEdgeLeft(m_segment)+RD::segmentMCtoEdgeRight(m_segment));
    }

    void paint(QPainter* /*painter*/, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)

    {
    }


private:
    const int m_segment;
};

class GraphicsArrowItem : public QGraphicsItem
{
public:
    GraphicsArrowItem(int color)
    {
        typedef Arrow_dimensions<SCALE_ALL_FACTOR> AD;
        QGraphicsRectItem * arrowBaseRect = new QGraphicsRectItem(0.0f,-AD::arrowBreadth()*0.5f,
                                                                  AD::arrowLength()-AD::arrowHeadLength(),
                                                                  AD::arrowBreadth());
        QVector<QPoint> points;
        points.push_back(QPoint(AD::arrowLength()-AD::arrowHeadLength(),-AD::arrowHeadBreadth()*0.5f));
        points.push_back(QPoint(AD::arrowLength(),0.0f));
        points.push_back(QPoint(AD::arrowLength()-AD::arrowHeadLength(),AD::arrowHeadBreadth()*0.5f));
        QGraphicsPolygonItem * arrowHeadTri = new QGraphicsPolygonItem(QPolygonF(points));

        QPen arrowPen;
        arrowPen.setStyle(Qt::SolidLine);
        arrowPen.setWidth(1);
        arrowPen.setBrush(Qt::black);
        arrowBaseRect->setPen(arrowPen);
        arrowHeadTri->setPen(arrowPen);
        QBrush arrowBrush;
        arrowBrush.setStyle(Qt::SolidPattern);
        arrowBrush.setColor(Qt::GlobalColor(color));
        arrowBaseRect->setBrush(arrowBrush);
        arrowHeadTri->setBrush(arrowBrush);
        arrowBaseRect->setParentItem(this);
        arrowHeadTri->setParentItem(this);
    }

    ~GraphicsArrowItem(){}

    QRectF boundingRect() const
    {
        typedef Arrow_dimensions<SCALE_ALL_FACTOR> AD;
        return QRectF(0,
                      -AD::arrowHeadBreadth()*0.5f,
                      AD::arrowLength(),
                      AD::arrowHeadBreadth());
    }

    void paint(QPainter* /*painter*/, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
    {
    }

    void modify(float x, float y, bool show)
    {
        qreal len = std::sqrt(x*x+y*y);
        if(len > 0.1f)
        {
            QTransform p; // Scale length but maintain height
            p.rotate(std::atan2(y,x)*180.0f/3.1415f);
            p.scale(len,1);
            this->setTransform(p);
            //this->setRotation(std::atan2(y,x)*180.0f/3.1415f);
            this->setVisible(show);
        }
        else
        {
            this->setVisible(false);
        }
    }
};

class GraphicsTorqueDisplay : public QGraphicsItem
{
private:
    int numJoints;
    QVector<QGraphicsRectItem*> meters;
    const float itemheight;
    const float meterwidth;
    const float length;
    const float scale;
public:
    GraphicsTorqueDisplay(float scale, float length, int numberOfJoints, int torqueColor) :
        numJoints(numberOfJoints),
        itemheight(0.2f*SCALE_ALL_FACTOR),
        meterwidth(0.06f*SCALE_ALL_FACTOR),
        length(length),
        scale(scale)
    {
        float xstart = -length*0.5f;
        float ystart = -itemheight*0.5f;

        QGraphicsTextItem * nmLabel = new QGraphicsTextItem("nm");
        nmLabel->setPos(xstart-nmLabel->boundingRect().height()/2,
                        ystart-nmLabel->boundingRect().height());
        nmLabel->setParentItem(this);
        QGraphicsLineItem * scaleLine = new QGraphicsLineItem(xstart,ystart,xstart,ystart+itemheight);
        QPen scaleLinePen;
        scaleLinePen.setStyle(Qt::SolidLine);
        scaleLine->setPen(scaleLinePen);
        scaleLine->setParentItem(this);


        float freeSpace = length - meterwidth*numberOfJoints;
        // Choose indentation such that it is 0.5 of the spacing between the meters
        float spacing = freeSpace / float(numberOfJoints);
        float indentation = spacing/2;

        for(int i = 0; i < numberOfJoints; ++i)
        {
            float meterXStart = xstart + indentation + i*(meterwidth+spacing);
            float meterYStart = 0.0f;
            float meterWidth = meterwidth;
            float meterHeight = 0.0f;
            QGraphicsRectItem * m = new QGraphicsRectItem(meterXStart,meterYStart,meterWidth,meterHeight);
            QPen mPen;
            mPen.setStyle(Qt::SolidLine);
            mPen.setBrush(Qt::black);
            m->setPen(mPen);
            QBrush mBrush;
            mBrush.setStyle(Qt::SolidPattern);
            mBrush.setColor(Qt::GlobalColor(torqueColor));
            m->setBrush(mBrush);
            meters.push_back(m);
            m->setParentItem(this);
        }

        for(int i = -2; i < 3; ++i)
        {
            float posY = itemheight*float(i)/4.0f;
            QGraphicsLineItem * line = new QGraphicsLineItem(xstart,posY,length*0.5f,posY);
            QPen linePen;
            scaleLinePen.setStyle(Qt::DashLine);
            line->setPen(linePen);
            line->setParentItem(this);

            QGraphicsTextItem * scaleLabel = new QGraphicsTextItem(QString::number(-scale*float(i)/2.0f,'g',3));
            scaleLabel->setPos(xstart-scaleLabel->boundingRect().width(),
                               posY-scaleLabel->boundingRect().height()/2);
            scaleLabel->setParentItem(this);
        }
    }

    void modify(int jointIndex, float value)
    {
        float xstart = -length*0.5f;
        float freeSpace = length - meterwidth*numJoints;
        // Choose indentation such that it is 0.5 of the spacing between the meters
        float spacing = freeSpace / float(numJoints);
        float indentation = spacing/2;
        float meterXStart = xstart+jointIndex*(meterwidth+spacing) + indentation;
        float meterYStart;
        float meterHeight;
        if(value > 0.0f)
        {
            meterYStart = -(value/scale)*itemheight/2.0f;
            meterHeight = -meterYStart;
        }
        else if(value < 0.0f)
        {
            meterYStart = 0.0f;
            meterHeight = -(value/scale)*itemheight/2.0f;
        }
        else
        {
            meterYStart = 0.0f;
            meterHeight = 0.0f;
        }
        meters.at(jointIndex)->setRect(meterXStart,meterYStart,meterwidth,meterHeight);
    }

    QRectF boundingRect() const
    {
        return QRectF(-length*0.5f,
                      -itemheight*0.5f,
                      length,
                      itemheight);
    }

    void paint(QPainter* /*painter*/, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
    {
    }
};



#endif // GRAPHICSITEMS_H

