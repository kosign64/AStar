#include "odometrymap.h"
#include <QPainter>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QFile>
#include <QTextStream>
#include <QDebug>

OdometryMap::OdometryMap(QWidget *parent) : QWidget(parent),
    xPosition_(0),
    yPosition_(0),
    anglePosition_(0),
    scale_(26),
    translateX_(0),
    translateY_(0),
    mousePressed_(false),
    // Measured!
    ROBOT_WIDTH(0.65),
    ROBOT_LENGTH(1)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    startTimer(1000 / 50);
    setFocus();
    setFocusPolicy(Qt::StrongFocus);
}

void OdometryMap::timerEvent(QTimerEvent *)
{
    update();
}

void OdometryMap::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    scaleMeter_ = width() / scale_;
    QBrush brush(Qt::black);
    painter.setPen(Qt::NoPen);
    painter.setBrush(brush);
    painter.drawRect(0, 0, width(), height());
    painter.translate(width() / 2., height() / 2.);
    painter.translate(translateX_, translateY_);
    if(!map_.map.empty()) drawMap(painter, scaleMeter_);
    drawGrid(painter, scaleMeter_);
    //drawRobot(painter, scaleMeter);
    if(!laserRanges_.empty()) drawLaser(painter, scaleMeter_);
    if(!points_.empty()) drawPoints(painter, scaleMeter_);
}

void OdometryMap::wheelEvent(QWheelEvent *event)
{
    scale_ += event->angleDelta().y() / 50.;
    if(scale_ < 1) scale_ = 1;
}

void OdometryMap::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        mousePressed_ = true;
        mouseStart_ = event->pos();
    }
    else if(event->button() == Qt::RightButton)
    {
        Point2D p;
        p.x = (event->pos().x() - width() / 2 - translateX_) / scaleMeter_;
        p.y = -(event->pos().y() - height() / 2 - translateY_) / scaleMeter_;

        Q_EMIT pressedPoint(p);
    }
}

void OdometryMap::mouseMoveEvent(QMouseEvent *event)
{
    if(mousePressed_)
    {
        QPoint delta = event->pos() - mouseStart_;
        translateX_ += delta.x();
        translateY_ += delta.y();
        mouseStart_ = event->pos();
    }
}

void OdometryMap::mouseReleaseEvent(QMouseEvent *)
{
    mousePressed_ = false;
}

void OdometryMap::drawGrid(QPainter &painter, const double scaleMeter)
{
    QPen pen(Qt::white);
    painter.setPen(pen);

    for(double x = 0; x < (width() * 2.); x += scaleMeter)
    {
        painter.drawLine(x, (-height() * 2.), x, (height() * 2.));
    }
    for(double x = 0; x > (-width() * 2.); x -= scaleMeter)
    {
        painter.drawLine(x, (-height() * 2.), x, (height() * 2.));
    }

    for(double y = 0; y < (height() * 2.); y += scaleMeter)
    {
        painter.drawLine((-width() * 2.), y, (width() * 2.), y);
    }
    for(double y = 0; y > (-height() * 2.); y -= scaleMeter)
    {
        painter.drawLine((-width() * 2.), y, (width() * 2.), y);
    }
}

void OdometryMap::drawMap(QPainter &painter, const double scaleMeter)
{
    painter.save();
    const double mapScale = map_.resolution * scaleMeter;
    QBrush brush(Qt::lightGray);
    painter.setPen(Qt::NoPen);
    painter.translate(map_.x * scaleMeter, -map_.y * scaleMeter);
    //painter.rotate(-mapAngle_ * 180. / M_PI);
    for(int x = 0; x < map_.width; ++x)
    {
        for(int y = 0; y < map_.height; ++y)
        {
            int8_t occupacy = map_.map[x + y * map_.width];
            if(occupacy == -1)
            {
                continue;
            }
            else
            {
                if(occupacy == 100)
                {
                    brush.setColor(Qt::darkBlue);
                }
                else if(occupacy == 50)
                {
                    brush.setColor(Qt::green);
                }
                else
                {
                    brush.setColor(Qt::lightGray);
                }
            }
            painter.setBrush(brush);
            painter.drawRect(
                        QRectF((x * map_.resolution -
                                map_.resolution / 2) *
                               scaleMeter,
                               -(y * map_.resolution -
                                map_.resolution / 2) *
                               scaleMeter,
                               mapScale,
                               mapScale));
        }
    }
    painter.restore();
}

void OdometryMap::drawRobot(QPainter &painter, const double scaleMeter)
{
    painter.save();
    QPen pen(Qt::green);
    QBrush brush(Qt::gray);
    // X - forward moving
    painter.translate(yPosition_  * scaleMeter,
                      -xPosition_  * scaleMeter);
    painter.rotate(anglePosition_ * 180 / M_PI);
    brush.setColor(Qt::gray);
    pen.setColor(Qt::green);
    pen.setWidth(3);
    painter.setBrush(brush);
    painter.setPen(pen);
    const double robotWidth = ROBOT_WIDTH * scaleMeter;
    const double robotLength = ROBOT_LENGTH * scaleMeter;
    painter.drawRect(QRectF(-robotWidth / 2, -robotLength / 2,
                     robotWidth, robotLength));
    brush.setColor(Qt::black);
    painter.setPen(Qt::NoPen);
    painter.setBrush(brush);
    painter.drawRect(-robotWidth / 4, -robotLength / 2,
                     robotWidth / 2, robotWidth / 2);
    painter.restore();
}

void OdometryMap::drawLaser(QPainter &painter, const double scaleMeter)
{
    painter.save();
    QBrush brush(Qt::green);
    painter.setBrush(brush);
    painter.setPen(Qt::NoPen);
    painter.translate(yPosition_  * scaleMeter,
                      -xPosition_  * scaleMeter);
    painter.rotate(anglePosition_ * 180 / M_PI);
    painter.translate(0, -ROBOT_LENGTH * scaleMeter / 2.);
    painter.drawEllipse(QPointF(0, 0), 0.2 * scaleMeter,
                        0.2 * scaleMeter);
    painter.rotate(-laserAngleMin_ * 180. / M_PI);
    const double angleStep = laserAngleIncrement_ * 180. / M_PI;
    brush.setColor(Qt::red);
    painter.setBrush(brush);
    for(const auto &range : laserRanges_)
    {
        if(range > 0 && range < 100)
        {
            painter.drawEllipse(QPointF(0, -range * scaleMeter),
                                0.05 * scaleMeter, 0.05 * scaleMeter);
        }
        painter.rotate(-angleStep);
    }
    painter.restore();
}

void OdometryMap::drawPoints(QPainter &painter, const double scaleMeter)
{
    painter.save();
    painter.setBrush(Qt::red);
    painter.setPen(Qt::NoPen);
    for(Point2D &point : points_)
    {
        painter.drawEllipse(QPointF(point.x * scaleMeter,
                                    -point.y * scaleMeter),
                            0.05 * scaleMeter,
                            0.05 * scaleMeter);
    }
    painter.setBrush(Qt::green);
    Point2D &end = points_.back();
    painter.drawEllipse(QPointF(points_[0].x * scaleMeter,
                                -points_[0].y * scaleMeter),
                        0.05 * scaleMeter,
                        0.05 * scaleMeter);
    painter.drawEllipse(QPointF(end.x * scaleMeter,
                                -end.y * scaleMeter),
                        0.05 * scaleMeter,
                        0.05 * scaleMeter);
    painter.restore();
}

void OdometryMap::setRobotPosition(double x, double y, double angle)
{
    xPosition_ = x;
    yPosition_ = y;
    anglePosition_ = angle;
}

void OdometryMap::setLaser(float angleMin, float angleIncrement,
                           const vector<float> ranges)
{
    laserAngleMin_ = angleMin;
    laserAngleIncrement_ = angleIncrement;
    laserRanges_ = ranges;
}

void OdometryMap::setMap(float xOrigin, float yOrigin, float angle,
                         int32_t width, int32_t height,
                         float resolution, const vector<int8_t> map)
{
    map_.map = map;
    map_.x = xOrigin;
    map_.y = yOrigin;
    //mapAngle_ = angle;
    map_.width = width;
    map_.height = height;
    map_.resolution = resolution;
}

void OdometryMap::setMapStruct(const Map &map)
{
    map_ = map;
}

