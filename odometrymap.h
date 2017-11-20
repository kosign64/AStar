#ifndef ODOMETRYMAP_H
#define ODOMETRYMAP_H

#include <QWidget>
#include <vector>
#include "common.h"

using namespace std;

class OdometryMap : public QWidget
{
    Q_OBJECT
public:
    explicit OdometryMap(QWidget *parent = nullptr);
    QSize sizeHint() const {return QSize(1200, 600);}
    void clearPoints() {points_.clear();}
    void addPoint(Point2D point) {points_.push_back(point);}

protected:
    void timerEvent(QTimerEvent *);
    void paintEvent(QPaintEvent *);
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *);

private:
    double xPosition_;
    double yPosition_;
    double anglePosition_;

    float laserAngleMin_;
    float laserAngleIncrement_;
    vector<float> laserRanges_;

    vector<int8_t> map_;
    float mapResolution_;
    int32_t mapWidth_;
    int32_t mapHeight_;
    float mapX_;
    float mapY_;
    float mapAngle_;

    double scale_;
    double translateX_;
    double translateY_;

    bool mousePressed_;
    QPoint mouseStart_;

    vector<Point2D> points_;

    const double ROBOT_WIDTH;
    const double ROBOT_LENGTH;

    void drawGrid(QPainter &painter, const double scaleMeter);
    void drawMap(QPainter &painter, const double scaleMeter);
    void drawRobot(QPainter &painter, const double scaleMeter);
    void drawLaser(QPainter &painter, const double scaleMeter);
    void drawPoints(QPainter &painter, const double scaleMeter);

Q_SIGNALS:

public Q_SLOTS:
    void setRobotPosition(double x, double y, double angle);
    void setLaser(float angleMin, float angleIncrement,
                  const vector<float> ranges);
    void setMap(float xOrigin, float yOrigin, float angle, int32_t width,
                int32_t height, float resolution,
                const vector<int8_t> map);
    void setMapStruct(const Map &map);
};

#endif // ODOMETRYMAP_H
