#ifndef ASTAR_H
#define ASTAR_H

#include <QObject>
#include "common.h"

class AStar : public QObject
{
    Q_OBJECT
public:
    explicit AStar(QObject *parent = nullptr);
    const Map &getMap() const {return map_;}
    void findPath();
    void findPath(const Point2D &start, const Point2D &stop);
    const std::vector<Point2D> &getPath() const {return path_;}

private:
    Map map_;

    Point2D start_;
    Point2D stop_;
    MapPoint startPath_;
    MapPoint stopPath_;
    std::vector<Point2D> path_;
    std::vector<MapPoint> mapPath_;

    struct AStarPoint
    {
        double weight;
        double pathWeight;
        std::vector<MapPoint> smallestPath;
        bool checked;
        uint8_t direction;
        MapPoint point;
        AStarPoint(MapPoint p) :
            point(p)
        {
            weight = std::numeric_limits<double>::max();
            checked = false;
        }
    };

    std::vector<AStarPoint> data_;

    void reduceMap();
    void dilateMap();
    MapPoint xyToMapPoint(Point2D p);
    MapPoint xyToMapPoint(float x, float y);
    void mapPointToxy(MapPoint point, float &x, float &y);
    Point2D mapPointToxy(MapPoint point);
    void algorithm();
    int isDataContains(const MapPoint &point) const;
    void checkPoint(const MapPoint &point, const AStarPoint &prev);

signals:

public slots:
    void setStart(float x, float y) {start_ = Point2D{x, y};}
    void setStop(float x, float y) {stop_ = Point2D{x, y};}
    void setMap(const Map &map);
};

#endif // ASTAR_H
