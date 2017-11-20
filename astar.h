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
    const std::vector<Point2D> &getPath() const {return path_;}

private:
    Map map_;

    struct MapPoint
    {
        int x;
        int y;
    };

    Point2D start_;
    Point2D stop_;
    MapPoint startPath_;
    MapPoint stopPath_;
    std::vector<Point2D> path_;
    std::vector<MapPoint> mapPath_;

    struct StarPoint
    {
        double weight;
        std::vector<MapPoint> smallestPath;
        bool checked;
        uint8_t direction;
        MapPoint point;
        StarPoint(MapPoint p) :
            point(p)
        {
            weight = std::numeric_limits<double>::max();
            checked = false;
        }
    };

    std::vector<StarPoint> data_;

    void reduceMap();
    MapPoint xyToMapPoint(Point2D p);
    MapPoint xyToMapPoint(float x, float y);
    void mapPointToxy(MapPoint point, float &x, float &y);
    Point2D mapPointToxy(MapPoint point);
    double mapDistance(MapPoint p1, MapPoint p2);

    void algorithm();

    int isDataContains(const MapPoint &point) const;
    void checkPoint(const MapPoint &point, const StarPoint &prev);

signals:

public slots:
    void setStart(float x, float y) {start_ = Point2D{x, y};}
    void setStop(float x, float y) {stop_ = Point2D{x, y};}
    void setMap(const Map &map) {map_ = map;}
};

#endif // ASTAR_H
