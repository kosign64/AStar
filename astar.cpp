#include "astar.h"
#include <QDebug>

using namespace std;

double mapDistance(MapPoint p1, MapPoint p2);

AStar::AStar(QObject *parent) : QObject(parent),
    start_{-1, -1},
    stop_{-1, -1},
    reduced_(false)
{

}

void AStar::findPath()
{
    if(map_.map.empty()) return;
    if(!reduced_) reduceMap();
    if(start_.x < 0 || stop_.y < 0) return;
    startPath_ = xyToMapPoint(start_);
    stopPath_ = xyToMapPoint(stop_);
    if(startPath_.x == stopPath_.x &&
            startPath_.y == stopPath_.y) return;
    if(map_(stopPath_.x, stopPath_.y) == 100) return;
    data_.clear();
    AStarPoint startPoint(startPath_);
    startPoint.checked = true;
    data_.push_back(startPoint);
    int index = -1;
    while(index == -1)
    {
        algorithm();
        index = isDataContains(stopPath_);
    }
    path_.clear();
    for(const auto &p : data_[index].smallestPath)
    {
        map_(p.x, p.y) = 50;
        path_.push_back(mapPointToxy(p));
    }
}

void AStar::findPath(const Point2D &start, const Point2D &stop)
{
    setStart(start.x, start.y);
    setStop(stop.x, stop.y);
    findPath();
}

void AStar::reduceMap()
{
    int reduceFactor = 4;
    float resolution = map_.resolution * reduceFactor;
    int width = map_.width / reduceFactor;
    int height = map_.height / reduceFactor;
    vector<int8_t> mapData;
    for(int y = 0; y < (map_.height - (reduceFactor - 1)); y +=
        reduceFactor)
    {
        for(int x = 0; x < (map_.width - (reduceFactor - 1)); x +=
            reduceFactor)
        {
            int8_t maxValue = -1;
            bool foundMax = false;
            for(int i = 0; i < reduceFactor && !foundMax; ++i)
            {
                for(int j = 0; j < reduceFactor && !foundMax; ++j)
                {
                    if(map_(x + i, y + j) > maxValue)
                    {
                        maxValue = map_(x + i, y + j);
                        if(maxValue == 100) foundMax = true;
                    }
                }
            }
            mapData.push_back(maxValue);
        }
    }
    map_.map = std::move(mapData);
    map_.width = width;
    map_.height = height;
    map_.resolution = resolution;
    reduced_ = true;
}

MapPoint AStar::xyToMapPoint(Point2D p)
{
    return xyToMapPoint(p.x, p.y);
}

MapPoint AStar::xyToMapPoint(float x, float y)
{
    MapPoint point;
    x -= map_.x;
    y -= map_.y;

    point.x = round(x / map_.resolution);
    point.y = round(y / map_.resolution);

    return point;
}

void AStar::mapPointToxy(MapPoint point, float &x, float &y)
{
    x = point.x * map_.resolution + map_.x;
    y = point.y * map_.resolution + map_.y;
}

Point2D AStar::mapPointToxy(MapPoint point)
{
    Point2D res;
    mapPointToxy(point, res.x, res.y);

    return res;
}

double mapDistance(MapPoint p1, MapPoint p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void AStar::algorithm()
{
    std::sort(data_.begin(), data_.end(),
              [](const AStarPoint &a, const AStarPoint &b) -> bool
                {
                    int aChecked = (a.checked) ? 100000 : 1;
                    int bChecked = (b.checked) ? 100000 : 1;
                    return (a.weight * aChecked) < (b.weight * bChecked);
                });
    AStarPoint best = data_[0];
    const MapPoint points[] =
    {{best.point.x - 1, best.point.y - 1},
     {best.point.x - 1, best.point.y    },
     {best.point.x - 1, best.point.y + 1},
     {best.point.x    , best.point.y + 1},
     {best.point.x + 1, best.point.y + 1},
     {best.point.x + 1, best.point.y    },
     {best.point.x + 1, best.point.y - 1},
     {best.point.x    , best.point.y - 1}};
    for(size_t i = 0; i < sizeof(points) / sizeof(points[0]); ++i)
    {
        checkPoint(points[i], best);
    }
    best.checked = true;
}

int AStar::isDataContains(const MapPoint &point) const
{
    for(size_t i = 0; i < data_.size(); ++i)
    {
        const AStarPoint &p = data_[i];
        if((p.point.x == point.x) && (p.point.y == point.y))
        {
            return i;
        }
    }

    return -1;
}

void AStar::checkPoint(const MapPoint &point,
                       const AStar::AStarPoint &prev)
{
    if(map_(point.x, point.y) == 100) return;
    if(point.x < 0 || point.x >= map_.width ||
            point.y < 0 || point.y >= map_.height) return;
    AStarPoint starPoint(point);
    starPoint.weight = mapDistance(point, stopPath_) +
            mapDistance(point, prev.point);
    int index = isDataContains(point);
    if(index != -1)
    {
        if(data_[index].weight > starPoint.weight)
        {
            starPoint.smallestPath = prev.smallestPath;
            starPoint.smallestPath.push_back(prev.point);
            data_[index] = starPoint;
        }
        return;
    }
    starPoint.smallestPath = prev.smallestPath;
    starPoint.smallestPath.push_back(prev.point);
    data_.push_back(starPoint);
}
