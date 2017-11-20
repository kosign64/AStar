#include "astar.h"
#include <QDebug>

using namespace std;

AStar::AStar(QObject *parent) : QObject(parent)
{

}

void AStar::findPath()
{
    if(map_.map.empty()) return;
    reduceMap();
    startPath_ = xyToMapPoint(start_);
    stopPath_ = xyToMapPoint(stop_);
    data_.clear();
    StarPoint point(startPath_);
    point.checked = true;
    data_.push_back(point);
    int index = -1;
    while(index == -1)
    {
        algorithm();
        index = isDataContains(stopPath_);
    }
    path_.clear();
    for(auto &p : data_[index].smallestPath)
    {
        map_(p.x, p.y) = 50;
        path_.push_back(mapPointToxy(p));
    }
}

void AStar::reduceMap()
{
    int reduceFactor = 4;
    float resolution = map_.resolution * reduceFactor;
    int width = map_.width / reduceFactor;
    int height = map_.height / reduceFactor;
    vector<int8_t> mapData;
    qDebug() << start_.x << start_.y;
    for(int y = 0; y < (map_.height - (reduceFactor - 1)); y +=
        reduceFactor)
    {
        for(int x = 0; x < (map_.width - (reduceFactor - 1)); x +=
            reduceFactor)
        {
            int8_t maxValue = -1;
            for(int i = 0; i < reduceFactor; ++i)
            {
                for(int j = 0; j < reduceFactor; ++j)
                {
                    if(map_(x + i, y + j) > maxValue)
                    {
                        maxValue = map_(x + i, y + j);

                    }
                }
            }
            if((x / reduceFactor) == start_.x &&
                    (y / reduceFactor) == start_.y) maxValue = 50;
            if((x / reduceFactor) == stop_.x &&
                    (y / reduceFactor) == stop_.y) maxValue = 50;
            mapData.push_back(maxValue);
        }
    }
    map_.map = mapData;
    map_.width = width;
    map_.height = height;
    map_.resolution = resolution;
}

AStar::MapPoint AStar::xyToMapPoint(Point2D p)
{
    return xyToMapPoint(p.x, p.y);
}

AStar::MapPoint AStar::xyToMapPoint(float x, float y)
{
    MapPoint point;
    x -= map_.x;
    y -= map_.y;

    point.x = round(x / map_.resolution);
    point.y = round(y / map_.resolution);

    return point;
}

void AStar::mapPointToxy(AStar::MapPoint point, float &x, float &y)
{
    x = point.x * map_.resolution + map_.x;
    y = point.y * map_.resolution + map_.y;
}

Point2D AStar::mapPointToxy(AStar::MapPoint point)
{
    Point2D res;
    mapPointToxy(point, res.x, res.y);

    return res;
}

double AStar::mapDistance(AStar::MapPoint p1, AStar::MapPoint p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void AStar::algorithm()
{
    std::sort(data_.begin(), data_.end(),
              [](const StarPoint &a, const StarPoint &b) -> bool
                {
                    int aChecked = (a.checked) ? 100000 : 1;
                    int bChecked = (b.checked) ? 100000 : 1;
                    return (a.weight * aChecked) < (b.weight * bChecked);
                });
    StarPoint best = data_[0];
    if(best.point.x > 0)
    {
        MapPoint p{best.point.x - 1, best.point.y};
        checkPoint(p, best);
        if(best.point.y > 0)
        {
            p.y = best.point.y - 1;
            checkPoint(p, best);
        }
        if(best.point.y < (map_.height - 1))
        {
            p.y = best.point.y + 1;
            checkPoint(p, best);
        }
    }
    if(best.point.x < (map_.width - 1))
    {
        MapPoint p{best.point.x + 1, best.point.y};
        checkPoint(p, best);
        if(best.point.y > 0)
        {
            p.y = best.point.y - 1;
            checkPoint(p, best);
        }
        if(best.point.y < (map_.height - 1))
        {
            p.y = best.point.y + 1;
            checkPoint(p, best);
        }
    }
    if(best.point.y > 0)
    {
        MapPoint p{best.point.x, best.point.y - 1};
        checkPoint(p, best);
        if(best.point.x > 0)
        {
            p.x = best.point.x - 1;
            checkPoint(p, best);
        }
        if(best.point.x < (map_.width - 1))
        {
            p.x = best.point.x + 1;
            checkPoint(p, best);
        }
    }
    if(best.point.y < (map_.height - 1))
    {
        MapPoint p{best.point.x, best.point.y + 1};
        checkPoint(p, best);
        if(best.point.x > 0)
        {
            p.x = best.point.x - 1;
            checkPoint(p, best);
        }
        if(best.point.x < (map_.width - 1))
        {
            p.x = best.point.x + 1;
            checkPoint(p, best);
        }
    }
    best.checked = true;
}

int AStar::isDataContains(const AStar::MapPoint &point) const
{
    for(size_t i = 0; i < data_.size(); ++i)
    {
        const StarPoint &p = data_[i];
        if((p.point.x == point.x) && (p.point.y == point.y))
        {
            return i;
        }
    }

    return -1;
}

void AStar::checkPoint(const AStar::MapPoint &point,
                       const AStar::StarPoint &prev)
{
    if(map_(point.x, point.y) == 100) return;
    StarPoint starPoint(point);
    starPoint.weight = mapDistance(point, stopPath_) +
            mapDistance(point, prev.point);
    starPoint.smallestPath = prev.smallestPath;
    starPoint.smallestPath.push_back(prev.point);
    int index = isDataContains(point);
    if(index == -1)
    {
        data_.push_back(starPoint);
    }
    else
    {
        if(data_[index].weight > starPoint.weight)
        {
            data_[index] = starPoint;
        }
    }
}
