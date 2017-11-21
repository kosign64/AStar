#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <stdint-gcc.h>

struct MapPoint
{
    int x;
    int y;
};

struct Point2D
{
    float x;
    float y;
};

struct Map
{
    float resolution;
    float x;
    float y;
    int width;
    int height;
    std::vector<int8_t> map;

    Map(float mapResolution, float mapX, float mapY, int mapWidth,
        int mapHeight, const std::vector<int8_t> &mapData) :
        resolution(mapResolution),
        x(mapX),
        y(mapY),
        width(mapWidth),
        height(mapHeight),
        map(mapData)
    {}
    Map()
    {}
    int8_t &operator()(int x, int y)
    {
        return map[x + y * width];
    }

    const int8_t &operator()(int x, int y) const
    {
        return map[x + y * width];
    }

    int8_t &operator()(const MapPoint &p)
    {
        return map[p.x + p.y * width];
    }

    const int8_t &operator()(const MapPoint &p) const
    {
        return map[p.x + p.y * width];
    }
};

#endif // COMMON_H
