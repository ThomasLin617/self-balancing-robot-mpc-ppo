#pragma once

#include <string>
#include <navigation/occupancy_grid.hpp>

namespace navigation
{

struct MapObstacle
{
    std::string name;
    double min_x;
    double min_y;
    double max_x;
    double max_y;
    double height;
};

struct KnownMap
{
    OccupancyGrid grid;
    PathPoint goal;
    std::vector<MapObstacle> obstacles;
    static KnownMap load(const std::string& path);
};

}
