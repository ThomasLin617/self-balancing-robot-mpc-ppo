#pragma once

#include <navigation/occupancy_grid.hpp>

namespace navigation
{

class AStarPlanner
{
public:
    std::vector<PathPoint> plan(const OccupancyGrid& map, PathPoint start, PathPoint goal) const;
};

std::vector<PathPoint> simplify_path(const OccupancyGrid& map, const std::vector<PathPoint>& path);

}
