#pragma once

#include <navigation/global_planner.hpp>
#include <navigation/timed_trajectory.hpp>

namespace navigation
{

struct PathSegment
{
    std::vector<TrajectoryPoint> points;
    bool rotation = false;
};

std::vector<PathSegment> smooth_path(
    const OccupancyGrid& map, const std::vector<PathPoint>& waypoints,
    double start_yaw, const MotionLimits& limits
);

TimedTrajectory time_parameterize(const std::vector<PathSegment>& segments, const MotionLimits& limits);
void validate_trajectory(const OccupancyGrid& map, const TimedTrajectory& trajectory);

}
