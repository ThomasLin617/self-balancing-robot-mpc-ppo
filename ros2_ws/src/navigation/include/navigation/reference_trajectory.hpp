#pragma once

#include <vector>
#include <memory>
#include <navigation/planar_odometry.hpp>

namespace navigation
{

struct TrajectoryPoint
{
    double x_ref = 0.0;
    double y_ref = 0.0;
    double yaw_ref = 0.0;
    double forward_position_ref = 0.0;
    double forward_velocity_ref = 0.0;
    double yaw_rate_ref = 0.0;
    double curvature = 0.0;
    bool has_curvature = true;
};

class TimedTrajectory;

class ReferenceTrajectory
{
public:
    ReferenceTrajectory(double max_velocity, double acceleration);
    explicit ReferenceTrajectory(const TimedTrajectory& trajectory);
    void generate(const PlanarPose& start, double goal_x, double goal_y);
    TrajectoryPoint sample(double elapsed) const;
    const std::vector<TrajectoryPoint>& points() const;
    double duration() const;

private:
    std::vector<TrajectoryPoint> points_;
    std::shared_ptr<const TimedTrajectory> timed_;
    double max_velocity_;
    double acceleration_;
    double peak_velocity_ = 0.0;
    double acceleration_time_ = 0.0;
    double cruise_time_ = 0.0;
    double length_ = 0.0;
};

}
