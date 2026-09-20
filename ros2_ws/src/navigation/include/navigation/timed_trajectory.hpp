#pragma once

#include <navigation/reference_trajectory.hpp>

namespace navigation
{

struct MotionLimits
{
    double robot_radius = 0.27;
    double safety_margin = 0.15;
    double smoothing_clearance = 0.10;
    double max_velocity = 0.16;
    double acceleration = 0.06;
    double deceleration = 0.06;
    double lateral_acceleration = 0.025;
    double max_yaw_rate = 0.45;
    double yaw_acceleration = 0.35;
    double corner_distance = 0.25;
    double sample_distance = 0.01;
};

struct TimedSample
{
    TrajectoryPoint point;
    double time = 0.0;
};

class TimedTrajectory
{
public:
    explicit TimedTrajectory(std::vector<TimedSample> samples);
    TrajectoryPoint sample(double elapsed) const;
    double duration() const;
    const std::vector<TimedSample>& samples() const;

private:
    std::vector<TimedSample> samples_;
};

void validate_motion_limits(const MotionLimits& limits);
std::vector<TimedSample> profile_motion(std::vector<TrajectoryPoint> points, const MotionLimits& limits);
std::vector<TimedSample> profile_turn(const TrajectoryPoint& start, double final_yaw, const MotionLimits& limits);

}
