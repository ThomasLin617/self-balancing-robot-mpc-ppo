#include <navigation/reference_trajectory.hpp>
#include <navigation/timed_trajectory.hpp>
#include <robot_interface/angle.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navigation
{

ReferenceTrajectory::ReferenceTrajectory(double max_velocity, double acceleration)
    : max_velocity_(max_velocity), acceleration_(acceleration)
{
    if (!std::isfinite(max_velocity) || !std::isfinite(acceleration)
        || max_velocity <= 0.0 || acceleration <= 0.0) {
        throw std::invalid_argument("Trajectory velocity and acceleration must be positive.");
    }
}

ReferenceTrajectory::ReferenceTrajectory(const TimedTrajectory& trajectory)
    : timed_(std::make_shared<TimedTrajectory>(trajectory)),
      max_velocity_(0.0), acceleration_(0.0)
{
    for (const auto& sample : trajectory.samples()) points_.push_back(sample.point);
}

void ReferenceTrajectory::generate(const PlanarPose& start, double goal_x, double goal_y) {
    if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.yaw)
        || !std::isfinite(goal_x) || !std::isfinite(goal_y)) {
        throw std::invalid_argument("Invalid trajectory pose or goal.");
    }
    if (timed_) throw std::logic_error("Construct a new timed trajectory to replace a planned path.");
    points_.clear();
    length_ = 0.0;
    acceleration_time_ = 0.0;
    cruise_time_ = 0.0;
    peak_velocity_ = 0.0;
    const double dx = goal_x - start.x;
    const double dy = goal_y - start.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 1e-6) {
        TrajectoryPoint point;
        point.x_ref = start.x;
        point.y_ref = start.y;
        point.yaw_ref = robot_interface::wrap_to_pi(start.yaw);
        points_.push_back(point);
        return;
    }

    const double tangent_x = distance * std::cos(start.yaw);
    const double tangent_y = distance * std::sin(start.yaw);
    constexpr int segments = 200;
    for (int i = 0; i <= segments; ++i) {
        const double u = static_cast<double>(i) / segments;
        const double u2 = u * u;
        const double u3 = u2 * u;
        const double h10 = u3 - 2.0 * u2 + u;
        const double h01 = -2.0 * u3 + 3.0 * u2;
        const double h11 = u3 - u2;
        TrajectoryPoint point;
        point.x_ref = start.x + h10 * tangent_x + (h01 + h11) * dx;
        point.y_ref = start.y + h10 * tangent_y + (h01 + h11) * dy;
        const double derivative_x = (3.0 * u2 - 4.0 * u + 1.0) * tangent_x
                                  + (-3.0 * u2 + 4.0 * u) * dx;
        const double derivative_y = (3.0 * u2 - 4.0 * u + 1.0) * tangent_y
                                  + (-3.0 * u2 + 4.0 * u) * dy;
        const double second_x = (6.0 * u - 4.0) * tangent_x + (4.0 - 6.0 * u) * dx;
        const double second_y = (6.0 * u - 4.0) * tangent_y + (4.0 - 6.0 * u) * dy;
        const double tangent_norm = std::hypot(derivative_x, derivative_y);
        if (tangent_norm < 1e-8) {
            throw std::runtime_error("Trajectory has a stationary tangent.");
        }
        point.yaw_ref = std::atan2(derivative_y, derivative_x);
        point.curvature = (derivative_x * second_y - derivative_y * second_x)
                        / (tangent_norm * tangent_norm * tangent_norm);
        if (!points_.empty()) {
            length_ += std::hypot(point.x_ref - points_.back().x_ref,
                                  point.y_ref - points_.back().y_ref);
        }
        point.forward_position_ref = length_;
        points_.push_back(point);
    }
    peak_velocity_ = std::min(max_velocity_, std::sqrt(acceleration_ * length_));
    acceleration_time_ = peak_velocity_ / acceleration_;
    cruise_time_ = std::max(0.0, (length_ - peak_velocity_ * acceleration_time_) / peak_velocity_);
}

TrajectoryPoint ReferenceTrajectory::sample(double elapsed) const {
    if (timed_) return timed_->sample(elapsed);
    if (points_.empty() || !std::isfinite(elapsed)) {
        throw std::runtime_error("Cannot sample an empty trajectory or invalid time.");
    }
    if (elapsed <= 0.0 || length_ == 0.0) return points_.front();
    if (elapsed >= duration()) return points_.back();

    double distance;
    double velocity;
    if (elapsed < acceleration_time_) {
        velocity = acceleration_ * elapsed;
        distance = 0.5 * acceleration_ * elapsed * elapsed;
    }
    else if (elapsed < acceleration_time_ + cruise_time_) {
        velocity = peak_velocity_;
        distance = 0.5 * peak_velocity_ * acceleration_time_
                 + peak_velocity_ * (elapsed - acceleration_time_);
    }
    else {
        const double remaining = duration() - elapsed;
        velocity = acceleration_ * remaining;
        distance = length_ - 0.5 * acceleration_ * remaining * remaining;
    }

    const auto upper = std::lower_bound(points_.begin(), points_.end(), distance,
        [](const TrajectoryPoint& point, double s) {
            return point.forward_position_ref < s;
        });
    if (upper == points_.begin()) return points_.front();
    if (upper == points_.end()) return points_.back();
    const auto& lower = *(upper - 1);
    const double ratio = (distance - lower.forward_position_ref)
                       / (upper->forward_position_ref - lower.forward_position_ref);
    TrajectoryPoint point;
    point.x_ref = lower.x_ref + ratio * (upper->x_ref - lower.x_ref);
    point.y_ref = lower.y_ref + ratio * (upper->y_ref - lower.y_ref);
    point.yaw_ref = robot_interface::wrap_to_pi(lower.yaw_ref + ratio
        * robot_interface::wrap_to_pi(upper->yaw_ref - lower.yaw_ref));
    point.curvature = lower.curvature + ratio * (upper->curvature - lower.curvature);
    point.forward_position_ref = distance;
    point.forward_velocity_ref = velocity;
    point.yaw_rate_ref = velocity * point.curvature;
    return point;
}

const std::vector<TrajectoryPoint>& ReferenceTrajectory::points() const {
    return points_;
}

double ReferenceTrajectory::duration() const {
    if (timed_) return timed_->duration();
    return 2.0 * acceleration_time_ + cruise_time_;
}

}
