#include <navigation/timed_trajectory.hpp>
#include <robot_interface/angle.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navigation
{

void validate_motion_limits(const MotionLimits& limits) {
    for (double value : {limits.robot_radius, limits.safety_margin, limits.smoothing_clearance,
                        limits.max_velocity, limits.acceleration, limits.deceleration,
                        limits.lateral_acceleration, limits.max_yaw_rate, limits.yaw_acceleration,
                        limits.corner_distance, limits.sample_distance}) {
        if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("Motion limits must be finite and positive.");
    }
}

TimedTrajectory::TimedTrajectory(std::vector<TimedSample> samples)
    : samples_(std::move(samples))
{
    if (samples_.empty()) throw std::invalid_argument("Empty timed trajectory.");
    for (std::size_t i = 0; i < samples_.size(); ++i) {
        const auto& sample = samples_[i];
        const auto& p = sample.point;
        for (double value : {sample.time, p.x_ref, p.y_ref, p.yaw_ref, p.forward_position_ref,
                             p.forward_velocity_ref, p.yaw_rate_ref}) {
            if (!std::isfinite(value)) throw std::invalid_argument("Non-finite timed trajectory.");
        }
        if ((p.has_curvature && !std::isfinite(p.curvature)) || p.forward_velocity_ref < 0.0
            || (!p.has_curvature && p.forward_velocity_ref != 0.0)
            || (i == 0 && sample.time != 0.0)
            || (i > 0 && (sample.time <= samples_[i - 1].time
                || p.forward_position_ref < samples_[i - 1].point.forward_position_ref))) {
            throw std::invalid_argument("Invalid timed trajectory ordering or motion.");
        }
        if (i > 0) {
            const auto& previous = samples_[i - 1];
            const double dt = sample.time - previous.time;
            const double ds = p.forward_position_ref - previous.point.forward_position_ref;
            const double chord = std::hypot(p.x_ref - previous.point.x_ref, p.y_ref - previous.point.y_ref);
            if (std::abs(ds - chord) > 1e-8) throw std::invalid_argument("Arc length does not match the sampled path.");
            if (ds > 1e-12) {
                if (std::abs(ds - 0.5 * (p.forward_velocity_ref + previous.point.forward_velocity_ref) * dt) > 1e-8) {
                    throw std::invalid_argument("Moving interval timing is inconsistent.");
                }
            }
            else if (p.forward_velocity_ref != 0.0 || previous.point.forward_velocity_ref != 0.0
                     || std::abs(robot_interface::wrap_to_pi(p.yaw_ref - previous.point.yaw_ref)
                         - 0.5 * (p.yaw_rate_ref + previous.point.yaw_rate_ref) * dt) > 1e-8) {
                throw std::invalid_argument("Rotation interval timing is inconsistent.");
            }
        }
    }
    if (samples_.back().point.forward_velocity_ref != 0.0 || samples_.back().point.yaw_rate_ref != 0.0) {
        throw std::invalid_argument("Timed trajectory must finish at rest.");
    }
}

const std::vector<TimedSample>& TimedTrajectory::samples() const { return samples_; }
double TimedTrajectory::duration() const { return samples_.back().time; }

TrajectoryPoint TimedTrajectory::sample(double elapsed) const {
    if (!std::isfinite(elapsed)) throw std::invalid_argument("Invalid trajectory time.");
    if (elapsed <= 0.0) return samples_.front().point;
    if (elapsed >= duration()) return samples_.back().point;
    const auto upper = std::upper_bound(samples_.begin(), samples_.end(), elapsed,
        [](double time, const TimedSample& sample) { return time < sample.time; });
    const auto& a = *(upper - 1);
    const auto& b = *upper;
    const double dt = b.time - a.time;
    const double t = elapsed - a.time;
    const double ds = b.point.forward_position_ref - a.point.forward_position_ref;
    TrajectoryPoint point = a.point;
    if (ds > 1e-12) {
        const double acceleration = (b.point.forward_velocity_ref - a.point.forward_velocity_ref) / dt;
        const double distance = a.point.forward_velocity_ref * t + 0.5 * acceleration * t * t;
        const double ratio = std::clamp(distance / ds, 0.0, 1.0);
        point.x_ref += ratio * (b.point.x_ref - a.point.x_ref);
        point.y_ref += ratio * (b.point.y_ref - a.point.y_ref);
        point.yaw_ref = robot_interface::wrap_to_pi(a.point.yaw_ref + ratio
            * robot_interface::wrap_to_pi(b.point.yaw_ref - a.point.yaw_ref));
        point.forward_position_ref += distance;
        point.forward_velocity_ref += acceleration * t;
        point.has_curvature = true;
        point.curvature = a.point.curvature + ratio * (b.point.curvature - a.point.curvature);
        point.yaw_rate_ref = point.forward_velocity_ref * point.curvature;
    }
    else {
        const double acceleration = (b.point.yaw_rate_ref - a.point.yaw_rate_ref) / dt;
        point.yaw_ref = robot_interface::wrap_to_pi(a.point.yaw_ref
            + a.point.yaw_rate_ref * t + 0.5 * acceleration * t * t);
        point.forward_velocity_ref = 0.0;
        point.yaw_rate_ref = a.point.yaw_rate_ref + acceleration * t;
        point.has_curvature = false;
    }
    return point;
}

std::vector<TimedSample> profile_motion(std::vector<TrajectoryPoint> points, const MotionLimits& limits) {
    validate_motion_limits(limits);
    if (points.size() < 3) throw std::invalid_argument("Moving profile needs at least three samples.");
    const std::size_t count = points.size();
    std::vector<double> cap(count, limits.max_velocity);
    for (std::size_t i = 0; i < count; ++i) {
        const double k = std::abs(points[i].curvature);
        if (!points[i].has_curvature || !std::isfinite(k)) throw std::invalid_argument("Moving curvature is invalid.");
        if (k > 1e-10) cap[i] = std::min({cap[i], std::sqrt(limits.lateral_acceleration / k), limits.max_yaw_rate / k});
    }
    // Bound both ends of each interval by its largest endpoint curvature.
    const auto local_cap = cap;
    for (std::size_t i = 0; i < count; ++i) {
        if (i > 0) cap[i] = std::min(cap[i], local_cap[i - 1]);
        if (i + 1 < count) cap[i] = std::min(cap[i], local_cap[i + 1]);
    }
    points[0].forward_velocity_ref = 0.0;
    for (std::size_t i = 1; i < count; ++i) {
        const double ds = points[i].forward_position_ref - points[i - 1].forward_position_ref;
        if (!std::isfinite(ds) || ds <= 0.0) throw std::invalid_argument("Moving path distance must increase.");
        const double previous = points[i - 1].forward_velocity_ref;
        points[i].forward_velocity_ref = std::min(cap[i], std::sqrt(previous * previous + 2.0 * limits.acceleration * ds));
    }
    points.back().forward_velocity_ref = 0.0;
    for (std::size_t i = count - 1; i > 0; --i) {
        const double ds = points[i].forward_position_ref - points[i - 1].forward_position_ref;
        const double next = points[i].forward_velocity_ref;
        points[i - 1].forward_velocity_ref = std::min(points[i - 1].forward_velocity_ref,
            std::sqrt(next * next + 2.0 * limits.deceleration * ds));
    }
    std::vector<TimedSample> result;
    double time = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        if (i > 0) {
            const double speed_sum = points[i - 1].forward_velocity_ref + points[i].forward_velocity_ref;
            if (speed_sum <= 0.0) throw std::runtime_error("Positive distance has zero traversal speed.");
            time += 2.0 * (points[i].forward_position_ref - points[i - 1].forward_position_ref) / speed_sum;
        }
        points[i].yaw_rate_ref = points[i].forward_velocity_ref * points[i].curvature;
        result.push_back({points[i], time});
    }
    return result;
}

std::vector<TimedSample> profile_turn(const TrajectoryPoint& start, double final_yaw, const MotionLimits& limits) {
    validate_motion_limits(limits);
    const double delta = robot_interface::wrap_to_pi(final_yaw - start.yaw_ref);
    if (std::abs(delta) < 1e-8) return {{start, 0.0}};
    const double direction = delta > 0.0 ? 1.0 : -1.0;
    const double peak = std::min(limits.max_yaw_rate, std::sqrt(limits.yaw_acceleration * std::abs(delta)));
    const double ramp = peak / limits.yaw_acceleration;
    const double cruise = std::max(0.0, (std::abs(delta) - peak * ramp) / peak);
    std::vector<TimedSample> result;
    TrajectoryPoint point = start;
    point.has_curvature = false;
    point.forward_velocity_ref = 0.0;
    point.yaw_rate_ref = 0.0;
    result.push_back({point, 0.0});
    point.yaw_ref = robot_interface::wrap_to_pi(start.yaw_ref + direction * 0.5 * peak * ramp);
    point.yaw_rate_ref = direction * peak;
    result.push_back({point, ramp});
    if (cruise > 1e-10) {
        point.yaw_ref = robot_interface::wrap_to_pi(point.yaw_ref + direction * peak * cruise);
        result.push_back({point, ramp + cruise});
    }
    point.yaw_ref = robot_interface::wrap_to_pi(final_yaw);
    point.yaw_rate_ref = 0.0;
    result.push_back({point, 2.0 * ramp + cruise});
    return result;
}

}
