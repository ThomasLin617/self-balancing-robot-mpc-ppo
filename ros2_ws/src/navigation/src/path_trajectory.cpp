#include <navigation/path_trajectory.hpp>
#include <robot_interface/angle.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navigation
{
namespace {

double distance(PathPoint a, PathPoint b) {
    return std::hypot(b[0] - a[0], b[1] - a[1]);
}

TrajectoryPoint make_point(PathPoint p, double yaw, double curvature = 0.0) {
    TrajectoryPoint result;
    result.x_ref = p[0];
    result.y_ref = p[1];
    result.yaw_ref = robot_interface::wrap_to_pi(yaw);
    result.curvature = curvature;
    return result;
}

void append_point(std::vector<TrajectoryPoint>& path, TrajectoryPoint point) {
    if (!path.empty()) {
        const double ds = std::hypot(point.x_ref - path.back().x_ref, point.y_ref - path.back().y_ref);
        point.forward_position_ref = path.back().forward_position_ref + ds;
        if (ds < 1e-12) {
            point.forward_position_ref = path.back().forward_position_ref;
            path.back() = point;
            return;
        }
    }
    path.push_back(point);
}

void append_line(std::vector<TrajectoryPoint>& path, PathPoint from, PathPoint to, double spacing) {
    const double length = distance(from, to);
    if (length < 1e-10) return;
    const int count = std::max(2, static_cast<int>(std::ceil(length / spacing)));
    const double yaw = std::atan2(to[1] - from[1], to[0] - from[0]);
    for (int i = 0; i <= count; ++i) {
        const double u = static_cast<double>(i) / count;
        append_point(path, make_point({from[0] + u * (to[0] - from[0]), from[1] + u * (to[1] - from[1])}, yaw));
    }
}

std::vector<TrajectoryPoint> corner_curve(PathPoint before, PathPoint corner, PathPoint after, double cut, double spacing) {
    const double incoming = distance(before, corner);
    const double outgoing = distance(corner, after);
    PathPoint p[4];
    for (int axis = 0; axis < 2; ++axis) {
        const double a = (corner[axis] - before[axis]) / incoming;
        const double b = (after[axis] - corner[axis]) / outgoing;
        p[0][axis] = corner[axis] - cut * a;
        p[1][axis] = corner[axis] - 0.45 * cut * a;
        p[2][axis] = corner[axis] + 0.45 * cut * b;
        p[3][axis] = corner[axis] + cut * b;
    }
    const int count = std::max(8, static_cast<int>(std::ceil(2.0 * cut / spacing)));
    std::vector<TrajectoryPoint> result;
    for (int i = 0; i <= count; ++i) {
        const double u = static_cast<double>(i) / count;
        const double v = 1.0 - u;
        PathPoint position{}, first{}, second{};
        for (int axis = 0; axis < 2; ++axis) {
            position[axis] = v*v*v*p[0][axis] + 3*v*v*u*p[1][axis] + 3*v*u*u*p[2][axis] + u*u*u*p[3][axis];
            first[axis] = 3*v*v*(p[1][axis]-p[0][axis]) + 6*v*u*(p[2][axis]-p[1][axis]) + 3*u*u*(p[3][axis]-p[2][axis]);
            second[axis] = 6*v*(p[2][axis]-2*p[1][axis]+p[0][axis]) + 6*u*(p[3][axis]-2*p[2][axis]+p[1][axis]);
        }
        const double norm = std::hypot(first[0], first[1]);
        if (norm < 1e-10) return {};
        const double k = (first[0]*second[1] - first[1]*second[0]) / (norm*norm*norm);
        result.push_back(make_point(position, std::atan2(first[1], first[0]), k));
    }
    return result;
}

bool collision_free(const OccupancyGrid& map, const std::vector<TrajectoryPoint>& points) {
    if (points.empty()) return false;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        const auto& previous = points[i == 0 ? 0 : i - 1];
        if (!std::isfinite(p.yaw_ref) || (p.has_curvature && !std::isfinite(p.curvature))
            || !map.segment_free({previous.x_ref, previous.y_ref}, {p.x_ref, p.y_ref})) return false;
    }
    return true;
}

}

std::vector<PathSegment> smooth_path(
    const OccupancyGrid& map, const std::vector<PathPoint>& waypoints,
    double start_yaw, const MotionLimits& limits
) {
    validate_motion_limits(limits);
    if (!std::isfinite(start_yaw) || waypoints.empty()) throw std::invalid_argument("Invalid path start.");
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        if (!map.segment_free(waypoints[i == 0 ? 0 : i - 1], waypoints[i])) {
            throw std::invalid_argument("Cannot smooth a colliding route.");
        }
        if (i > 0 && distance(waypoints[i - 1], waypoints[i]) < 1e-8) {
            throw std::invalid_argument("Duplicate path waypoints.");
        }
    }
    if (waypoints.size() == 1) return {{{make_point(waypoints[0], start_yaw)}, false}};
    std::vector<PathSegment> result;
    const double initial_yaw = std::atan2(waypoints[1][1] - waypoints[0][1], waypoints[1][0] - waypoints[0][0]);
    if (std::abs(robot_interface::wrap_to_pi(initial_yaw - start_yaw)) > 1e-6) {
        result.push_back({{make_point(waypoints[0], start_yaw), make_point(waypoints[0], initial_yaw)}, true});
    }
    std::vector<TrajectoryPoint> moving;
    PathPoint current = waypoints[0];
    for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
        double cut = std::min({limits.corner_distance, 0.3 * distance(waypoints[i - 1], waypoints[i]),
                              0.3 * distance(waypoints[i], waypoints[i + 1])});
        std::vector<TrajectoryPoint> curve;
        for (int attempt = 0; attempt < 5; ++attempt) {
            curve = corner_curve(waypoints[i - 1], waypoints[i], waypoints[i + 1], cut, limits.sample_distance);
            if (collision_free(map, curve)) break;
            curve.clear();
            cut *= 0.5;
        }
        if (!curve.empty()) {
            append_line(moving, current, {curve.front().x_ref, curve.front().y_ref}, limits.sample_distance);
            for (const auto& point : curve) append_point(moving, point);
            current = {curve.back().x_ref, curve.back().y_ref};
        }
        else {
            append_line(moving, current, waypoints[i], limits.sample_distance);
            const auto before = moving.back();
            result.push_back({moving, false});
            moving.clear();
            const double yaw = std::atan2(waypoints[i + 1][1] - waypoints[i][1], waypoints[i + 1][0] - waypoints[i][0]);
            result.push_back({{before, make_point(waypoints[i], yaw)}, true});
            current = waypoints[i];
        }
    }
    append_line(moving, current, waypoints.back(), limits.sample_distance);
    if (!moving.empty()) result.push_back({moving, false});
    for (const auto& segment : result) {
        if (!collision_free(map, segment.points)) throw std::runtime_error("Smoothed path collision check failed.");
    }
    return result;
}

TimedTrajectory time_parameterize(const std::vector<PathSegment>& segments, const MotionLimits& limits) {
    std::vector<TimedSample> samples;
    double distance_origin = 0.0;
    for (const auto& segment : segments) {
        if (segment.points.empty()) throw std::invalid_argument("Empty geometry segment.");
        auto points = segment.points;
        const double local_origin = points.front().forward_position_ref;
        for (auto& point : points) point.forward_position_ref += distance_origin - local_origin;
        std::vector<TimedSample> phase;
        if (segment.rotation) phase = profile_turn(points.front(), points.back().yaw_ref, limits);
        else if (points.size() == 1) phase = {{points.front(), 0.0}};
        else phase = profile_motion(points, limits);
        const double time_origin = samples.empty() ? 0.0 : samples.back().time;
        if (!samples.empty()) {
            if (std::hypot(samples.back().point.x_ref - phase.front().point.x_ref,
                           samples.back().point.y_ref - phase.front().point.y_ref) > 1e-8
                || std::abs(robot_interface::wrap_to_pi(samples.back().point.yaw_ref - phase.front().point.yaw_ref)) > 1e-6) {
                throw std::runtime_error("Discontinuous trajectory segment join.");
            }
            samples.back().point = phase.front().point;
        }
        const std::size_t first = samples.empty() ? 0 : 1;
        for (std::size_t i = first; i < phase.size(); ++i) {
            phase[i].time += time_origin;
            samples.push_back(phase[i]);
        }
        distance_origin = samples.back().point.forward_position_ref;
    }
    return TimedTrajectory(std::move(samples));
}

void validate_trajectory(const OccupancyGrid& map, const TimedTrajectory& trajectory) {
    std::vector<TrajectoryPoint> points;
    for (const auto& sample : trajectory.samples()) points.push_back(sample.point);
    if (!collision_free(map, points)) throw std::runtime_error("Timed reference collides with inflated map.");
}

}
