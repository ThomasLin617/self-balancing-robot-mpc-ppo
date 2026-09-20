#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <navigation/known_map.hpp>
#include <navigation/path_trajectory.hpp>
#include <navigation/reference_preview.hpp>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template<class Function>
void rejects(Function function, const char* message) {
    bool rejected = false;
    try { function(); }
    catch (const std::exception&) { rejected = true; }
    check(rejected, message);
}

void check_profile(const navigation::TimedTrajectory& trajectory, const navigation::MotionLimits& limits) {
    const auto& samples = trajectory.samples();
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto& point = samples[i].point;
        const double v = point.forward_velocity_ref;
        check(v <= limits.max_velocity + 1e-10, "Profile exceeds maximum velocity.");
        if (point.has_curvature) {
            check(v*v*std::abs(point.curvature) <= limits.lateral_acceleration + 1e-10, "Lateral acceleration limit failed.");
        }
        if (i > 0) {
            const double ds = point.forward_position_ref - samples[i - 1].point.forward_position_ref;
            const double previous = samples[i - 1].point.forward_velocity_ref;
            check(v*v - previous*previous <= 2*limits.acceleration*ds + 1e-10, "Forward pass failed.");
            check(previous*previous - v*v <= 2*limits.deceleration*ds + 1e-10, "Backward pass failed.");
        }
    }
    for (double t = 0.0; t < trajectory.duration(); t += 0.003) {
        const auto point = trajectory.sample(t);
        check(std::isfinite(point.yaw_ref) && std::isfinite(point.yaw_rate_ref), "Non-finite sampled reference.");
        check(point.forward_velocity_ref >= -1e-10 && point.forward_velocity_ref <= limits.max_velocity + 1e-10, "Interpolated speed limit failed.");
        check(std::abs(point.yaw_rate_ref) <= limits.max_yaw_rate + 1e-10, "Yaw rate limit failed.");
        if (point.has_curvature) {
            check(point.forward_velocity_ref*point.forward_velocity_ref*std::abs(point.curvature)
                  <= limits.lateral_acceleration + 1e-10, "Interpolated curvature limit failed.");
        }
    }
    check(trajectory.sample(trajectory.duration() + 1.0).forward_velocity_ref == 0.0, "Final velocity is nonzero.");
}

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    try {
        using namespace navigation;
        OccupancyGrid grid(10, 8, 0.1, -0.5, -0.4);
        for (int r = 0; r < 8; ++r) for (int c = 0; c < 10; ++c) {
            check(grid.world_to_grid(grid.grid_to_world({r,c})) == GridCell{r,c}, "Coordinate round trip failed.");
        }
        check(grid.world_to_grid({-0.5,-0.4}) == GridCell{0,0}, "Lower boundary is excluded.");
        rejects([&] { grid.world_to_grid({0.5,0.0}); }, "Upper boundary is included.");
        rejects([&] { grid.world_to_grid({-0.5001,0.0}); }, "Negative outside point was truncated into map.");
        rejects([&] { grid.grid_to_world({8,0}); }, "Invalid grid index accepted.");
        rejects([&] { grid.world_to_grid({std::numeric_limits<double>::quiet_NaN(),0}); }, "NaN coordinates accepted.");
        OccupancyGrid inflation(21, 21, 0.1, 0.0, 0.0);
        inflation.set_occupied({10,10});
        const auto inflated = inflation.inflated(0.11);
        check(inflated.occupied({10,12}) && !inflated.occupied({10,13}), "Inflation neighborhood is incorrect.");
        check(inflated.occupied({0,10}) && inflated.occupied({1,10}) && !inflated.occupied({2,10}), "Boundary inflation is incorrect.");
        AStarPlanner planner;
        const auto straight = planner.plan(grid, {-0.35,-0.25}, {0.35,0.25});
        check(straight.front() == PathPoint{-0.35,-0.25} && straight.back() == PathPoint{0.35,0.25}, "A* endpoints changed.");
        const auto reduced = simplify_path(grid, straight);
        check(reduced.size() == 2, "Open-space waypoint simplification failed.");
        check(planner.plan(grid, {0.05,0.05}, {0.05,0.05}).size() == 1, "Identical start and goal failed.");
        rejects([&] { planner.plan(grid, {-2.0,0.0}, {0.0,0.0}); }, "Invalid start accepted.");
        rejects([&] { planner.plan(grid, {0.0,0.0}, {2.0,0.0}); }, "Invalid goal accepted.");
        grid.set_occupied({4,5});
        rejects([&] { planner.plan(grid, grid.grid_to_world({4,5}), {0.35,0.25}); }, "Occupied start accepted.");
        rejects([&] { planner.plan(grid, {-0.35,-0.25}, grid.grid_to_world({4,5})); }, "Occupied goal accepted.");
        for (int r = 0; r < 8; ++r) grid.set_occupied({r,5});
        rejects([&] { planner.plan(grid, {-0.35,-0.25}, {0.35,0.25}); }, "No-path search returned a partial route.");
        OccupancyGrid corner(3,3,1.0,0.0,0.0);
        corner.set_occupied({0,1}); corner.set_occupied({1,0});
        rejects([&] { planner.plan(corner, {0.5,0.5}, {1.5,1.5}); }, "A* cut an occupied diagonal corner.");
        check(!corner.segment_free({0.5,0.5},{1.5,1.5}), "Segment checker missed corner contact.");

        MotionLimits fallback_limits;
        OccupancyGrid tight_corner(4,4,1.0,0.0,0.0);
        tight_corner.set_occupied({1,1});
        const auto fallback = smooth_path(tight_corner,
            {{0.5,0.99999},{2.00001,0.99999},{2.00001,2.5}}, 0.0, fallback_limits);
        bool fallback_turn = false;
        for (const auto& segment : fallback) if (segment.rotation) fallback_turn = true;
        check(fallback_turn, "Unsafe corner smoothing did not fall back to a stop and turn.");
        const auto fallback_timing = time_parameterize(fallback, fallback_limits);
        validate_trajectory(tight_corner, fallback_timing);
        check_profile(fallback_timing, fallback_limits);

        const auto scenario = KnownMap::load(argv[1]);
        MotionLimits limits;
        const auto safe = scenario.grid.inflated(limits.robot_radius + limits.safety_margin);
        const auto routing = scenario.grid.inflated(limits.robot_radius + limits.safety_margin + limits.smoothing_clearance);
        check(!safe.segment_free({0.0,0.0}, scenario.goal), "Scenario does not force a detour.");
        const auto raw = planner.plan(routing, {0.0,0.0}, scenario.goal);
        const auto waypoints = simplify_path(routing, raw);
        check(waypoints.size() > 2 && waypoints.size() < raw.size(), "Simplification lost the protected obstacle corner.");
        const auto segments = smooth_path(safe, waypoints, 0.0, limits);
        const auto timed = time_parameterize(segments, limits);
        validate_trajectory(safe, timed);
        check_profile(timed, limits);
        const auto first = timed.sample(0.0);
        const auto last = timed.sample(timed.duration());
        check(std::hypot(first.x_ref,first.y_ref) < 1e-10
              && std::hypot(last.x_ref-scenario.goal[0],last.y_ref-scenario.goal[1]) < 1e-10, "Smoothing changed endpoints.");
        bool turning = false;
        bool curved = false;
        for (const auto& p : timed.samples()) {
            if (!p.point.has_curvature && std::abs(p.point.yaw_rate_ref) > 0.01) turning = true;
            if (p.point.has_curvature && std::abs(p.point.curvature) > 0.1) curved = true;
        }
        check(turning && curved, "Scenario did not exercise rotation and curved motion.");
        ReferenceTrajectory reference(timed);
        robot_interface::ReferenceHorizon horizon(250);
        sample_reference_horizon(reference, reference.duration()-0.1, 7.0, 0.002, horizon);
        check(std::abs(horizon.back().forward_position - 7.0 - last.forward_position_ref) < 1e-10
              && horizon.back().yaw_rate == 0.0 && horizon.back().forward_velocity == 0.0, "D13 preview adapter lost endpoint semantics.");
        auto rotation_start = first;
        rotation_start.yaw_ref = 179.0 * std::acos(-1.0)/180.0;
        const TimedTrajectory turn(profile_turn(rotation_start, -179.0 * std::acos(-1.0)/180.0, limits));
        const auto middle = turn.sample(turn.duration()/2.0);
        check(!middle.has_curvature && middle.forward_velocity_ref == 0.0
              && middle.yaw_rate_ref > 0.0 && middle.x_ref == first.x_ref
              && middle.forward_position_ref == first.forward_position_ref, "In-place wrapped rotation failed.");
        rejects([&] { smooth_path(safe, {{0,0},scenario.goal}, 0.0, limits); }, "Colliding smoothing input accepted.");
        auto bad = timed.samples(); bad[1].point.x_ref = 1.1; bad[1].point.y_ref = 0.0;
        rejects([&] { validate_trajectory(safe, TimedTrajectory(bad)); }, "Colliding trajectory accepted.");
        std::vector<TrajectoryPoint> bend(101);
        for (int i = 0; i <= 100; ++i) {
            bend[i].x_ref = i * 0.01;
            bend[i].forward_position_ref = i * 0.01;
            bend[i].curvature = i >= 70 ? 10.0 : 0.0;
        }
        const TimedTrajectory slowdown(profile_motion(bend, limits));
        check_profile(slowdown, limits);
        check(slowdown.samples()[65].point.forward_velocity_ref < limits.max_velocity,
              "Backward pass did not slow down before future curvature.");
        auto inconsistent = slowdown.samples();
        inconsistent[1].time *= 2.0;
        rejects([&] { TimedTrajectory invalid(inconsistent); }, "Impossible interval timing accepted.");
        const auto stationary = time_parameterize(smooth_path(safe, {{0,0}}, 0.0, limits), limits);
        check(stationary.duration() == 0.0, "Zero-distance plan failed.");
        std::cout << "PASS planning | raw = " << raw.size() << " | waypoints = " << waypoints.size()
                  << " | samples = " << timed.samples().size() << " | duration = " << timed.duration()
                  << " | length = " << last.forward_position_ref << '\n';
        for (const auto& p : waypoints) std::cout << "Waypoint " << p[0] << ", " << p[1] << '\n';
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
