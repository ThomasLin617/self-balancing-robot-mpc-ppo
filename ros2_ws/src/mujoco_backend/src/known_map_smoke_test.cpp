#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller/mpc_controller.hpp>
#include <mujoco_backend/known_map_world.hpp>
#include <mujoco_backend/mujoco_backend.hpp>
#include <mujoco_backend/mujoco_viewer.hpp>
#include <navigation/known_map.hpp>
#include <navigation/path_trajectory.hpp>
#include <navigation/reference_preview.hpp>
#include <robot_interface/angle.hpp>
#include <state_estimator/state_estimator.hpp>

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: known_map_smoke_test <robot_model> <scenario> [--headless]\n";
        return 1;
    }
    try {
        const bool headless = argc == 4 && std::string(argv[3]) == "--headless";
        if (argc == 4 && !headless) throw std::invalid_argument("Unknown known-map argument.");
        const auto scenario = navigation::KnownMap::load(argv[2]);
        const navigation::MotionLimits limits;
        const auto safe_map = scenario.grid.inflated(limits.robot_radius + limits.safety_margin);
        const auto routing_map = scenario.grid.inflated(limits.robot_radius + limits.safety_margin + limits.smoothing_clearance);
        const auto footprint_map = scenario.grid.inflated(limits.robot_radius);
        mujoco_backend::KnownMapWorld world(argv[1], scenario);
        mujoco_backend::MujocoBackend backend(world.path());
        std::unique_ptr<mujoco_backend::MujocoViewer> viewer;
        if (!headless) {
            viewer = std::make_unique<mujoco_backend::MujocoViewer>(backend);
            const auto lower = scenario.grid.grid_to_world({0,0});
            const auto upper = scenario.grid.grid_to_world({scenario.grid.height()-1,scenario.grid.width()-1});
            viewer->set_camera_overview({0.5*(lower[0]+upper[0]),0.5*(lower[1]+upper[1]),0.0},
                1.25*scenario.grid.resolution()*std::max(scenario.grid.width(),scenario.grid.height()));
            viewer->set_camera_azimuth(135.0);
            viewer->enable_studio_lighting();
        }
        state_estimator::StateEstimator estimator(0.5, 0.12);
        navigation::PlanarOdometry odometry;
        controller::MPCController mpc_controller(250, 2.0);
        robot_interface::ReferenceHorizon references(mpc_controller.horizon_size());
        robot_interface::Reference hold;
        robot_interface::RobotCommand command;
        std::unique_ptr<navigation::ReferenceTrajectory> trajectory;
        double position_origin = 0.0;
        double end_time = 0.0;
        double max_torque = 0.0;
        double max_pitch = 0.0;
        double max_odometry_error = 0.0;
        constexpr double startup_duration = 4.0;
        backend.reset();
        std::cout << std::fixed << std::setprecision(5);
        std::cout << "Known map | clearance = " << limits.robot_radius + limits.safety_margin
                  << " | recovery = " << startup_duration << " s\n";

        for (int step = 0; ; ++step) {
            if (viewer && viewer->should_close()) return 0;
            const auto state = estimator.update(backend.read_sensor_data());
            const auto pose = odometry.update(state);
            const double now = state.timestamp;
            if (!trajectory && now + 1e-9 >= startup_duration) {
                const navigation::PathPoint start{pose.x, pose.y};
                if (safe_map.segment_free(start, scenario.goal)) {
                    throw std::runtime_error("Known-map demonstration must require a detour.");
                }
                const auto raw = navigation::AStarPlanner().plan(routing_map, start, scenario.goal);
                const auto waypoints = navigation::simplify_path(routing_map, raw);
                const auto geometry = navigation::smooth_path(safe_map, waypoints, pose.yaw, limits);
                const auto timed = navigation::time_parameterize(geometry, limits);
                navigation::validate_trajectory(safe_map, timed);
                if (timed.duration() > 90.0) throw std::runtime_error("Known-map trajectory exceeds demonstration duration.");
                trajectory = std::make_unique<navigation::ReferenceTrajectory>(timed);
                position_origin = state.forward_position;
                end_time = startup_duration + trajectory->duration() + 5.0;
                std::size_t rotations = 0;
                for (const auto& segment : geometry) if (segment.rotation) ++rotations;
                std::cout << "Plan at " << now << " | estimated start = " << pose.x << ", " << pose.y << ", " << pose.yaw
                          << " | raw cells = " << raw.size() << " | waypoints = " << waypoints.size()
                          << " | rotations = " << rotations << " | samples = " << timed.samples().size()
                          << " | length = " << timed.samples().back().point.forward_position_ref
                          << " | duration = " << trajectory->duration() << '\n';
                for (const auto& p : waypoints) std::cout << "Waypoint " << p[0] << ", " << p[1] << '\n';
                if (viewer) {
                    std::vector<std::array<double,3>> raw_points, corners, points;
                    for (const auto& p : raw) raw_points.push_back({p[0],p[1],0.03});
                    for (const auto& p : waypoints) corners.push_back({p[0],p[1],0.045});
                    const auto& samples = trajectory->points();
                    const std::size_t stride = (samples.size() + 498) / 499;
                    for (std::size_t i = 0; i < samples.size(); i += stride) {
                        points.push_back({samples[i].x_ref,samples[i].y_ref,0.015});
                    }
                    points.push_back({samples.back().x_ref,samples.back().y_ref,0.015});
                    viewer->set_reference_path(points, {scenario.goal[0],scenario.goal[1],0.055});
                    viewer->set_planned_route(raw_points, corners);
                }
            }
            navigation::TrajectoryPoint point;
            if (trajectory) point = trajectory->sample(now - startup_duration);
            if (step % 2 == 0) {
                if (trajectory) {
                    navigation::sample_reference_horizon(*trajectory, now - startup_duration, position_origin,
                                                         mpc_controller.sample_time(), references);
                    command = mpc_controller.compute_command(state, references);
                }
                else command = mpc_controller.compute_command(state, hold);
            }
            max_torque = std::max({max_torque,std::abs(command.left_wheel_torque),std::abs(command.right_wheel_torque)});
            if (trajectory) max_pitch = std::max(max_pitch,std::abs(state.pitch));
            if (!std::isfinite(state.pitch) || !std::isfinite(command.left_wheel_torque)
                || !std::isfinite(command.right_wheel_torque) || max_torque > 2.000001
                || std::abs(state.pitch) > 0.7 || max_pitch > 0.2) {
                throw std::runtime_error("Known-map balance or torque validation failed.");
            }

            // Ground truth below is only for collision and result verification.
            const auto truth = backend.debug_planar_pose("base_body");
            if (!footprint_map.point_free({truth[0],truth[1]})) {
                throw std::runtime_error("Actual robot footprint left free space.");
            }
            for (const auto& obstacle : scenario.obstacles) {
                if (backend.debug_has_contact(("known_" + obstacle.name).c_str())) {
                    throw std::runtime_error("Robot contacted a known-map obstacle.");
                }
            }
            max_odometry_error = std::max(max_odometry_error,std::hypot(pose.x-truth[0],pose.y-truth[1]));
            if (step % 500 == 0) {
                std::cout << "t = " << now << " | pose = " << pose.x << ", " << pose.y << ", " << pose.yaw
                          << " | ref = " << point.x_ref << ", " << point.y_ref << ", " << point.yaw_ref
                          << " | phase = " << (!trajectory ? "recovery" : point.has_curvature ? "moving" : "rotation")
                          << " | v = " << state.forward_velocity << " | pitch = " << state.pitch
                          << " | L = " << command.left_wheel_torque << " | R = " << command.right_wheel_torque
                          << " | goal distance = " << std::hypot(pose.x-scenario.goal[0],pose.y-scenario.goal[1]) << '\n';
                std::cout << "debug truth = " << truth[0] << ", " << truth[1] << ", " << truth[2]
                          << " | odometry error = " << std::hypot(pose.x-truth[0],pose.y-truth[1]) << '\n';
            }
            if (trajectory && now + 1e-9 >= end_time) {
                const double estimated_error = std::hypot(pose.x-scenario.goal[0],pose.y-scenario.goal[1]);
                const double true_error = std::hypot(truth[0]-scenario.goal[0],truth[1]-scenario.goal[1]);
                if (estimated_error > 0.15 || true_error > 0.15 || std::abs(state.forward_velocity) > 0.04
                    || std::abs(robot_interface::wrap_to_pi(state.yaw-point.yaw_ref)) > 0.08) {
                    throw std::runtime_error("Known-map final tracking validation failed.");
                }
                std::cout << "PASS known map | time = " << now << " | estimated goal error = " << estimated_error
                          << " | debug true goal error = " << true_error << " | max torque = " << max_torque
                          << " | max pitch after recovery = " << max_pitch
                          << " | max odometry error = " << max_odometry_error << '\n';
                break;
            }
            backend.apply_command(command);
            backend.step();
            if (viewer && step % 16 == 0) viewer->render();
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
