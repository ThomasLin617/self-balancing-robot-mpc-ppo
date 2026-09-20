#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <controller/mpc_controller.hpp>
#include <mujoco_backend/mujoco_backend.hpp>
#include <mujoco_backend/mujoco_viewer.hpp>
#include <navigation/planar_odometry.hpp>
#include <navigation/reference_trajectory.hpp>
#include <navigation/reference_preview.hpp>
#include <state_estimator/state_estimator.hpp>
#include <robot_interface/angle.hpp>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: navigation_smoke_test <model_path> [--headless] [--duration seconds]\n";
        return 1;
    }

    try {
        bool headless = false;
        double duration = 80.0;
        for (int i = 2; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--headless") headless = true;
            else if (argument == "--duration" && i + 1 < argc) duration = std::stod(argv[++i]);
            else throw std::invalid_argument("Unknown navigation argument.");
        }
        if (!std::isfinite(duration) || duration < 10.0) {
            throw std::invalid_argument("Navigation duration must be at least 10 seconds.");
        }
        mujoco_backend::MujocoBackend backend(argv[1]);
        std::unique_ptr<mujoco_backend::MujocoViewer> viewer;
        if (!headless) {
            viewer = std::make_unique<mujoco_backend::MujocoViewer>(backend);
            viewer->track_body("base_body");
            viewer->set_camera_azimuth(135.0);
            viewer->enable_studio_lighting();
        }
        state_estimator::StateEstimator estimator(0.5, 0.12);
        controller::MPCController mpc_controller(250, 2.0);
        navigation::PlanarOdometry odometry;
        navigation::ReferenceTrajectory trajectory(0.18, 0.08);
        const std::array<std::array<double, 2>, 6> goals = {{
            {0.6, 0.0}, {1.2, 0.4}, {1.2, 1.1}, {0.6, 1.5},
            {0.0, 1.1}, {0.0, 0.4}
        }};
        std::size_t goal_index = 0;
        std::array<double, 2> goal{};
        double trajectory_start_time = 0.0;
        double position_origin = 0.0;
        double max_torque = 0.0;
        double max_pitch = 0.0;
        double max_goal_error = 0.0;
        constexpr double startup_duration = 4.0;
        const double end_time = startup_duration + duration;
        double next_target_time = startup_duration;
        int completed_targets = 0;
        bool active = false;
        robot_interface::Reference reference;
        robot_interface::ReferenceHorizon references(mpc_controller.horizon_size());
        robot_interface::RobotCommand command;
        backend.reset();
        estimator.reset();
        odometry.reset();
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Stabilizing until t = " << startup_duration << " s\n";

        for (int step = 0; ; ++step) {
            if (viewer && viewer->should_close()) return 0;
            const auto state = estimator.update(backend.read_sensor_data());
            const auto pose = odometry.update(state);
            const double now = state.timestamp;
            if (now + 1e-9 >= next_target_time || now + 1e-9 >= end_time) {
                if (active && now + 1e-9 >= next_target_time) {
                    const double error = std::hypot(pose.x - goal[0], pose.y - goal[1]);
                    max_goal_error = std::max(max_goal_error, error);
                    ++completed_targets;
                    std::cout << "Goal check at " << now << " | distance = " << error << '\n';
                    if (error > 0.20) throw std::runtime_error("Navigation goal error exceeds 0.20 m.");
                }
                if (now + 1e-9 >= end_time) break;
                goal = goals[goal_index];
                goal_index = (goal_index + 1) % goals.size();
                trajectory.generate(pose, goal[0], goal[1]);
                trajectory_start_time = now;
                position_origin = state.forward_position;
                next_target_time += 10.0;
                active = true;
                if (trajectory.duration() > 9.5) {
                    throw std::runtime_error("Trajectory cannot finish before the next target.");
                }
                if (viewer) {
                    std::vector<std::array<double, 3>> points;
                    for (const auto& point : trajectory.points()) {
                        points.push_back({point.x_ref, point.y_ref, 0.015});
                    }
                    viewer->set_reference_path(points, {goal[0], goal[1], 0.055});
                }
                std::cout << "New target at " << now << " | goal = " << goal[0] << ", " << goal[1]
                          << " | trajectory duration = " << trajectory.duration() << '\n';
            }
            navigation::TrajectoryPoint point;
            if (active) point = trajectory.sample(now - trajectory_start_time);
            if (step % 2 == 0) {
                if (active) {
                    navigation::sample_reference_horizon(
                        trajectory, now - trajectory_start_time, position_origin,
                        mpc_controller.sample_time(), references
                    );
                    command = mpc_controller.compute_command(state, references);
                }
                else {
                    command = mpc_controller.compute_command(state, reference);
                }
            }
            max_torque = std::max({max_torque, std::abs(command.left_wheel_torque),
                                  std::abs(command.right_wheel_torque)});
            if (now > startup_duration) max_pitch = std::max(max_pitch, std::abs(state.pitch));
            if (!std::isfinite(state.pitch) || !std::isfinite(command.left_wheel_torque)
                || !std::isfinite(command.right_wheel_torque)
                || std::abs(state.pitch) > 0.7 || max_torque > 2.000001 || max_pitch > 0.2) {
                throw std::runtime_error("Navigation state or torque limit failed.");
            }
            if (step % 500 == 0) {
                const auto truth = backend.debug_planar_pose("base_body");
                std::cout << "debug truth = " << truth[0] << ", " << truth[1] << ", " << truth[2]
                          << " | odometry error = " << std::hypot(pose.x - truth[0], pose.y - truth[1])
                          << " | yaw error = " << robot_interface::wrap_to_pi(pose.yaw - truth[2]) << '\n';
                std::cout << "t = " << now
                          << " | phase = " << (active ? "navigation" : "recovery")
                          << " | pose = " << pose.x << ", " << pose.y << ", " << pose.yaw
                          << " | goal = " << goal[0] << ", " << goal[1]
                          << " | ref = " << point.x_ref << ", " << point.y_ref << ", " << point.yaw_ref
                          << " | preview end p/yaw = " << references.back().forward_position
                          << ", " << references.back().yaw
                          << " | path error = " << state.forward_position
                              - (position_origin + point.forward_position_ref)
                          << " | heading error = " << robot_interface::wrap_to_pi(state.yaw - point.yaw_ref)
                          << " | v = " << state.forward_velocity << " | pitch = " << state.pitch
                          << " | L = " << command.left_wheel_torque << " | R = " << command.right_wheel_torque
                          << " | distance = " << std::hypot(pose.x - goal[0], pose.y - goal[1]) << '\n';
            }
            backend.apply_command(command);
            backend.step();
            if (viewer && step % 16 == 0) viewer->render();
        }
        std::cout << "PASS navigation | targets = " << completed_targets
                  << " | max goal error = " << max_goal_error
                  << " | max torque = " << max_torque
                  << " | max pitch after startup = " << max_pitch << '\n';
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
