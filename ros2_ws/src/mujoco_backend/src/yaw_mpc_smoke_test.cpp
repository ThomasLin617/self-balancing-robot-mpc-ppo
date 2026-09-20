#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller/mpc_controller.hpp>
#include <mujoco_backend/mujoco_backend.hpp>
#include <mujoco_backend/mujoco_viewer.hpp>
#include <robot_interface/angle.hpp>
#include <state_estimator/state_estimator.hpp>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: yaw_mpc_smoke_test <model_path> <forward|yaw|combined> [--headless]\n";
        return 1;
    }

    try {
        const std::string mode = argv[2];
        if (mode != "forward" && mode != "yaw" && mode != "combined") {
            throw std::invalid_argument("Unknown MPC test case.");
        }
        const bool headless = argc == 4 && std::string(argv[3]) == "--headless";
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
        robot_interface::Reference reference;
        reference.forward_position = 0.1;
        robot_interface::RobotCommand command;
        robot_interface::RobotState state;
        backend.reset();
        double max_torque = 0.0;
        double max_pitch = 0.0;
        double max_difference = 0.0;
        bool changed_reference = false;
        std::cout << std::fixed << std::setprecision(6);

        for (int step = 0; step <= 10000; ++step) {
            if (viewer && viewer->should_close()) {
                throw std::runtime_error("Test closed before completion.");
            }
            state = estimator.update(backend.read_sensor_data());
            if (!changed_reference && state.timestamp >= 4.0) {
                if (mode == "yaw") {
                    reference.forward_position = state.forward_position;
                    reference.yaw = 25.0 * std::acos(-1.0) / 180.0;
                }
                if (mode == "combined") {
                    reference.forward_position = state.forward_position + 0.1;
                    reference.yaw = 15.0 * std::acos(-1.0) / 180.0;
                }
                changed_reference = true;
            }
            if (step % 2 == 0) {
                command = mpc_controller.compute_command(state, reference);
            }
            if (!std::isfinite(state.pitch) || !std::isfinite(state.yaw)
                || !std::isfinite(command.left_wheel_torque)
                || !std::isfinite(command.right_wheel_torque)) {
                throw std::runtime_error("Non-finite MPC state or command.");
            }
            max_torque = std::max({max_torque, std::abs(command.left_wheel_torque),
                                  std::abs(command.right_wheel_torque)});
            if (changed_reference) {
                max_pitch = std::max(max_pitch, std::abs(state.pitch));
                max_difference = std::max(max_difference,
                    std::abs(command.left_wheel_torque - command.right_wheel_torque));
            }
            if (max_torque > 2.000001 || std::abs(state.pitch) > 0.7) {
                throw std::runtime_error("Torque or pitch limit exceeded.");
            }
            if (step == 0 && std::abs(command.left_wheel_torque + 0.597280148684) > 0.002) {
                throw std::runtime_error("Forward MPC differs from the original numerical baseline.");
            }
            if (step % 1000 == 0) {
                std::cout << "t = " << state.timestamp
                          << " | p = " << state.forward_position
                          << " | pitch = " << state.pitch
                          << " | yaw = " << state.yaw
                          << " | L = " << command.left_wheel_torque
                          << " | R = " << command.right_wheel_torque << '\n';
            }
            backend.apply_command(command);
            backend.step();
            if (viewer && step % 16 == 0) viewer->render();
        }
        const double yaw_error = robot_interface::wrap_to_pi(state.yaw - reference.yaw);
        if (std::abs(state.forward_position - reference.forward_position) > 0.04
            || std::abs(yaw_error) > 0.04 || max_pitch > 0.15
            || (mode == "forward" && max_difference > 0.002)
            || (mode != "forward" && max_difference < 0.05)) {
            throw std::runtime_error("MPC tracking validation failed.");
        }
        std::cout << "PASS " << mode << " | max torque = " << max_torque
                  << " | maneuver max pitch = " << max_pitch
                  << " | yaw error = " << yaw_error << '\n';
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
