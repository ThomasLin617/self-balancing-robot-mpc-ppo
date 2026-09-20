#include <mujoco_backend/mujoco_backend.hpp>
#include <mujoco_backend/mujoco_viewer.hpp>
#include <learning_policy/ppo_policy.hpp>
#include <navigation/planar_odometry.hpp>
#include <state_estimator/state_estimator.hpp>
#include <robot_interface/robot_command.hpp>
#include <robot_interface/angle.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::string nominal_model(const std::string& original) {
    std::ifstream input(original);
    if (!input) throw std::runtime_error("Cannot open PPO smoke-test model.");
    std::ostringstream content;
    content << input.rdbuf();
    const std::string old_value = "euler=\"0 -20 0\"";
    const std::string new_value = "euler=\"0 0 0\"";
    const auto position = content.str().find(old_value);
    if (position == std::string::npos) throw std::runtime_error("Nominal PPO model replacement was not found.");
    std::string xml = content.str();
    xml.replace(position, old_value.size(), new_value);
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::string path = (std::filesystem::temp_directory_path()
        / ("ppo_nominal_" + std::to_string(stamp) + ".xml")).string();
    std::ofstream output(path);
    if (!output) throw std::runtime_error("Cannot create nominal PPO model.");
    output << xml;
    return path;
}

}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: ppo_smoke_test <robot_model> <policy.txt> <scenario.txt> [--headless]\n";
        return 1;
    }
    const bool headless = argc > 4 && std::string(argv[4]) == "--headless";
    try {
        std::ifstream scenario(argv[3]);
        std::string format;
        double radius, velocity_limit, yaw_rate_limit, pitch_limit, hold_seconds, timeout, fall_angle;
        std::size_t goal_count = 0;
        if (!(scenario >> format >> radius >> velocity_limit >> yaw_rate_limit >> pitch_limit
              >> hold_seconds >> timeout >> fall_angle >> goal_count)
            || format != "d15_goal_sequence_v1" || goal_count == 0 || goal_count > 100) {
            throw std::runtime_error("Invalid PPO goal scenario.");
        }
        for (double value : {radius, velocity_limit, yaw_rate_limit, pitch_limit, hold_seconds, timeout, fall_angle}) {
            if (!std::isfinite(value) || value <= 0.0) throw std::runtime_error("Invalid PPO scenario limit.");
        }
        std::vector<std::array<double, 2>> offsets(goal_count);
        for (auto& goal : offsets) {
            if (!(scenario >> goal[0] >> goal[1]) || !std::isfinite(goal[0]) || !std::isfinite(goal[1])) {
                throw std::runtime_error("Invalid PPO goal offset.");
            }
        }
        const std::string model_path = nominal_model(argv[1]);
        mujoco_backend::MujocoBackend backend(model_path);
        std::filesystem::remove(model_path);
        std::unique_ptr<mujoco_backend::MujocoViewer> viewer;
        if (!headless) {
            viewer = std::make_unique<mujoco_backend::MujocoViewer>(backend);
            viewer->track_body("base_body");
            viewer->set_camera_azimuth(135.0);
            viewer->enable_studio_lighting();
        }
        learning_policy::PPOPolicy policy(argv[2]);
        learning_policy::GoalObservationBuilder observation_builder(policy.observation_config());
        state_estimator::StateEstimator estimator(0.5, 0.12);
        navigation::PlanarOdometry odometry;
        std::array<double, 2> goal = offsets.front();
        std::size_t goal_index = 0;
        std::size_t successes = 0;
        double goal_start = 0.0;
        double settled_time = 0.0;
        double maximum_pitch = 0.0;
        double maximum_torque = 0.0;
        double maximum_truth_error = 0.0;
        double previous_timestamp = 0.0;
        int control_step = 0;
        robot_interface::RobotCommand command;
        backend.reset();
        estimator.reset();
        odometry.reset();
        while (goal_index < offsets.size()) {
            const auto state = estimator.update(backend.read_sensor_data());
            const auto pose = odometry.update(state);
            if (control_step % 2 == 0) {
                const auto torque = policy.torque(state, pose, goal, observation_builder);
                command.timestamp = state.timestamp;
                command.left_wheel_torque = torque[0];
                command.right_wheel_torque = torque[1];
            }
            if (std::abs(command.left_wheel_torque) > 2.000001 || std::abs(command.right_wheel_torque) > 2.000001) {
                throw std::runtime_error("PPO torque limit failed.");
            }
            maximum_torque = std::max({maximum_torque, std::abs(command.left_wheel_torque), std::abs(command.right_wheel_torque)});
            maximum_pitch = std::max(maximum_pitch, std::abs(state.pitch));
            if (std::abs(state.pitch) > fall_angle) throw std::runtime_error("PPO fell; no recovery controller is used.");
            const double distance = std::hypot(goal[0] - pose.x, goal[1] - pose.y);
            const bool settled = distance < radius && std::abs(state.forward_velocity) < velocity_limit
                && std::abs(state.yaw_rate) < yaw_rate_limit && std::abs(state.pitch) < pitch_limit;
            settled_time = settled ? settled_time + state.timestamp - previous_timestamp : 0.0;
            previous_timestamp = state.timestamp;
            const auto truth = backend.debug_planar_pose("base_body");
            const double truth_distance = std::hypot(goal[0] - truth[0], goal[1] - truth[1]);
            if (control_step % 500 == 0) {
                std::cout << "t = " << state.timestamp << " | goal = " << goal_index
                          << " | estimated distance = " << distance << " | truth distance = " << truth_distance
                          << " | pitch = " << state.pitch << " | torque = " << command.left_wheel_torque
                          << ", " << command.right_wheel_torque << '\n';
            }
            const bool success = settled_time >= hold_seconds;
            if (success || state.timestamp - goal_start >= timeout) {
                std::cout << "goal " << goal_index << (success ? " reached" : " timed out")
                          << " at t = " << state.timestamp << " | duration = " << state.timestamp - goal_start
                          << " | estimated distance = " << distance << " | truth distance = " << truth_distance << '\n';
                successes += success;
                ++goal_index;
                settled_time = 0.0;
                goal_start = state.timestamp;
                if (goal_index < offsets.size()) {
                    const auto offset = offsets[goal_index];
                    goal = {pose.x + std::cos(pose.yaw) * offset[0] - std::sin(pose.yaw) * offset[1],
                            pose.y + std::sin(pose.yaw) * offset[0] + std::cos(pose.yaw) * offset[1]};
                }
            }
            maximum_truth_error = std::max(maximum_truth_error, std::hypot(pose.x - truth[0], pose.y - truth[1]));
            backend.apply_command(command);
            backend.step();
            if (viewer && control_step % 16 == 0) {
                viewer->set_reference_path({{goal[0], goal[1], 0.05}}, {goal[0], goal[1], 0.05});
                viewer->render();
                if (viewer->should_close()) break;
            }
            ++control_step;
        }
        if (successes != offsets.size()) throw std::runtime_error("PPO smoke test did not reach every goal.");
        std::cout << std::fixed << std::setprecision(6)
                  << "PASS PPO torque control | goals = " << goal_index
                  << " | max pitch = " << maximum_pitch
                  << " | max torque = " << maximum_torque
                  << " | max odometry error = " << maximum_truth_error << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
