#include <cmath>
#include <iostream>
#include <stdexcept>
#include <navigation/planar_odometry.hpp>
#include <navigation/reference_trajectory.hpp>
#include <navigation/reference_preview.hpp>
#include <robot_interface/angle.hpp>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    try {
        navigation::PlanarOdometry odometry;
        robot_interface::RobotState state;
        state.forward_velocity = 0.2;
        odometry.update(state);
        navigation::PlanarPose pose;
        for (int i = 1; i <= 1000; ++i) {
            state.timestamp = i * 0.001;
            pose = odometry.update(state);
        }
        check(std::abs(pose.x - 0.2) < 1e-12 && pose.y == 0.0, "Straight odometry failed.");
        odometry.reset();
        state.timestamp = 0.0;
        state.yaw = 3.13;
        pose = odometry.update(state);
        check(pose.x == 0.0 && pose.y == 0.0, "Odometry reset failed.");
        state.timestamp = 0.1;
        state.yaw = -3.13;
        pose = odometry.update(state);
        check(std::abs(pose.x + 0.02) < 1e-12 && std::abs(pose.y) < 1e-12,
              "Midpoint yaw crossed the long way around pi.");
        odometry.reset();
        state = robot_interface::RobotState{};
        state.forward_velocity = 0.2;
        odometry.update(state);
        for (int i = 1; i <= 4000; ++i) {
            state.timestamp = i * 0.001;
            state.yaw = robot_interface::wrap_to_pi(state.timestamp);
            pose = odometry.update(state);
        }
        check(std::hypot(pose.x - 0.2 * std::sin(4.0),
                         pose.y - 0.2 * (1.0 - std::cos(4.0))) < 1e-7,
              "Circular odometry failed.");

        navigation::ReferenceTrajectory trajectory(0.18, 0.08);
        for (double yaw : {0.0, 0.7, 3.13}) {
            navigation::PlanarPose start{0.2, -0.1, yaw};
            const double goal_x = start.x + 0.7 * std::cos(yaw + 0.4);
            const double goal_y = start.y + 0.7 * std::sin(yaw + 0.4);
            trajectory.generate(start, goal_x, goal_y);
            auto first = trajectory.sample(0.0);
            auto last = trajectory.sample(trajectory.duration());
            check(std::hypot(first.x_ref - start.x, first.y_ref - start.y) < 1e-12,
                  "Trajectory start does not match current pose.");
            check(std::abs(robot_interface::wrap_to_pi(first.yaw_ref - yaw)) < 1e-12,
                  "Starting tangent does not follow yaw.");
            check(std::hypot(last.x_ref - goal_x, last.y_ref - goal_y) < 1e-12,
                  "Trajectory endpoint is incorrect.");
            check(first.forward_velocity_ref == 0.0 && last.forward_velocity_ref == 0.0
                  && last.yaw_rate_ref == 0.0, "Endpoint velocity is nonzero.");
            double previous_distance = 0.0;
            double previous_velocity = 0.0;
            for (double t = 0.0; t < trajectory.duration() + 0.01; t += 0.001) {
                const auto point = trajectory.sample(t);
                check(point.forward_position_ref >= previous_distance
                      && point.forward_velocity_ref <= 0.18 + 1e-12,
                      "Trajectory distance or speed limit failed.");
                check(std::abs(point.forward_velocity_ref - previous_velocity) <= 0.000080001,
                      "Acceleration limit failed.");
                check(std::isfinite(point.yaw_rate_ref), "Invalid yaw rate.");
                previous_distance = point.forward_position_ref;
                previous_velocity = point.forward_velocity_ref;
            }
        }
        trajectory.generate({0.0, 0.0, 0.0}, 0.01, 0.0);
        check(trajectory.duration() < 1.0, "Short triangular profile failed.");
        trajectory.generate({0.0, 0.0, 0.0}, 0.0, 0.0);
        check(trajectory.duration() == 0.0 && trajectory.sample(1.0).forward_velocity_ref == 0.0,
              "Zero-distance trajectory failed.");
        trajectory.generate({0.0, 0.0, 0.0}, 0.6, 0.4);
        robot_interface::ReferenceHorizon references(250);
        navigation::sample_reference_horizon(trajectory, 1.0, 12.4, 0.002, references);
        const auto expected_first = trajectory.sample(1.002);
        const auto expected_last = trajectory.sample(1.5);
        check(std::abs(references.front().forward_position - 12.4
                       - expected_first.forward_position_ref) < 1e-12
              && std::abs(references.back().forward_position - 12.4
                          - expected_last.forward_position_ref) < 1e-12,
              "Preview sampling time or longitudinal origin is incorrect.");
        navigation::sample_reference_horizon(trajectory, trajectory.duration() - 0.1,
                                              -7.0, 0.002, references);
        const auto endpoint = trajectory.sample(trajectory.duration());
        for (int i = 50; i < 250; ++i) {
            check(std::abs(references[i].forward_position + 7.0
                           - endpoint.forward_position_ref) < 1e-12
                  && references[i].forward_velocity == 0.0
                  && references[i].yaw_rate == 0.0
                  && references[i].yaw == endpoint.yaw_ref,
                  "Preview extrapolated beyond the trajectory endpoint.");
        }
        std::cout << "PASS planar odometry, wrap, trajectory endpoints, speed, acceleration and preview sampling\n";
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
