#include <chrono>
#include <cmath>
#include <controller/mpc_controller.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <learning_policy/trajectory_policy.hpp>
#include <navigation/reference_preview.hpp>

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "Usage: d18_compute_timing actor output.csv\n";
        return 1;
    }
    try {
        learning_policy::TrajectoryPolicy policy(argv[1]);
        controller::MPCController mpc(250, 2.0);
        navigation::PlanarPose pose;
        navigation::ReferenceTrajectory trajectory(.2, .05);
        trajectory.generate(pose, .6, .15);
        robot_interface::ReferenceHorizon references(250);
        robot_interface::RobotState state;
        std::ofstream output(argv[2]);
        if (!output)
            throw std::runtime_error("Cannot open timing output.");
        output << std::setprecision(17)
               << "sample,actor_us,ppo_update_us,mpc_compute_us,mpc_update_us\n";
        double checksum = 0;
        for (int i = -500; i < 5000; ++i) {
            const double elapsed = (i + 500) * .0002;
            state.pitch = .001 * std::sin(elapsed);
            state.forward_velocity = .01 * std::cos(elapsed);
            navigation::sample_reference_horizon(trajectory, elapsed, 0, .002, references);
            const auto observation = policy.observation(state, references);
            const auto actor_begin = std::chrono::steady_clock::now();
            const auto action = policy.normalized_action(observation);
            const auto actor_end = std::chrono::steady_clock::now();
            checksum += action[0] + action[1];
            const auto ppo_begin = std::chrono::steady_clock::now();
            navigation::sample_reference_horizon(trajectory, elapsed, 0, .002, references);
            const auto torque = policy.torque(state, references);
            const auto ppo_end = std::chrono::steady_clock::now();
            checksum += torque[0] + torque[1];
            const auto mpc_begin = std::chrono::steady_clock::now();
            navigation::sample_reference_horizon(trajectory, elapsed, 0, .002, references);
            const auto compute_begin = std::chrono::steady_clock::now();
            const auto command = mpc.compute_command(state, references);
            const auto mpc_end = std::chrono::steady_clock::now();
            checksum += command.left_wheel_torque + command.right_wheel_torque;
            if (i >= 0) {
                output << i << ','
                       << std::chrono::duration<double, std::micro>(actor_end - actor_begin).count()
                       << ','
                       << std::chrono::duration<double, std::micro>(ppo_end - ppo_begin).count()
                       << ','
                       << std::chrono::duration<double, std::micro>(mpc_end - compute_begin).count()
                       << ','
                       << std::chrono::duration<double, std::micro>(mpc_end - mpc_begin).count()
                       << '\n';
            }
        }
        std::cout << "Timing checksum " << checksum << '\n';
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
