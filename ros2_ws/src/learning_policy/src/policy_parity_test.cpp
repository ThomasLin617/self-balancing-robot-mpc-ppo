#include <learning_policy/ppo_policy.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) return 2;
    try {
        learning_policy::PPOPolicy policy(argv[1]);
        learning_policy::GoalObservationBuilder builder(policy.observation_config());
        robot_interface::RobotState state;
        state.forward_velocity = 0.1;
        state.pitch = 0.03;
        state.pitch_rate = -0.2;
        state.yaw_rate = 0.4;
        navigation::PlanarPose pose;
        pose.x = 2.0;
        pose.y = -1.0;
        pose.yaw = 1.5707963267948966;
        const auto built = builder.build(state, pose, {2.0, 0.0});
        const std::array<double, 6> expected_observation{1.0, 0.0, 0.1, 0.03 / 0.35, -0.1, 0.2};
        for (std::size_t i = 0; i < built.size(); ++i) {
            if (std::abs(built[i] - expected_observation[i]) > 1e-12) throw std::runtime_error("Goal observation parity failed.");
        }
        if (argc == 4) {
            std::ifstream observations(argv[3]);
            if (!observations) throw std::runtime_error("Cannot open preprocessing vectors.");
            std::size_t count = 0;
            while (observations >> pose.x) {
                std::array<double, 2> goal{};
                observations >> pose.y >> pose.yaw >> goal[0] >> goal[1]
                    >> state.forward_velocity >> state.pitch >> state.pitch_rate >> state.yaw_rate;
                std::array<double, 6> expected{};
                for (double& value : expected) observations >> value;
                if (!observations) throw std::runtime_error("Truncated preprocessing vector.");
                const auto actual = builder.build(state, pose, goal);
                for (std::size_t i = 0; i < actual.size(); ++i) {
                    if (!std::isfinite(expected[i]) || std::abs(actual[i] - expected[i]) > 1e-6) {
                        throw std::runtime_error("Python/C++ preprocessing mismatch.");
                    }
                }
                ++count;
            }
            if (count == 0) throw std::runtime_error("Empty preprocessing vectors.");
        }
        std::ifstream file(argv[2]);
        if (!file) throw std::runtime_error("Cannot open parity vectors.");
        std::string tag;
        double maximum_error = 0.0;
        std::size_t count = 0;
        while (file >> tag) {
            if (tag != "observation") throw std::runtime_error("Invalid parity vector.");
            std::array<double, 6> input{};
            for (double& value : input) file >> value;
            file >> tag;
            if (tag != "action") throw std::runtime_error("Invalid parity action.");
            std::array<double, 2> expected{};
            for (double& value : expected) file >> value;
            if (!file) throw std::runtime_error("Truncated parity action.");
            const auto actual = policy.normalized_action(input);
            for (std::size_t i = 0; i < actual.size(); ++i) {
                if (!std::isfinite(expected[i])) throw std::runtime_error("Non-finite parity action.");
                maximum_error = std::max(maximum_error, std::abs(actual[i] - expected[i]));
            }
            ++count;
        }
        if (count == 0) throw std::runtime_error("Empty parity vectors.");
        if (maximum_error > 1e-6) throw std::runtime_error("Python/C++ actor parity failed.");
        std::cout << "PASS policy parity | maximum error = " << maximum_error << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
