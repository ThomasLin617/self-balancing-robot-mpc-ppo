#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <controller/mpc_controller.hpp>
#include <controller/mpc_model.hpp>
#include <robot_interface/angle.hpp>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    try {
        controller::MPCController mpc(250, 2.0);
        robot_interface::RobotState state;
        state.forward_position = 0.03;
        state.forward_velocity = -0.02;
        state.pitch = 0.01;
        state.pitch_rate = -0.015;
        state.yaw = 0.1;
        state.yaw_rate = 0.03;
        robot_interface::Reference reference;
        reference.forward_position = 0.1;
        reference.yaw = 0.2;
        robot_interface::ReferenceHorizon references(mpc.horizon_size(), reference);

        Eigen::Matrix<double, 6, 6> Ad, Qd, P;
        Eigen::Matrix<double, 6, 2> Bd;
        Eigen::Matrix2d Rd;
        controller::mpc_model::initialize(Ad, Bd, Qd, Rd, P);
        const Eigen::Matrix<double, 2, 6> K =
            (Rd + Bd.transpose() * P * Bd).ldlt().solve(Bd.transpose() * P * Ad);
        Eigen::Matrix<double, 6, 1> error;
        error << -0.07, -0.02, 0.01, -0.015, -0.1, 0.03;
        const Eigen::Vector2d old_action = -K * error;
        const auto constant = mpc.compute_command(state, references);
        const auto wrapped_api = mpc.compute_command(state, reference);
        check(std::abs(constant.left_wheel_torque - old_action[0]) < 2e-5
              && std::abs(constant.right_wheel_torque - old_action[1]) < 2e-5,
              "Stationary reference differs from D11 regulation.");
        check(std::abs(constant.left_wheel_torque - wrapped_api.left_wheel_torque) < 2e-5
              && std::abs(constant.right_wheel_torque - wrapped_api.right_wheel_torque) < 2e-5,
              "Single reference wrapper differs from constant horizon.");
        std::cout << std::setprecision(12)
                  << "Constant old = " << old_action.transpose()
                  << " | preview = " << constant.left_wheel_torque << ", "
                  << constant.right_wheel_torque << '\n';

        const double expected_actions[3][2] = {
            {-1.063967408145692, -0.337346832374379},
            {-0.839021124039543, -0.112400548267249},
            {-0.795624159180824, -0.026170681100824}
        };
        for (int i = 0; i < 3; ++i) {
            auto nonstationary = reference;
            if (i == 0) nonstationary.forward_velocity = 0.1;
            if (i == 1) nonstationary.pitch = 0.02;
            if (i == 2) nonstationary.yaw_rate = 0.1;
            const auto action = mpc.compute_command(state, nonstationary);
            check(std::abs(action.left_wheel_torque - expected_actions[i][0]) < 2e-5
                  && std::abs(action.right_wheel_torque - expected_actions[i][1]) < 2e-5,
                  "Nonstationary reference does not use the physical tracking cost.");
        }

        state = robot_interface::RobotState{};
        reference = robot_interface::Reference{};
        const auto held = mpc.compute_command(state, reference);
        for (int i = 0; i < mpc.horizon_size(); ++i) {
            const double t = (i + 1) * mpc.sample_time();
            const double q = std::clamp((t - 0.15) / 0.35, 0.0, 1.0);
            references[i] = reference;
            references[i].yaw = 0.2 * q * q * (3.0 - 2.0 * q);
            references[i].yaw_rate = 0.2 * 6.0 * q * (1.0 - q) / 0.35;
        }
        const auto preview = mpc.compute_command(state, references);
        // Independent dense solve in tools/mpc_preview_test.py.
        check(std::abs(preview.left_wheel_torque - 0.000170694155239) < 2e-5
              && std::abs(preview.right_wheel_torque + 0.000170694155456) < 2e-5,
              "Preview action differs from the numerical tracking solution.");
        check(std::abs(preview.left_wheel_torque - held.left_wheel_torque) > 1e-4,
              "Future references did not affect the first command.");
        std::cout << "Held L/R = " << held.left_wheel_torque << ", " << held.right_wheel_torque
                  << " | future turn L/R = " << preview.left_wheel_torque << ", "
                  << preview.right_wheel_torque << '\n';

        state.yaw = 179.0 * std::acos(-1.0) / 180.0;
        for (auto& r : references) {
            r.yaw = robot_interface::wrap_to_pi(state.yaw + r.yaw);
            r.timestamp = -1234.0;
        }
        const auto crossing = mpc.compute_command(state, references);
        check(std::abs(crossing.left_wheel_torque - preview.left_wheel_torque) < 2e-5
              && std::abs(crossing.right_wheel_torque - preview.right_wheel_torque) < 2e-5,
              "Wrapped future yaw changed the physical tracking problem.");
        state.forward_position = 12.4;
        for (auto& r : references) r.forward_position += 12.4;
        const auto translated = mpc.compute_command(state, references);
        check(std::abs(translated.left_wheel_torque - crossing.left_wheel_torque) < 2e-5,
              "Common longitudinal origin changed the command.");

        bool rejected = false;
        references.pop_back();
        try { mpc.compute_command(state, references); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "Incorrect horizon length was accepted.");
        references.push_back(reference);
        references.back().yaw = std::numeric_limits<double>::quiet_NaN();
        rejected = false;
        try { mpc.compute_command(state, references); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "Non-finite future reference was accepted.");
        std::cout << "PASS stationary regression, future preview, yaw unwrap, origin, input validation\n";
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
