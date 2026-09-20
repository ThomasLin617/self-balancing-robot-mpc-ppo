#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <controller/lqr_controller.hpp>
#include <navigation/reference_trajectory.hpp>

using Clock = std::chrono::steady_clock;

int main(int argc, char** argv) {
    try {
        if (argc != 7) throw std::runtime_error(
            "Usage: benchmark_audit_probe lqr K batches warmup unused output.csv\n"
            "       benchmark_audit_probe trajectory gx gy vmax acceleration output.csv");
        std::ofstream output(argv[6]);
        if (!output) throw std::runtime_error("Cannot open audit output.");
        output << std::setprecision(17);
        const std::string mode(argv[1]);
        if (mode == "trajectory") {
            navigation::ReferenceTrajectory trajectory(std::stod(argv[4]), std::stod(argv[5]));
            trajectory.generate({}, std::stod(argv[2]), std::stod(argv[3]));
            output << "time,duration,x,y,s,v,yaw,yaw_rate,curvature\n";
            const int steps = static_cast<int>(std::ceil(trajectory.duration() / 0.002));
            for (int i = 0; i <= steps; ++i) {
                const double time = std::min(i * 0.002, trajectory.duration());
                const auto point = trajectory.sample(time);
                output << time << ',' << trajectory.duration() << ',' << point.x_ref << ',' << point.y_ref << ','
                       << point.forward_position_ref << ',' << point.forward_velocity_ref << ','
                       << point.yaw_ref << ',' << point.yaw_rate_ref << ',' << point.curvature << '\n';
            }
        } else if (mode == "lqr") {
            const int updates = std::stoi(argv[2]);
            const int batches = std::stoi(argv[3]);
            const int warmup = std::stoi(argv[4]);
            if (updates < 1 || batches < 1 || warmup < 0) throw std::runtime_error("Invalid batch counts.");
            controller::LQRController lqr({-6.66667, -6.57554, -18.36740, -3.00994});
            std::vector<robot_interface::RobotState> states(updates);
            std::vector<robot_interface::Reference> references(updates);
            std::vector<robot_interface::RobotCommand> commands(updates);
            for (int i = 0; i < updates; ++i) {
                const double fraction = static_cast<double>(i % 101) / 100.0;
                states[i].forward_position = 0.02 * fraction;
                states[i].forward_velocity = 0.03 * (fraction - 0.5);
                states[i].pitch = 0.01 * (fraction - 0.5);
                states[i].pitch_rate = 0.02 * fraction;
                references[i].forward_position = 0.09 + 0.02 * fraction;
            }
            output << "batch,updates,total_us,per_update_us,checksum\n";
            for (int batch = -warmup; batch < batches; ++batch) {
                states[0].forward_position = 1e-6 * (batch + warmup);
                const auto begin = Clock::now();
                for (int i = 0; i < updates; ++i) commands[i] = lqr.compute_command(states[i], references[i]);
                const double elapsed = std::chrono::duration<double, std::micro>(Clock::now() - begin).count();
                double checksum = 0.0;
                for (const auto& command : commands) checksum += command.left_wheel_torque + command.right_wheel_torque;
                if (!std::isfinite(checksum)) throw std::runtime_error("Invalid LQR output.");
                if (batch >= 0) output << batch << ',' << updates << ',' << elapsed << ',' << elapsed / updates << ',' << checksum << '\n';
            }
        } else throw std::runtime_error("Unknown audit mode.");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
