#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <controller/mpc_controller.hpp>

int main() {
    try {
        controller::MPCController controller(250, 2.0);
        robot_interface::RobotState state;
        robot_interface::Reference reference;
        constexpr double pi = 3.14159265358979323846;
        state.yaw = 179.0 * pi / 180.0;
        reference.yaw = -179.0 * pi / 180.0;
        const auto wrapped = controller.compute_command(state, reference);
        if (!(wrapped.left_wheel_torque < 0.0 && wrapped.right_wheel_torque > 0.0)
            || std::abs(wrapped.left_wheel_torque) > 0.2) {
            throw std::runtime_error("MPC did not use the shortest yaw error.");
        }
        reference.yaw += 2.0 * pi;
        const auto equivalent = controller.compute_command(state, reference);
        if (std::abs(wrapped.left_wheel_torque - equivalent.left_wheel_torque) > 1e-5) {
            throw std::runtime_error("Equivalent yaw references changed the command.");
        }
        state = robot_interface::RobotState{};
        reference.yaw = 1.0;
        reference.forward_position = 0.5;
        const auto saturated = controller.compute_command(state, reference);
        std::cout << std::setprecision(15) << "Saturated L/R = " << saturated.left_wheel_torque
                  << " / " << saturated.right_wheel_torque << '\n';
        if (std::abs(saturated.left_wheel_torque) > 2.000001
            || std::abs(saturated.right_wheel_torque) > 2.000001) {
            throw std::runtime_error("MPC saturation constraint failed.");
        }
        std::cout << "PASS MPC yaw wrapping and saturation\n";
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
