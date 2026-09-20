#include <cmath>
#include <iostream>
#include <stdexcept>
#include <mujoco_backend/mujoco_backend.hpp>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: yaw_sign_smoke_test <model_path>\n";
        return 1;
    }

    try {
        mujoco_backend::MujocoBackend backend(argv[1]);
        for (double direction : {-1.0, 1.0}) {
            backend.reset();
            robot_interface::RobotCommand command;
            command.left_wheel_torque = -0.02 * direction;
            command.right_wheel_torque = 0.02 * direction;
            for (int i = 0; i < 100; ++i) {
                backend.apply_command(command);
                backend.step();
            }
            const double yaw_rate = backend.read_sensor_data().gyroscope[2];
            std::cout << "L = " << command.left_wheel_torque
                      << " | R = " << command.right_wheel_torque
                      << " | gyro Z = " << yaw_rate << '\n';
            if (!std::isfinite(yaw_rate) || direction * yaw_rate <= 0.001) {
                throw std::runtime_error("Differential torque yaw sign is incorrect.");
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
