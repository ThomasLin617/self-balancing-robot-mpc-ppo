#include <iostream>
#include <cmath>
#include "mujoco_backend/mujoco_backend.hpp"
#include "mujoco_backend/mujoco_viewer.hpp"
#include "robot_interface/robot_command.hpp"

int main() {
    mujoco_backend::MujocoBackend backend(
        "/Users/thomas/robotics/ros2_mujoco_balancing_robot/model/Robot1.xml"
    );
    auto sensor_data = backend.read_sensor_data();
    std::cout << "Accel: ["
            << sensor_data.accelerometer[0] << ", "
            << sensor_data.accelerometer[1] << ", "
            << sensor_data.accelerometer[2] << "]" << std::endl;
    double pitch_from_accel = std::atan2(
        -sensor_data.accelerometer[0],
        sensor_data.accelerometer[2]
    );
    std::cout << "Pitch from accelerometer: " << pitch_from_accel << "rad" << std::endl;

    mujoco_backend::MujocoViewer viewer(backend);
    robot_interface::RobotCommand command;
    command.left_wheel_torque = 0.2;
    command.right_wheel_torque = 0.2;

    bool reset_done = false;

    while (!viewer.should_close()) {
        // Reset once after 2 seconds of simulation.
        if (!reset_done && backend.simulation_time() >= 2.0) {
            sensor_data = backend.read_sensor_data();
            std::cout << "===== Before Reset =====" << std::endl;
            std::cout << "Time: "
                      << backend.simulation_time()
                      << " s" << std::endl;
            std::cout << "Gyro: ["
                      << sensor_data.gyroscope[0] << ", "
                      << sensor_data.gyroscope[1] << ", "
                      << sensor_data.gyroscope[2] << "]" << std::endl;
            std::cout << "Wheel position (L/R): "
                      << sensor_data.left_wheel_position << " / "
                      << sensor_data.right_wheel_position << std::endl;
            std::cout << "Wheel velocity (L/R): "
                      << sensor_data.left_wheel_velocity << " / "
                      << sensor_data.right_wheel_velocity << std::endl;
            backend.reset();
            sensor_data = backend.read_sensor_data();
            std::cout << std::endl;
            std::cout << "===== After Reset =====" << std::endl;
            std::cout << "Time: "
                      << backend.simulation_time()
                      << " s" << std::endl;
            std::cout << "Gyro: ["
                      << sensor_data.gyroscope[0] << ", "
                      << sensor_data.gyroscope[1] << ", "
                      << sensor_data.gyroscope[2] << "]" << std::endl;
            std::cout << "Accel: ["
                      << sensor_data.accelerometer[0] << ", "
                      << sensor_data.accelerometer[1] << ", "
                      << sensor_data.accelerometer[2] << "]" << std::endl;
            std::cout << "Wheel position (L/R): "
                      << sensor_data.left_wheel_position << " / "
                      << sensor_data.right_wheel_position << std::endl;
            std::cout << "Wheel velocity (L/R): "
                      << sensor_data.left_wheel_velocity << " / "
                      << sensor_data.right_wheel_velocity << std::endl;
            // Stop applying torque after reset.
            command.left_wheel_torque = 0.0;
            command.right_wheel_torque = 0.0;
            reset_done = true;
        }

        double frame_start = backend.simulation_time();
        while (backend.simulation_time() - frame_start < 1.0 / 60.0) {
            backend.apply_command(command);
            backend.step();
        }
        viewer.render();

    }

    return 0;
}