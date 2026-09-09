#include <iostream>
#include "mujoco_backend/mujoco_backend.hpp"
#include "robot_interface/robot_command.hpp"
#include "mujoco_backend/mujoco_viewer.hpp"
int main(){
    mujoco_backend::MujocoBackend backend("/Users/thomas/robotics/ros2_mujoco_balancing_robot/model/Robot1.xml");
    auto sensor_data = backend.read_sensor_data();
    robot_interface::RobotCommand command;
    std::cout<<backend.simulation_time()<<std::endl;
    std::cout<<sensor_data.gyroscope[0]<<std::endl;
    std::cout<<sensor_data.gyroscope[1]<<std::endl;
    std::cout<<sensor_data.gyroscope[2]<<std::endl;

    backend.step();
    sensor_data = backend.read_sensor_data();
    std::cout<<backend.simulation_time()<<std::endl;
    std::cout<<sensor_data.gyroscope[0]<<std::endl;
    std::cout<<sensor_data.gyroscope[1]<<std::endl;
    std::cout<<sensor_data.gyroscope[2]<<std::endl;
    for (int i = 0; i < 1000; i++) {
    command.left_wheel_torque  = 0.2;
    command.right_wheel_torque = -0.2;
    backend.apply_command(command);
    backend.step();
    sensor_data = backend.read_sensor_data();
    std::cout << "===== Step " << i << " =====" << std::endl;
    std::cout << "Time: " << backend.simulation_time() << " s" << std::endl;

    std::cout << "Gyro:  ["
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
    std::cout << "Motor torque (L/R): "
                << command.left_wheel_torque << " / "
                << command.right_wheel_torque << std::endl;

    std::cout << std::endl; 
    }

    // mujoco_backend::MujocoViewer viewer(backend);
    // while (!viewer.should_close()) {
    // double frame_start = backend.simulation_time();
    // while (backend.simulation_time() - frame_start < 1.0 / 60.0) {
    //         command.left_wheel_torque  = -0.2;
    //         command.right_wheel_torque = 0.2;
    //         backend.apply_command(command);
    //         backend.step();
    //     }

    //     viewer.render();
    // }
}