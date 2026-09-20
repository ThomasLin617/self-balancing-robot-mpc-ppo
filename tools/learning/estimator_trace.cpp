#include <state_estimator/state_estimator.hpp>
#include <navigation/planar_odometry.hpp>
#include <iomanip>
#include <iostream>

int main() {
    state_estimator::StateEstimator estimator(0.5, 0.12);
    navigation::PlanarOdometry odometry;
    robot_interface::SensorData sensor;
    std::cout << std::setprecision(17);
    while (std::cin >> sensor.timestamp) {
        for (double& value : sensor.gyroscope) std::cin >> value;
        for (double& value : sensor.accelerometer) std::cin >> value;
        std::cin >> sensor.left_wheel_position >> sensor.right_wheel_position
                 >> sensor.left_wheel_velocity >> sensor.right_wheel_velocity;
        if (!std::cin) return 1;
        const auto state = estimator.update(sensor);
        const auto pose = odometry.update(state);
        std::cout << state.forward_position << ' ' << state.forward_velocity << ' '
                  << state.pitch << ' ' << state.pitch_rate << ' '
                  << state.yaw << ' ' << state.yaw_rate << ' '
                  << pose.x << ' ' << pose.y << '\n';
    }
    return 0;
}
