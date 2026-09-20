#include <cmath>
#include <iostream>
#include <stdexcept>
#include <robot_interface/angle.hpp>
#include <state_estimator/state_estimator.hpp>

int main() {
    try {
        constexpr double pi = 3.14159265358979323846;
        state_estimator::StateEstimator estimator(0.5, 0.12);
        robot_interface::SensorData sensor;
        sensor.accelerometer[2] = 9.81;
        sensor.gyroscope[2] = 1.0;
        estimator.update(sensor);
        for (int i = 1; i <= 7000; ++i) {
            sensor.timestamp = i * 0.001;
            const auto state = estimator.update(sensor);
            const double expected = robot_interface::wrap_to_pi(sensor.timestamp);
            if (std::abs(state.yaw - expected) > 1e-10
                || state.yaw < -pi || state.yaw >= pi || state.yaw_rate != 1.0) {
                throw std::runtime_error("Yaw integration or wrapping failed.");
            }
        }
        estimator.reset();
        sensor.timestamp = 0.0;
        if (estimator.update(sensor).yaw != 0.0) {
            throw std::runtime_error("Yaw reset failed.");
        }
        bool rejected = false;
        try { estimator.update(sensor); }
        catch (const std::runtime_error&) { rejected = true; }
        if (!rejected) throw std::runtime_error("Repeated timestamp was accepted.");
        const double error = robot_interface::wrap_to_pi(358.0 * pi / 180.0);
        if (std::abs(error + 2.0 * pi / 180.0) > 1e-12) {
            throw std::runtime_error("Yaw boundary error is incorrect.");
        }
        std::cout << "PASS yaw integration, wrap, reset, timestamp validation\n";
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
