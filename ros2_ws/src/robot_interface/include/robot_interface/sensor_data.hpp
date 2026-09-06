#pragma once
#include <array>

namespace robot_interface
{

struct SensorData
{
    double timestamp = 0.0;

    std::array<double, 3> gyroscope = {};
    std::array<double, 3> accelerometer = {};

    double left_wheel_position = 0.0;
    double right_wheel_position = 0.0;
    double left_wheel_velocity = 0.0;
    double right_wheel_velocity = 0.0;
};
}