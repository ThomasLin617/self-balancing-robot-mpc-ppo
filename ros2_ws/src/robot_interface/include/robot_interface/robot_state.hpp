#pragma once

namespace robot_interface
{
struct RobotState
{
    double timestamp = 0.0;

    double pitch = 0.0;
    double pitch_rate = 0.0;

    double forward_velocity = 0.0;
    double forward_position = 0.0;
    double yaw_rate = 0.0;

    double left_wheel_position = 0.0;
    double right_wheel_position = 0.0;

    double left_wheel_velocity = 0.0;
    double right_wheel_velocity = 0.0;
};


}