#pragma once

namespace robot_interface
{
struct RobotCommand
{
    double timestamp = 0.0;
    double left_wheel_torque = 0.0;
    double right_wheel_torque = 0.0;
};


}