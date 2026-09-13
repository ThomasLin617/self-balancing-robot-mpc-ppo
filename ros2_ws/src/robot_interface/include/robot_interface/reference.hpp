#pragma once

namespace robot_interface
{

struct Reference
{
    double timestamp = 0.0;
    double forward_velocity = 0.0;
    double forward_position = 0.0;
    double yaw_rate = 0.0;
    double pitch = 0.0;
};

}