#pragma once

#include <robot_interface/robot_state.hpp>

namespace navigation
{

struct PlanarPose
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

class PlanarOdometry
{
public:
    PlanarPose update(const robot_interface::RobotState& state);
    void reset();

private:
    PlanarPose pose_;
    double previous_timestamp_ = 0.0;
    double previous_velocity_ = 0.0;
    bool initialized_ = false;
};

}
