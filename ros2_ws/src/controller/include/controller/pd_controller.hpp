#pragma once
#include <robot_interface/robot_state.hpp>
#include <robot_interface/robot_command.hpp>
#include <robot_interface/reference.hpp>
namespace controller
{
    class PDController
    {
    public:
        PDController(double kp, double kd);
        robot_interface::RobotCommand compute_command(
            const robot_interface::RobotState& robot_state,
            const robot_interface::Reference& reference
        ) const;
    private:
        double kp_ = 0.0;
        double kd_ = 0.0;

    };
    
}