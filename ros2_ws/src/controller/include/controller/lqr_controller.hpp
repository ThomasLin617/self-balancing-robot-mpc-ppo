#pragma once
#include <array>
#include <robot_interface/robot_state.hpp>
#include <robot_interface/robot_command.hpp>
#include <robot_interface/reference.hpp>
namespace controller
{
    class LQRController
    {
    public:
        explicit LQRController(const std::array<double, 4>& gains);
        robot_interface::RobotCommand compute_command(
            const robot_interface::RobotState& state,
            const robot_interface::Reference& reference
        ) const;
    private:
        std::array<double, 4> gains_;
    };
}