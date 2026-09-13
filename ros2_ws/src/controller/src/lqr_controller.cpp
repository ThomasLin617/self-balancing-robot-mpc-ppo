#include <controller/lqr_controller.hpp>

namespace controller
{
    LQRController::LQRController(const std::array<double, 4>& gains)
    : gains_(gains)
    {
    }
 
    robot_interface::RobotCommand LQRController::compute_command(
        const robot_interface::RobotState& state,
        const robot_interface::Reference& reference
    ) const
    {
        const double position_error =
            state.forward_position - reference.forward_position;

        const double velocity_error =
            state.forward_velocity - reference.forward_velocity;

        const double pitch_error =
            state.pitch - reference.pitch;

        const double pitch_rate_error =
            state.pitch_rate;

        const double k_position = gains_[0];
        const double k_velocity = gains_[1];
        const double k_pitch = gains_[2];
        const double k_pitch_rate = gains_[3];

        const double feedback =
            k_position * position_error
            + k_velocity * velocity_error
            + k_pitch * pitch_error
            + k_pitch_rate * pitch_rate_error;
        const double torque = -feedback;


        robot_interface::RobotCommand command;
        command.timestamp = state.timestamp;
        command.left_wheel_torque = torque;
        command.right_wheel_torque = torque;
        return command;
    }
} // namespace controller
