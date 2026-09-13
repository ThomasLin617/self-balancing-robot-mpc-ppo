#include <controller/pd_controller.hpp>
#include <stdexcept>

namespace controller
{
    PDController::PDController(double kp, double kd)
        :kp_(kp), kd_(kd)
    {
        if(kp < 0){
            throw std::invalid_argument("Kp must be non-negative.");
        }
        if(kd < 0){
            throw std::invalid_argument("Kd must be non-negative. ");
        }
    }
    robot_interface::RobotCommand PDController::compute_command(
            const robot_interface::RobotState& robot_state,
            const robot_interface::Reference& reference
        ) const
    {
        double error_pitch = reference.pitch - robot_state.pitch;
        double error_pitch_rate = -robot_state.pitch_rate;
        double torque_command = -kp_ * error_pitch - kd_ * error_pitch_rate;

        robot_interface::RobotCommand command;
        command.left_wheel_torque = torque_command;
        command.right_wheel_torque = torque_command;
        command.timestamp = robot_state.timestamp;
        return command;
    }


} // namespace controller