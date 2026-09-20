#include <fstream>
#include <iomanip>
#include <controller/mpc_controller.hpp>
#include <state_estimator/state_estimator.hpp>
#include <mujoco_backend/mujoco_backend.hpp>

int main(int argc, char** argv) {
    if (argc != 4) {
        return 1;
    }
    mujoco_backend::MujocoBackend backend(argv[1]);
    state_estimator::StateEstimator estimator(.5, .12);
    controller::MPCController controller(250, 2.0);
    robot_interface::Reference reference;
    reference.forward_position = .1;
    robot_interface::RobotCommand command;
    backend.reset();
    std::ofstream output(argv[3]);
    output << std::setprecision(17)
           << "time,p,v,pitch,pitch_rate,yaw,yaw_rate,left,right,applied_l,applied_r\n";
    const int steps = static_cast<int>(std::llround(std::stod(argv[2]) * 1000));
    for (int i = 0; i < steps; ++i) {
        const auto state = estimator.update(backend.read_sensor_data());
        if (i % 2 == 0) {
            command = controller.compute_command(state, reference);
        }
        output << state.timestamp << ',' << state.forward_position << ',' << state.forward_velocity << ','
               << state.pitch << ',' << state.pitch_rate << ',' << state.yaw << ',' << state.yaw_rate << ','
               << command.left_wheel_torque << ',' << command.right_wheel_torque << ','
               << std::clamp(command.left_wheel_torque, -2.0, 2.0) << ','
               << std::clamp(command.right_wheel_torque, -2.0, 2.0) << '\n';
        backend.apply_command(command);
        backend.step();
    }
}
