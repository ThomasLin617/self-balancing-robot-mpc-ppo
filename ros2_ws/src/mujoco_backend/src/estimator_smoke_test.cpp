#include <iostream>
#include <iomanip>
#include <string>

#include <mujoco_backend/mujoco_backend.hpp>
#include <mujoco_backend/mujoco_viewer.hpp>
#include <state_estimator/state_estimator.hpp>
#include <robot_interface/robot_command.hpp>
#include <robot_interface/robot_state.hpp>
#include <robot_interface/sensor_data.hpp>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr
            << "Usage: estimator_smoke_test <model_path>\n";
        return 1;
    }

    const std::string model_path = argv[1];

    try {
        mujoco_backend::MujocoBackend backend(model_path);
        mujoco_backend::MujocoViewer viewer(backend);

        state_estimator::StateEstimator estimator(
            0.5,   // pitch complementary-filter time constant [s]
            0.12   // wheel radius [m]
        );

        robot_interface::RobotCommand command;

        backend.reset();
        estimator.reset();

        constexpr int print_interval_steps = 100;
        constexpr int render_interval_steps = 16;

        int step_count = 0;

        std::cout << std::fixed << std::setprecision(4);

        while (!viewer.should_close()) {
            const double time = backend.simulation_time();

            // ---------------------------------------------------------
            // Simple test command
            //
            // 0–1 s : no torque
            // >=1 s : equal wheel torque
            // ---------------------------------------------------------
            command.timestamp = time;

            if (time < 1.0) {
                command.left_wheel_torque = 0.0;
                command.right_wheel_torque = 0.0;
            }
            else {
                command.left_wheel_torque = 0.2;
                command.right_wheel_torque = 0.2;
            }

            backend.apply_command(command);

            // Advance physics by one MuJoCo timestep.
            backend.step();

            // ---------------------------------------------------------
            // Sensor -> estimator -> robot state
            // ---------------------------------------------------------
            robot_interface::SensorData sensor_data =
                backend.read_sensor_data();

            robot_interface::RobotState state =
                estimator.update(sensor_data);

            // ---------------------------------------------------------
            // Print estimator output every 100 physics steps
            // ≈ every 0.1 s for a 1 kHz simulation
            // ---------------------------------------------------------
            if (step_count % print_interval_steps == 0) {
                std::cout
                    << "t = " << state.timestamp
                    << " | pitch = " << state.pitch
                    << " rad"
                    << " | pitch_rate = " << state.pitch_rate
                    << " rad/s"
                    << " | velocity = " << state.forward_velocity
                    << " m/s"
                    << " | yaw_rate = " << state.yaw_rate
                    << " rad/s"
                    << '\n';
            }

            // Rendering does not need to run at physics frequency.
            if (step_count % render_interval_steps == 0) {
                viewer.render();
            }

            ++step_count;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}