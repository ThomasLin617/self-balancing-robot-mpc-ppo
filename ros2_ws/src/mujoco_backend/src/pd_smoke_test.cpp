#include <iostream>
#include <iomanip>
#include <string>

#include <mujoco_backend/mujoco_backend.hpp>
#include <mujoco_backend/mujoco_viewer.hpp>
#include <state_estimator/state_estimator.hpp>

#include <robot_interface/robot_command.hpp>
#include <robot_interface/robot_state.hpp>
#include <robot_interface/sensor_data.hpp>
#include <robot_interface/reference.hpp>

#include <controller/pd_controller.hpp>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: pd_smoke_test <model_path>\n";
        return 1;
    }

    const std::string model_path = argv[1];

    try {
        mujoco_backend::MujocoBackend backend(model_path);
        mujoco_backend::MujocoViewer viewer(backend);
        viewer.track_body("base_body");
        viewer.set_camera_azimuth(135.0);  // Front three-quarter view (+X is forward).
        viewer.enable_studio_lighting();

        state_estimator::StateEstimator estimator(
            0.5,   // complementary-filter time constant [s]
            0.12   // wheel radius [m]
        );

        controller::PDController pd_controller(
            5.0,   // Kp - temporary
            0.2    // Kd - temporary
        );

        robot_interface::Reference reference;
        reference.pitch = 0.0;

        robot_interface::RobotCommand command;

        backend.reset();
        estimator.reset();

        constexpr int controller_interval_steps = 2;  // 1000 Hz physics -> 500 Hz controller
        constexpr int print_interval_steps = 100;     // ~10 Hz print
        constexpr int render_interval_steps = 16;     // ~60 Hz render

        int step_count = 0;
        constexpr double push_interval = 5.0;  // simulation seconds
        constexpr double push_duration = 0.05; // 50 ms pulse
        constexpr double push_force = -50.0;   // N: from the front, toward local -X
        double next_push_time = push_interval;
        double push_end_time = 0.0;

        std::cout << std::fixed << std::setprecision(4);

        while (!viewer.should_close()) {

            robot_interface::SensorData sensor_data = backend.read_sensor_data();
            robot_interface::RobotState state = estimator.update(sensor_data);
            if(step_count % controller_interval_steps == 0){
                command = pd_controller.compute_command(state, reference);
            }

            backend.apply_command(command);
            const double now = backend.simulation_time();
            if (now + 1e-9 >= next_push_time) {
                push_end_time = next_push_time + push_duration;
                next_push_time += push_interval;
                std::cout << "Front push at t = " << now << " s\n";
            }
            backend.set_body_forward_force(
                "base_body", now + 1e-9 < push_end_time ? push_force : 0.0);
            backend.step();
            
            if (step_count % print_interval_steps == 0) {
                std::cout
                    << "t = " << state.timestamp
                    << " | pitch = " << state.pitch
                    << " rad"
                    << " | pitch_rate = " << state.pitch_rate
                    << " rad/s"
                    << " | velocity = " << state.forward_velocity
                    << " m/s"
                    << " | torque = " << command.left_wheel_torque
                    << " Nm"
                    << '\n';
            }

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
