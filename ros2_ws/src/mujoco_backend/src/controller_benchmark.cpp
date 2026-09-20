#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <controller/lqr_controller.hpp>
#include <controller/mpc_controller.hpp>
#include <learning_policy/ppo_policy.hpp>
#include <mujoco_backend/mujoco_backend.hpp>
#include <navigation/planar_odometry.hpp>
#include <navigation/reference_preview.hpp>
#include <state_estimator/state_estimator.hpp>

using Clock = std::chrono::steady_clock;

double microseconds(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::micro>(end - start).count();
}

int main(int argc, char** argv) {
    if (argc != 5 && argc != 6) {
        std::cerr << "Usage: controller_benchmark model actor job.txt output.csv [diagnostics.csv]\n";
        return 1;
    }
    try {
        std::ifstream job(argv[3]);
        std::string method, group;
        double gx, gy, duration, vmax, acceleration, force, push_time, push_duration;
        double radius, velocity_limit, yaw_limit, pitch_limit, hold, fall_pitch, dt;
        int control_steps;
        if (!(job >> method >> group >> gx >> gy >> duration >> vmax >> acceleration
              >> force >> push_time >> push_duration >> radius >> velocity_limit >> yaw_limit
              >> pitch_limit >> hold >> fall_pitch >> dt >> control_steps)) {
            throw std::runtime_error("Malformed benchmark job.");
        }
        if ((method != "lqr" && method != "mpc" && method != "ppo") || dt != 0.001
            || control_steps != 2 || duration <= 0.0) throw std::runtime_error("Invalid benchmark schedule/method.");
        if (argc == 6 && (method != "mpc" || group != "goal_reaching")) {
            throw std::runtime_error("Trajectory diagnostics require an MPC goal-reaching job.");
        }
        mujoco_backend::MujocoBackend backend(argv[1]);
        state_estimator::StateEstimator estimator(0.5, 0.12);
        navigation::PlanarOdometry odometry;
        controller::LQRController lqr({-6.66667, -6.57554, -18.36740, -3.00994});
        std::unique_ptr<controller::MPCController> mpc;
        std::unique_ptr<learning_policy::PPOPolicy> policy;
        std::unique_ptr<learning_policy::GoalObservationBuilder> builder;
        if (method == "mpc") mpc = std::make_unique<controller::MPCController>(250, 2.0);
        if (method == "ppo") {
            policy = std::make_unique<learning_policy::PPOPolicy>(argv[2]);
            builder = std::make_unique<learning_policy::GoalObservationBuilder>(policy->observation_config());
        }
        navigation::ReferenceTrajectory trajectory(vmax, acceleration);
        robot_interface::ReferenceHorizon horizon(250);
        robot_interface::Reference reference;
        reference.forward_position = gx;
        robot_interface::RobotCommand command;
        backend.reset();
        estimator.reset();
        odometry.reset();
        auto state = estimator.update(backend.read_sensor_data());
        auto pose = odometry.update(state);
        const std::array<double, 2> goal{pose.x + gx, pose.y + gy};
        const double origin = state.forward_position;
        if (method == "mpc" && group != "sagittal") trajectory.generate(pose, goal[0], goal[1]);
        std::ofstream output(argv[4]);
        if (!output) throw std::runtime_error("Cannot create benchmark trace.");
        output << std::setprecision(17);
        output << "time,p,v,pitch,pitch_rate,yaw_rate,x,y,true_x,true_y,goal_distance,true_distance,requested_l,requested_r,applied_l,applied_r,force,success,fall,full_us,core_us\n";
        std::ofstream diagnostics;
        if (argc == 6) {
            diagnostics.open(argv[5]);
            if (!diagnostics) throw std::runtime_error("Cannot create audit diagnostics.");
            diagnostics << std::setprecision(17)
                        << "time,yaw,true_yaw,reference_yaw,reference_yaw_rate,preview_yaw,preview_yaw_rate,left_wheel_velocity,right_wheel_velocity\n";
        }
        int settled_steps = 0;
        for (int step = 0; step < static_cast<int>(std::llround(duration / dt)); ++step) {
            double full_us = -1.0, core_us = -1.0;
            if (step % control_steps == 0) {
                const auto begin = Clock::now();
                if (method == "lqr") {
                    command = lqr.compute_command(state, reference);
                    full_us = microseconds(begin, Clock::now());
                    core_us = full_us;
                } else if (method == "mpc") {
                    if (group != "sagittal") navigation::sample_reference_horizon(trajectory, state.timestamp, origin, mpc->sample_time(), horizon);
                    const auto core_begin = Clock::now();
                    command = group == "sagittal" ? mpc->compute_command(state, reference) : mpc->compute_command(state, horizon);
                    const auto end = Clock::now();
                    core_us = microseconds(core_begin, end);
                    full_us = microseconds(begin, end);
                } else {
                    const auto torque = policy->torque(state, pose, goal, *builder);
                    command.left_wheel_torque = torque[0];
                    command.right_wheel_torque = torque[1];
                    full_us = microseconds(begin, Clock::now());
                    const auto observation = builder->build(state, pose, goal);
                    const auto actor_begin = Clock::now();
                    const auto action = policy->normalized_action(observation);
                    core_us = microseconds(actor_begin, Clock::now());
                    if (!std::isfinite(action[0]) || !std::isfinite(action[1])) throw std::runtime_error("Invalid actor output.");
                }
            }
            if (!std::isfinite(command.left_wheel_torque) || !std::isfinite(command.right_wheel_torque)) throw std::runtime_error("Invalid torque.");
            const double applied_l = std::clamp(command.left_wheel_torque, -2.0, 2.0);
            const double applied_r = std::clamp(command.right_wheel_torque, -2.0, 2.0);
            const double push = state.timestamp + 1e-9 >= push_time && state.timestamp + 1e-9 < push_time + push_duration ? force : 0.0;
            backend.set_body_forward_force("base_body", push);
            backend.apply_command(command);
            backend.step();
            state = estimator.update(backend.read_sensor_data());
            pose = odometry.update(state);
            const double distance = std::hypot(goal[0] - pose.x, goal[1] - pose.y);
            const bool settled = distance < radius && std::abs(state.forward_velocity) < velocity_limit
                && std::abs(state.yaw_rate) < yaw_limit && std::abs(state.pitch) < pitch_limit;
            settled_steps = settled ? settled_steps + 1 : 0;
            const bool fall = std::abs(state.pitch) > fall_pitch;
            const bool success = !fall && settled_steps * dt + 1e-12 >= hold;
            const auto truth = backend.debug_planar_pose("base_body");
            output << state.timestamp << ',' << state.forward_position << ',' << state.forward_velocity << ','
                   << state.pitch << ',' << state.pitch_rate << ',' << state.yaw_rate << ',' << pose.x << ',' << pose.y << ','
                   << truth[0] << ',' << truth[1] << ',' << distance << ',' << std::hypot(goal[0]-truth[0],goal[1]-truth[1]) << ','
                   << command.left_wheel_torque << ',' << command.right_wheel_torque << ',' << applied_l << ',' << applied_r << ','
                   << push << ',' << success << ',' << fall << ',' << full_us << ',' << core_us << '\n';
            if (diagnostics.is_open()) {
                const auto current = trajectory.sample(state.timestamp);
                const auto future = trajectory.sample(state.timestamp + 0.5);
                diagnostics << state.timestamp << ',' << state.yaw << ',' << truth[2] << ','
                            << current.yaw_ref << ',' << current.yaw_rate_ref << ','
                            << future.yaw_ref << ',' << future.yaw_rate_ref << ','
                            << state.left_wheel_velocity << ',' << state.right_wheel_velocity << '\n';
            }
            if (!std::isfinite(state.pitch) || !std::isfinite(distance)) throw std::runtime_error("Invalid estimated state.");
            const bool after_push = group != "disturbance" || state.timestamp >= push_time + push_duration + hold;
            if (fall || (success && after_push && group != "sagittal" && group != "compute")) break;
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
