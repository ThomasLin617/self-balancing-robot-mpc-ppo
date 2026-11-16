#include <algorithm>
#include <cmath>
#include <controller/mpc_controller.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <learning_policy/trajectory_policy.hpp>
#include <memory>
#include <mujoco_backend/known_map_world.hpp>
#include <mujoco_backend/mujoco_backend.hpp>
#include <mujoco_backend/mujoco_viewer.hpp>
#include <navigation/known_map.hpp>
#include <navigation/path_trajectory.hpp>
#include <navigation/reference_preview.hpp>
#include <sstream>
#include <state_estimator/state_estimator.hpp>
#include <unistd.h>

class UprightModel {
  public:
    explicit UprightModel(const std::string &source) {
        std::ifstream input(source);
        std::ostringstream buffer;
        buffer << input.rdbuf();
        auto xml = buffer.str();
        const std::string tilted = "euler=\"0 -20 0\"";
        const auto index = xml.find(tilted);
        if (index == std::string::npos)
            throw std::runtime_error("Unexpected initial robot pose.");
        xml.replace(index, tilted.size(), "euler=\"0 0 0\"");
        path = (std::filesystem::temp_directory_path() /
                ("d18-upright-" + std::to_string(getpid()) + ".xml"))
                   .string();
        std::ofstream output(path);
        output << xml;
        if (!output)
            throw std::runtime_error("Cannot write nominal reset model.");
    }
    ~UprightModel() {
        std::remove(path.c_str());
    }
    std::string path;
};

int main(int argc, char **argv) {
    if (argc < 4) {
        std::cerr << "Usage: trajectory_policy_smoke_test model scenario actor|mpc [--headless] "
                     "[--trace file]\n";
        return 1;
    }
    try {
        bool headless = false;
        std::string trace;
        for (int i = 4; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--headless")
                headless = true;
            else if (arg == "--trace" && i + 1 < argc)
                trace = argv[++i];
            else
                throw std::invalid_argument("Unknown smoke argument.");
        }
        UprightModel upright(argv[1]);
        const auto scenario = navigation::KnownMap::load(argv[2]);
        mujoco_backend::KnownMapWorld world(upright.path, scenario);
        mujoco_backend::MujocoBackend backend(world.path());
        state_estimator::StateEstimator estimator(.5, .12);
        navigation::PlanarOdometry odometry;
        std::unique_ptr<learning_policy::TrajectoryPolicy> policy;
        std::unique_ptr<controller::MPCController> mpc;
        if (std::string(argv[3]) == "mpc")
            mpc = std::make_unique<controller::MPCController>(250, 2.0);
        else
            policy = std::make_unique<learning_policy::TrajectoryPolicy>(argv[3]);
        std::unique_ptr<mujoco_backend::MujocoViewer> viewer;
        if (!headless) {
            viewer = std::make_unique<mujoco_backend::MujocoViewer>(backend);
            viewer->set_camera_overview({1.2, 0, 0}, 5.0);
            viewer->set_camera_azimuth(135);
            viewer->enable_studio_lighting();
        }
        backend.reset();
        auto state = estimator.update(backend.read_sensor_data());
        auto pose = odometry.update(state);
        const navigation::MotionLimits limits;
        const auto safe = scenario.grid.inflated(limits.robot_radius + limits.safety_margin);
        const auto route = scenario.grid.inflated(limits.robot_radius + limits.safety_margin +
                                                  limits.smoothing_clearance);
        const auto raw = navigation::AStarPlanner().plan(route, {pose.x, pose.y}, scenario.goal);
        const auto waypoints = navigation::simplify_path(route, raw);
        const auto timed = navigation::time_parameterize(
            navigation::smooth_path(safe, waypoints, pose.yaw, limits), limits);
        navigation::validate_trajectory(safe, timed);
        navigation::ReferenceTrajectory trajectory(timed);
        const double origin = state.forward_position;
        if (viewer) {
            std::vector<std::array<double, 3>> line, cells, corners;
            for (const auto &p : raw)
                cells.push_back({p[0], p[1], .03});
            for (const auto &p : waypoints)
                corners.push_back({p[0], p[1], .045});
            const auto &points = trajectory.points();
            const auto stride = (points.size() + 498) / 499;
            for (std::size_t i = 0; i < points.size(); i += stride)
                line.push_back({points[i].x_ref, points[i].y_ref, .015});
            line.push_back({points.back().x_ref, points.back().y_ref, .015});
            viewer->set_reference_path(line, {scenario.goal[0], scenario.goal[1], .055});
            viewer->set_planned_route(cells, corners);
        }
        std::ofstream output;
        if (!trace.empty()) {
            output.open(trace);
            output << std::setprecision(17)
                   << "time,p,v,pitch,pitch_rate,yaw,yaw_rate,x,y,left,right\n";
        }
        robot_interface::ReferenceHorizon refs(250);
        robot_interface::RobotCommand command;
        int held = 0;
        double maximum_pitch = 0, maximum_torque = 0;
        for (int step = 0;; ++step) {
            if (viewer && viewer->should_close())
                return 2;
            if (step > 0) {
                state = estimator.update(backend.read_sensor_data());
                pose = odometry.update(state);
            }
            if (step % 2 == 0) {
                navigation::sample_reference_horizon(trajectory, state.timestamp, origin, .002,
                                                     refs);
                if (mpc)
                    command = mpc->compute_command(state, refs);
                else {
                    const auto u = policy->torque(state, refs);
                    command.left_wheel_torque = u[0];
                    command.right_wheel_torque = u[1];
                }
                command.timestamp = state.timestamp;
                const double distance =
                    std::hypot(pose.x - scenario.goal[0], pose.y - scenario.goal[1]);
                const bool settled = state.timestamp >= trajectory.duration() && distance < .1 &&
                                     std::abs(state.forward_velocity) < .05 &&
                                     std::abs(state.yaw_rate) < .1 && std::abs(state.pitch) < .1;
                held = settled ? held + 1 : 0;
                if (held * .002 >= .5) {
                    const auto truth = backend.debug_planar_pose("base_body");
                    std::cout << std::setprecision(12) << "PASS D18 | time=" << state.timestamp
                              << " | estimated_distance=" << distance << " | true_distance="
                              << std::hypot(truth[0] - scenario.goal[0],
                                            truth[1] - scenario.goal[1])
                              << " | max_pitch=" << maximum_pitch
                              << " | max_torque=" << maximum_torque << '\n';
                    return 0;
                }
                if (output)
                    output << state.timestamp << ',' << state.forward_position << ','
                           << state.forward_velocity << ',' << state.pitch << ','
                           << state.pitch_rate << ',' << state.yaw << ',' << state.yaw_rate << ','
                           << pose.x << ',' << pose.y << ',' << command.left_wheel_torque << ','
                           << command.right_wheel_torque << '\n';
            }
            maximum_pitch = std::max(maximum_pitch, std::abs(state.pitch));
            maximum_torque = std::max({maximum_torque, std::abs(command.left_wheel_torque),
                                       std::abs(command.right_wheel_torque)});
            if (!std::isfinite(state.pitch) || maximum_pitch > .7 || maximum_torque > 2.00001 ||
                state.timestamp > trajectory.duration() + 10)
                throw std::runtime_error("D18 balance/timeout/torque failure.");
            for (const auto &obstacle : scenario.obstacles)
                if (backend.debug_has_contact(("known_" + obstacle.name).c_str()))
                    throw std::runtime_error("Obstacle contact.");
            if (viewer && step % 16 == 0)
                viewer->render();
            backend.apply_command(command);
            backend.step();
        }
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
