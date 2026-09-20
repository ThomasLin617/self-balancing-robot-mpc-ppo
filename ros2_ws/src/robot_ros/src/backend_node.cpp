#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <unistd.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mujoco_backend/mujoco_backend.hpp>
#include <mujoco_backend/mujoco_viewer.hpp>
#include <mujoco_backend/known_map_world.hpp>
#include <navigation/known_map.hpp>
#include <robot_ros/conversions.hpp>

using Steady = std::chrono::steady_clock;
class BackendNode : public rclcpp::Node {
public:
    BackendNode() : Node("mujoco_backend_node") {
        const auto share = ament_index_cpp::get_package_share_directory("robot_ros");
        std::string model = robot_ros::setting(*this, "model", share + "/model/Robot1.xml");
        map_file_ = robot_ros::setting(*this, "known_map", std::string());
        duration_ = robot_ros::setting(*this, "duration", 12.0);
        timeout_ = robot_ros::setting(*this, "command_timeout", .01);
        wait_timeout_ = robot_ros::setting(*this, "wall_wait_timeout", .5);
        stride_ = robot_ros::setting(*this, "encoder_stride", 1);
        force_ = robot_ros::setting(*this, "force", 0.0);
        if (duration_ <= 0 || timeout_ <= 0 || wait_timeout_ <= 0 || stride_ < 1) {
            throw std::runtime_error("Invalid backend parameters.");
        }
        if (robot_ros::setting(*this, "upright_reset", false)) {
            std::ifstream input(model);
            std::ostringstream buffer;
            buffer << input.rdbuf();
            auto xml = buffer.str();
            const std::string original = "euler=\"0 -20 0\"";
            const auto index = xml.find(original);
            if (index == std::string::npos) {
                throw std::runtime_error("Unexpected model initial pose.");
            }
            xml.replace(index, original.size(), "euler=\"0 0 0\"");
            temporary_ = (std::filesystem::temp_directory_path() /
                          ("ros-upright-" + std::to_string(getpid()) + ".xml"))
                             .string();
            std::ofstream output(temporary_);
            output << xml;
            output.close();
            model = temporary_;
        }
        if (!map_file_.empty()) {
            scenario_ = std::make_unique<navigation::KnownMap>(navigation::KnownMap::load(map_file_));
            world_ = std::make_unique<mujoco_backend::KnownMapWorld>(model, *scenario_);
            model = world_->path();
        }
        backend_ = std::make_unique<mujoco_backend::MujocoBackend>(model);
        if (!temporary_.empty()) {
            std::filesystem::remove(temporary_);
            temporary_.clear();
        }
        if (robot_ros::setting(*this, "viewer", false)) {
            viewer_ = std::make_unique<mujoco_backend::MujocoViewer>(*backend_);
            viewer_->track_body("base_body");
            if (scenario_) {
                const auto lower = scenario_->grid.grid_to_world({0, 0});
                const auto upper = scenario_->grid.grid_to_world(
                    {scenario_->grid.height() - 1, scenario_->grid.width() - 1});
                viewer_->set_camera_overview({.5 * (lower[0] + upper[0]), .5 * (lower[1] + upper[1]), 0},
                                             1.25 * scenario_->grid.resolution() *
                                                 std::max(scenario_->grid.width(), scenario_->grid.height()));
            }
            viewer_->set_camera_azimuth(135);
            viewer_->enable_studio_lighting();
        }
        const auto log = robot_ros::setting(*this, "trace", std::string());
        if (!log.empty()) {
            trace_.open(log);
            if (!trace_) {
                throw std::runtime_error("Cannot open backend trace.");
            }
            trace_ << std::setprecision(17)
                   << "epoch,time,p,v,pitch,pitch_rate,yaw,yaw_rate,left,right,applied_l,applied_r,true_x,"
                      "true_y,true_yaw,state_age,command_age,state_wall_us,command_wall_us,total_wall_us,"
                      "fault,contact\n";
        }
        imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", robot_ros::reliable());
        joints_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", robot_ros::reliable());
        clock_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", robot_ros::reliable());
        epoch_pub_ = create_publisher<std_msgs::msg::UInt64>("episode_start_ns", robot_ros::retained());
        command_sub_ = create_subscription<robot_msgs::msg::WheelTorqueCommand>(
            "wheel_torque", robot_ros::reliable(), [this](const robot_msgs::msg::WheelTorqueCommand& m) {
                if (m.epoch != epoch_ || robot_ros::nanoseconds(m.state_stamp) != current_ns()) {
                    return;
                }
                if (!std::isfinite(m.left) || !std::isfinite(m.right)) {
                    RCLCPP_ERROR(get_logger(), "Non-finite torque rejected");
                    command_ = robot_msgs::msg::WheelTorqueCommand();
                    return;
                }
                command_ = m;
                command_ready_ = true;
                command_wall_us_ = elapsed();
            });
        state_sub_ = create_subscription<robot_msgs::msg::RobotState>(
            "robot_state", robot_ros::reliable(), [this](const robot_msgs::msg::RobotState& m) {
                if (m.epoch == epoch_ && robot_ros::nanoseconds(m.header.stamp) == current_ns() &&
                    robot_ros::finite(m)) {
                    state_ = m;
                    state_ready_ = true;
                    state_wall_us_ = elapsed();
                }
            });
        estimator_ready_ = create_subscription<std_msgs::msg::UInt64>(
            "estimator_ready", robot_ros::retained(),
            [this](const std_msgs::msg::UInt64& m) { est_ready_ = m.data == epoch_; });
        controller_ready_ = create_subscription<std_msgs::msg::UInt64>(
            "controller_ready", robot_ros::retained(),
            [this](const std_msgs::msg::UInt64& m) { ctrl_ready_ = m.data == epoch_; });
        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                epoch_ = current_ns() + 1000000;
                reset();
                response->success = true;
                response->message = "Reset accepted; waiting for estimator/controller epoch acknowledgments.";
            });
        pause_service_ = create_service<std_srvs::srv::SetBool>(
            "simulation/pause", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                       std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                paused_ = request->data;
                command_ = robot_msgs::msg::WheelTorqueCommand();
                backend_->apply_command({});
                issued_ = Steady::now();
                response->success = true;
            });
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "reference_path", robot_ros::retained(), [this](const nav_msgs::msg::Path& m) {
                path_.clear();
                for (const auto& p : m.poses) {
                    path_.push_back({p.pose.position.x, p.pose.position.y, .015});
                }
                draw();
            });
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "active_goal", robot_ros::retained(), [this](const geometry_msgs::msg::PoseStamped& m) {
                goal_ = {m.pose.position.x, m.pose.position.y, .055};
                has_goal_ = true;
                draw();
            });
        raw_sub_ = create_subscription<nav_msgs::msg::Path>(
            "planned_path", robot_ros::retained(), [this](const nav_msgs::msg::Path& m) {
                if (!viewer_) {
                    return;
                }
                std::vector<std::array<double, 3>> points;
                for (const auto& p : m.poses) {
                    points.push_back({p.pose.position.x, p.pose.position.y, .03});
                }
                viewer_->set_planned_route(points, {});
            });
        reset();
        timer_ = create_wall_timer(std::chrono::microseconds(200), [this] { tick(); });
    }

private:
    int64_t current_ns() const {
        return epoch_ + std::llround(backend_->simulation_time() * 1e9);
    }
    double elapsed() const {
        return std::chrono::duration<double, std::micro>(Steady::now() - issued_).count();
    }
    void draw() {
        if (!viewer_) {
            return;
        }
        std::vector<std::array<double, 3>> points;
        const std::size_t stride = (path_.size() + 498) / 499;
        for (std::size_t i = 0; i < path_.size(); i += stride) {
            points.push_back(path_[i]);
        }
        if (!path_.empty()) {
            points.push_back(path_.back());
        }
        if (points.empty() && has_goal_) {
            points.push_back(goal_);
        }
        viewer_->set_reference_path(points, goal_);
    }
    void reset() {
        backend_->reset();
        command_ = robot_msgs::msg::WheelTorqueCommand();
        state_ = robot_msgs::msg::RobotState();
        step_ = 0;
        waiting_ = false;
        paused_ = false;
        est_ready_ = false;
        ctrl_ready_ = false;
        std_msgs::msg::UInt64 epoch;
        epoch.data = epoch_;
        epoch_pub_->publish(epoch);
        rosgraph_msgs::msg::Clock clock;
        clock.clock = robot_ros::stamp(current_ns());
        clock_->publish(clock);
        path_.clear();
        has_goal_ = false;
        if (viewer_) {
            viewer_->set_reference_path({}, {});
            viewer_->set_planned_route({}, {});
        }
        issued_ = Steady::now();
    }
    void publish_sensors() {
        const auto sensor = backend_->read_sensor_data();
        const auto time = robot_ros::stamp(current_ns());
        rosgraph_msgs::msg::Clock clock;
        clock.clock = time;
        clock_->publish(clock);
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = time;
        imu.header.frame_id = "imu_link";
        imu.orientation_covariance[0] = -1;
        imu.angular_velocity.x = sensor.gyroscope[0];
        imu.angular_velocity.y = sensor.gyroscope[1];
        imu.angular_velocity.z = sensor.gyroscope[2];
        imu.linear_acceleration.x = sensor.accelerometer[0];
        imu.linear_acceleration.y = sensor.accelerometer[1];
        imu.linear_acceleration.z = sensor.accelerometer[2];
        if (step_ % stride_ == 0) {
            sensor_msgs::msg::JointState joint;
            joint.header = imu.header;
            joint.header.frame_id = "base_link";
            joint.name = {"left_wheel_joint", "right_wheel_joint"};
            joint.position = {sensor.left_wheel_position, sensor.right_wheel_position};
            joint.velocity = {sensor.left_wheel_velocity, sensor.right_wheel_velocity};
            joints_->publish(joint);
        }
        imu_->publish(imu);
        waiting_ = true;
        state_ready_ = false;
        command_ready_ = false;
        issued_ = Steady::now();
        state_wall_us_ = -1;
        command_wall_us_ = -1;
    }
    void tick() {
        if (paused_ || !est_ready_ || !ctrl_ready_) {
            return;
        }
        if (viewer_ && viewer_->should_close()) {
            rclcpp::shutdown();
            return;
        }
        if (backend_->simulation_time() + 1e-9 >= duration_) {
            if (trace_) {
                trace_.flush();
            }
            return;
        }
        if (!waiting_) {
            publish_sensors();
            return;
        }
        const bool timeout = elapsed() > wait_timeout_ * 1e6;
        if (!(state_ready_ && (step_ % 2 || command_ready_)) && !timeout) {
            return;
        }
        bool valid = command_.valid && command_.epoch == epoch_ &&
                     robot_ros::fresh(robot_ros::nanoseconds(command_.header.stamp), current_ns(), timeout_);
        if (timeout || !state_ready_ || command_sub_->get_publisher_count() != 1) {
            valid = false;
        }
        robot_interface::RobotCommand torque;
        if (valid) {
            torque.left_wheel_torque = command_.left;
            torque.right_wheel_torque = command_.right;
        }
        if (!valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Missing/stale data or invalid command: applying zero torque");
        }
        backend_->apply_command(torque);
        const auto truth = backend_->debug_planar_pose("base_body");
        bool contact = false;
        if (scenario_) {
            for (const auto& o : scenario_->obstacles) {
                contact = contact || backend_->debug_has_contact(("known_" + o.name).c_str());
            }
        }
        if (trace_) {
            trace_ << epoch_ << ',' << backend_->simulation_time() << ',' << state_.forward_position << ','
                   << state_.forward_velocity << ',' << state_.pitch << ',' << state_.pitch_rate << ','
                   << state_.yaw << ',' << state_.yaw_rate << ',' << command_.left << ',' << command_.right
                   << ',' << std::clamp(torque.left_wheel_torque, -2.0, 2.0) << ','
                   << std::clamp(torque.right_wheel_torque, -2.0, 2.0) << ',' << truth[0] << ',' << truth[1]
                   << ',' << truth[2] << ','
                   << (current_ns() - robot_ros::nanoseconds(state_.header.stamp)) * 1e-9 << ','
                   << (current_ns() - robot_ros::nanoseconds(command_.header.stamp)) * 1e-9 << ','
                   << state_wall_us_ << ',' << command_wall_us_ << ',' << elapsed() << ',' << !valid << ','
                   << contact << '\n';
        }
        backend_->set_body_forward_force("base_body", backend_->simulation_time() + 1e-9 >= 5.0 &&
                                                              backend_->simulation_time() + 1e-9 < 5.05
                                                          ? force_
                                                          : 0.0);
        backend_->step();
        ++step_;
        waiting_ = false;
        if (viewer_ && step_ % 16 == 0) {
            viewer_->render();
        }
    }
    uint64_t epoch_ = 0;
    int step_ = 0, stride_ = 1;
    double duration_, timeout_, wait_timeout_, force_;
    bool has_goal_ = false, waiting_ = false, paused_ = false, state_ready_ = false, command_ready_ = false,
         est_ready_ = false, ctrl_ready_ = false;
    double state_wall_us_ = 0, command_wall_us_ = 0;
    Steady::time_point issued_;
    std::string map_file_, temporary_;
    std::ofstream trace_;
    std::unique_ptr<navigation::KnownMap> scenario_;
    std::unique_ptr<mujoco_backend::KnownMapWorld> world_;
    std::unique_ptr<mujoco_backend::MujocoBackend> backend_;
    std::unique_ptr<mujoco_backend::MujocoViewer> viewer_;
    std::vector<std::array<double, 3>> path_;
    std::array<double, 3> goal_{};
    robot_msgs::msg::WheelTorqueCommand command_;
    robot_msgs::msg::RobotState state_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr epoch_pub_;
    rclcpp::Subscription<robot_msgs::msg::WheelTorqueCommand>::SharedPtr command_sub_;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr estimator_ready_, controller_ready_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_, raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<BackendNode>());
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
