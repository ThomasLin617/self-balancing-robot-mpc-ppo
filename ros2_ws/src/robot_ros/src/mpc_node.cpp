#include <memory>
#include <robot_interface/angle.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <controller/mpc_controller.hpp>
#include <navigation/known_map.hpp>
#include <navigation/path_trajectory.hpp>
#include <navigation/reference_preview.hpp>
#include <robot_ros/state_input.hpp>

class MPCNode : public rclcpp::Node {
public:
    MPCNode() : Node("mpc_node"), horizon_(250) {
        mode_ = robot_ros::setting(*this, "mode", std::string("regulation"));
        if (mode_ != "regulation" && mode_ != "goal" && mode_ != "known_map" && mode_ != "velocity") {
            throw std::runtime_error("Invalid MPC mode.");
        }
        position_ = robot_ros::setting(*this, "position_reference", .1);
        goal_ = {robot_ros::setting(*this, "goal_x", .4), robot_ros::setting(*this, "goal_y", 0.0)};
        startup_ = robot_ros::setting(*this, "startup_duration",
                                      mode_ == "known_map" ? 4.0
                                      : mode_ == "goal"    ? 3.0
                                                           : 0.0);
        max_velocity_ = robot_ros::setting(*this, "max_velocity", .18);
        acceleration_ = robot_ros::setting(*this, "acceleration", .08);
        if (mode_ == "known_map") {
            map_ = std::make_unique<navigation::KnownMap>(
                navigation::KnownMap::load(robot_ros::setting(*this, "known_map", std::string())));
        }
        if (map_) {
            goal_ = map_->goal;
        }
        initial_goal_ = goal_;
        path_pub_ = create_publisher<nav_msgs::msg::Path>("reference_path", robot_ros::retained());
        raw_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", robot_ros::retained());
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("active_goal", robot_ros::retained());
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", robot_ros::reliable(), [this](const geometry_msgs::msg::PoseStamped& m) {
                if (m.header.frame_id != "odom" || !std::isfinite(m.pose.position.x) ||
                    !std::isfinite(m.pose.position.y)) {
                    RCLCPP_ERROR(get_logger(), "Goal must be finite in odom");
                    return;
                }
                if (mode_ != "goal" && mode_ != "known_map") {
                    return;
                }
                goal_ = {m.pose.position.x, m.pose.position.y};
                trajectory_.reset();
                failed_ = false;
            });
        velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", robot_ros::reliable(), [this](const geometry_msgs::msg::Twist& m) {
                if (mode_ != "velocity") {
                    return;
                }
                if (!std::isfinite(m.linear.x) || !std::isfinite(m.angular.z)) {
                    velocity_ = 0;
                    yaw_rate_ = 0;
                    return;
                }
                velocity_ = m.linear.x;
                yaw_rate_ = m.angular.z;
                velocity_time_ = now().seconds();
            });
        input_ = std::make_unique<robot_ros::StateInput>(
            *this, [this](const auto& s, const auto& p) { return compute(s, p); },
            [this] {
                goal_ = initial_goal_;
                publish_path({}, path_pub_);
                publish_path({}, raw_pub_);
                mpc_ = std::make_unique<controller::MPCController>(250, 2.0);
                trajectory_.reset();
                failed_ = false;
                reference_ = {};
                velocity_ = 0;
                yaw_rate_ = 0;
                previous_time_ = 0;
            });
    }

private:
    void publish_path(const std::vector<navigation::PathPoint>& points,
                      const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "odom";
        path.header.stamp = now();
        for (const auto& p : points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = p[0];
            pose.pose.position.y = p[1];
            pose.pose.orientation.w = 1;
            path.poses.push_back(pose);
        }
        publisher->publish(path);
    }
    robot_interface::RobotCommand compute(const robot_interface::RobotState& state,
                                          const navigation::PlanarPose& pose) {
        if (mode_ == "regulation") {
            robot_interface::Reference reference;
            reference.forward_position = position_;
            return mpc_->compute_command(state, reference);
        }
        if (mode_ == "velocity") {
            const double dt = state.timestamp - previous_time_;
            previous_time_ = state.timestamp;
            if (now().seconds() - velocity_time_ > .5) {
                velocity_ = 0;
                yaw_rate_ = 0;
            }
            reference_.forward_position += velocity_ * dt;
            reference_.yaw = robot_interface::wrap_to_pi(reference_.yaw + yaw_rate_ * dt);
            for (std::size_t i = 0; i < horizon_.size(); ++i) {
                horizon_[i] = reference_;
                horizon_[i].forward_position += velocity_ * (i + 1) * .002;
                horizon_[i].forward_velocity = velocity_;
                horizon_[i].yaw = robot_interface::wrap_to_pi(reference_.yaw + yaw_rate_ * (i + 1) * .002);
                horizon_[i].yaw_rate = yaw_rate_;
            }
            return mpc_->compute_command(state, horizon_);
        }
        if (state.timestamp + 1e-9 < startup_) {
            return mpc_->compute_command(state, robot_interface::Reference{});
        }
        if (failed_) {
            throw std::runtime_error("Planning previously failed; waiting for a new goal/reset.");
        }
        if (!trajectory_) {
            try {
                if (map_) {
                    const navigation::MotionLimits limits;
                    const auto safe = map_->grid.inflated(limits.robot_radius + limits.safety_margin);
                    const auto routing = map_->grid.inflated(limits.robot_radius + limits.safety_margin +
                                                             limits.smoothing_clearance);
                    const auto raw = navigation::AStarPlanner().plan(routing, {pose.x, pose.y}, goal_);
                    const auto waypoints = navigation::simplify_path(routing, raw);
                    const auto geometry = navigation::smooth_path(safe, waypoints, pose.yaw, limits);
                    const auto timed = navigation::time_parameterize(geometry, limits);
                    navigation::validate_trajectory(safe, timed);
                    trajectory_ = std::make_unique<navigation::ReferenceTrajectory>(timed);
                    publish_path(raw, raw_pub_);
                } else {
                    trajectory_ =
                        std::make_unique<navigation::ReferenceTrajectory>(max_velocity_, acceleration_);
                    trajectory_->generate(pose, goal_[0], goal_[1]);
                }
                origin_ = state.forward_position;
                start_ = state.timestamp;
                std::vector<navigation::PathPoint> points;
                for (const auto& p : trajectory_->points()) {
                    points.push_back({p.x_ref, p.y_ref});
                }
                publish_path(points, path_pub_);
                geometry_msgs::msg::PoseStamped target;
                target.header.frame_id = "odom";
                target.header.stamp = now();
                target.pose.position.x = goal_[0];
                target.pose.position.y = goal_[1];
                target.pose.orientation.w = 1;
                goal_pub_->publish(target);
                RCLCPP_INFO(get_logger(), "Plan from estimated pose %.6f %.6f; duration %.6f", pose.x, pose.y,
                            trajectory_->duration());
            } catch (...) {
                failed_ = true;
                throw;
            }
        }
        navigation::sample_reference_horizon(*trajectory_, state.timestamp - start_, origin_,
                                             mpc_->sample_time(), horizon_);
        return mpc_->compute_command(state, horizon_);
    }
    std::string mode_;
    double position_, startup_, max_velocity_, acceleration_,
        origin_ = 0, start_ = 0, velocity_ = 0, yaw_rate_ = 0, velocity_time_ = 0, previous_time_ = 0;
    bool failed_ = false;
    navigation::PathPoint goal_, initial_goal_;
    robot_interface::Reference reference_;
    robot_interface::ReferenceHorizon horizon_;
    std::unique_ptr<navigation::KnownMap> map_;
    std::unique_ptr<navigation::ReferenceTrajectory> trajectory_;
    std::unique_ptr<controller::MPCController> mpc_;
    std::unique_ptr<robot_ros::StateInput> input_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MPCNode>());
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
}
