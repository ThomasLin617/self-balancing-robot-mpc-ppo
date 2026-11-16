#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <learning_policy/trajectory_policy.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navigation/known_map.hpp>
#include <navigation/path_trajectory.hpp>
#include <navigation/reference_preview.hpp>
#include <robot_ros/state_input.hpp>

class TrajectoryPPONode : public rclcpp::Node {
  public:
    TrajectoryPPONode()
        : Node("trajectory_ppo_node"), policy_(robot_ros::setting(*this, "actor", std::string())),
          scenario_(navigation::KnownMap::load(
              robot_ros::setting(*this, "known_map",
                                 ament_index_cpp::get_package_share_directory("robot_ros") +
                                     "/model/known_map.txt"))) {
        goal_ = scenario_.goal;
        path_pub_ = create_publisher<nav_msgs::msg::Path>("reference_path", robot_ros::retained());
        raw_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", robot_ros::retained());
        goal_pub_ =
            create_publisher<geometry_msgs::msg::PoseStamped>("active_goal", robot_ros::retained());
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", robot_ros::reliable(), [this](const geometry_msgs::msg::PoseStamped &m) {
                if (m.header.frame_id != "odom" || !std::isfinite(m.pose.position.x) ||
                    !std::isfinite(m.pose.position.y)) {
                    RCLCPP_ERROR(get_logger(), "Expected finite goal in odom");
                    return;
                }
                goal_ = {m.pose.position.x, m.pose.position.y};
                trajectory_.reset();
                held_ = 0;
                complete_ = false;
            });
        input_ = std::make_unique<robot_ros::StateInput>(
            *this, [this](const auto &state, const auto &pose) { return compute(state, pose); },
            [this] {
                trajectory_.reset();
                goal_ = scenario_.goal;
                held_ = 0;
                complete_ = false;
            });
    }

  private:
    robot_interface::RobotCommand compute(const robot_interface::RobotState &state,
                                          const navigation::PlanarPose &pose) {
        if (!trajectory_) {
            const navigation::MotionLimits limits;
            const auto safe = scenario_.grid.inflated(limits.robot_radius + limits.safety_margin);
            const auto route = scenario_.grid.inflated(limits.robot_radius + limits.safety_margin +
                                                       limits.smoothing_clearance);
            const auto raw = navigation::AStarPlanner().plan(route, {pose.x, pose.y}, goal_);
            const auto waypoints = navigation::simplify_path(route, raw);
            const auto timed = navigation::time_parameterize(
                navigation::smooth_path(safe, waypoints, pose.yaw, limits), limits);
            navigation::validate_trajectory(safe, timed);
            trajectory_ = std::make_unique<navigation::ReferenceTrajectory>(timed);
            origin_ = state.forward_position;
            start_ = state.timestamp;
            nav_msgs::msg::Path path;
            path.header.frame_id = "odom";
            path.header.stamp = now();
            for (const auto &point : trajectory_->points()) {
                geometry_msgs::msg::PoseStamped p;
                p.header = path.header;
                p.pose.position.x = point.x_ref;
                p.pose.position.y = point.y_ref;
                p.pose.orientation.z = std::sin(point.yaw_ref / 2);
                p.pose.orientation.w = std::cos(point.yaw_ref / 2);
                path.poses.push_back(p);
            }
            path_pub_->publish(path);
            nav_msgs::msg::Path raw_path;
            raw_path.header = path.header;
            for (const auto &point : raw) {
                geometry_msgs::msg::PoseStamped p;
                p.header = path.header;
                p.pose.position.x = point[0];
                p.pose.position.y = point[1];
                p.pose.orientation.w = 1;
                raw_path.poses.push_back(p);
            }
            raw_pub_->publish(raw_path);
            geometry_msgs::msg::PoseStamped goal;
            goal.header = path.header;
            goal.pose.position.x = goal_[0];
            goal.pose.position.y = goal_[1];
            goal.pose.orientation.w = 1;
            goal_pub_->publish(goal);
        }
        const double elapsed = state.timestamp - start_;
        navigation::sample_reference_horizon(*trajectory_, elapsed, origin_, .002, refs_);
        const auto torque = policy_.torque(state, refs_);
        const double distance = std::hypot(pose.x - goal_[0], pose.y - goal_[1]);
        const bool settled = elapsed >= trajectory_->duration() && distance < .1 &&
                             std::abs(state.forward_velocity) < .05 &&
                             std::abs(state.yaw_rate) < .1 && std::abs(state.pitch) < .1;
        held_ = settled ? held_ + 1 : 0;
        if (!complete_ && (held_ * .002 >= .5 || elapsed > trajectory_->duration() + 10)) {
            RCLCPP_INFO(get_logger(), "D18 PPO success=%d elapsed=%.6f distance=%.6f",
                        held_ * .002 >= .5, elapsed, distance);
            complete_ = true;
        }
        robot_interface::RobotCommand command;
        command.timestamp = state.timestamp;
        command.left_wheel_torque = torque[0];
        command.right_wheel_torque = torque[1];
        return command;
    }
    learning_policy::TrajectoryPolicy policy_;
    navigation::KnownMap scenario_;
    std::array<double, 2> goal_;
    std::unique_ptr<navigation::ReferenceTrajectory> trajectory_;
    robot_interface::ReferenceHorizon refs_{250};
    double origin_ = 0, start_ = 0;
    int held_ = 0;
    bool complete_ = false;
    std::unique_ptr<robot_ros::StateInput> input_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<TrajectoryPPONode>());
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
}
