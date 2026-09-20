#include <geometry_msgs/msg/pose_stamped.hpp>
#include <learning_policy/ppo_policy.hpp>
#include <robot_ros/state_input.hpp>

class PPONode : public rclcpp::Node {
public:
    PPONode()
        : Node("ppo_node"), policy_(robot_ros::setting(*this, "actor", std::string())),
          builder_(policy_.observation_config()) {
        sequence_ = robot_ros::setting(*this, "goal_sequence", true);
        goal_ = {robot_ros::setting(*this, "goal_x", .4), robot_ros::setting(*this, "goal_y", 0.0)};
        initial_goal_ = goal_;
        initial_sequence_ = sequence_;
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("active_goal", robot_ros::retained());
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", robot_ros::reliable(), [this](const geometry_msgs::msg::PoseStamped& m) {
                if (m.header.frame_id != "odom" || !std::isfinite(m.pose.position.x) ||
                    !std::isfinite(m.pose.position.y)) {
                    RCLCPP_ERROR(get_logger(), "Goal must be finite in odom");
                    return;
                }
                goal_ = {m.pose.position.x, m.pose.position.y};
                sequence_ = false;
                assigned_ = false;
                settled_ = 0;
                complete_ = false;
            });
        input_ = std::make_unique<robot_ros::StateInput>(
            *this, [this](const auto& s, const auto& p) { return compute(s, p); },
            [this] {
                goal_ = initial_goal_;
                sequence_ = initial_sequence_;
                assigned_ = false;
                index_ = 0;
                settled_ = 0;
                complete_ = false;
            });
    }

private:
    robot_interface::RobotCommand compute(const robot_interface::RobotState& state,
                                          const navigation::PlanarPose& pose) {
        if (!assigned_) {
            if (sequence_) {
                const auto d = offsets_[index_];
                goal_ = {pose.x + std::cos(pose.yaw) * d[0] - std::sin(pose.yaw) * d[1],
                         pose.y + std::sin(pose.yaw) * d[0] + std::cos(pose.yaw) * d[1]};
            }
            assigned_ = true;
            start_ = state.timestamp;
            settled_ = 0;
            geometry_msgs::msg::PoseStamped goal;
            goal.header.frame_id = "odom";
            goal.header.stamp = now();
            goal.pose.position.x = goal_[0];
            goal.pose.position.y = goal_[1];
            goal.pose.orientation.w = 1;
            goal_pub_->publish(goal);
        }
        const double distance = std::hypot(goal_[0] - pose.x, goal_[1] - pose.y);
        const bool settled = distance < .1 && std::abs(state.forward_velocity) < .05 &&
                             std::abs(state.yaw_rate) < .1 && std::abs(state.pitch) < .1;
        settled_ = settled ? settled_ + 1 : 0;
        if (!complete_ && (settled_ * .002 >= .5 || state.timestamp - start_ >= 15)) {
            RCLCPP_INFO(get_logger(), "PPO goal %zu success=%d time=%.6f distance=%.6f pitch=%.6f", index_,
                        settled_ * .002 >= .5, state.timestamp, distance, state.pitch);
            if (sequence_ && index_ + 1 < offsets_.size()) {
                ++index_;
                assigned_ = false;
            } else {
                complete_ = true;
            }
        }
        const auto torque = policy_.torque(state, pose, goal_, builder_);
        robot_interface::RobotCommand command;
        command.left_wheel_torque = torque[0];
        command.right_wheel_torque = torque[1];
        return command;
    }
    learning_policy::PPOPolicy policy_;
    learning_policy::GoalObservationBuilder builder_;
    std::unique_ptr<robot_ros::StateInput> input_;
    bool initial_sequence_ = true, sequence_ = true, assigned_ = false, complete_ = false;
    std::size_t index_ = 0;
    int settled_ = 0;
    double start_ = 0;
    std::array<double, 2> goal_, initial_goal_;
    const std::array<std::array<double, 2>, 4> offsets_{{{.4, 0}, {.4, .3}, {.4, -.3}, {-.3, 0}}};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PPONode>());
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
}
