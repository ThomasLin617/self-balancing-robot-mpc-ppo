#pragma once

#include <functional>
#include <map>
#include <robot_interface/robot_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <navigation/planar_odometry.hpp>
#include <robot_ros/conversions.hpp>

namespace robot_ros
{
class StateInput {
public:
    using Update = std::function<robot_interface::RobotCommand(const robot_interface::RobotState&,
                                                               const navigation::PlanarPose&)>;
    StateInput(rclcpp::Node& node, Update update, std::function<void()> reset)
        : node_(node), update_(update), reset_(reset) {
        timeout_ = setting(node, "state_timeout", .01);
        wall_timeout_ = setting(node, "state_wall_timeout", .5);
        if (timeout_ <= 0 || wall_timeout_ <= 0) {
            throw std::runtime_error("Invalid state timeout.");
        }
        commands_ = node.create_publisher<robot_msgs::msg::WheelTorqueCommand>("wheel_torque", reliable());
        ready_ = node.create_publisher<std_msgs::msg::UInt64>("controller_ready", retained());
        epoch_sub_ = node.create_subscription<std_msgs::msg::UInt64>(
            "episode_start_ns", retained(), [this](const std_msgs::msg::UInt64& m) {
                epoch_ = m.data;
                states_.clear();
                poses_.clear();
                last_ = -1;
                reset_();
                active_ = true;
                received_ = std::chrono::steady_clock::now();
                ready_->publish(m);
            });
        clock_sub_ = node.create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", reliable(), [this](const rosgraph_msgs::msg::Clock& m) {
                clock_ = nanoseconds(m.clock);
                process();
            });
        state_sub_ = node.create_subscription<robot_msgs::msg::RobotState>(
            "robot_state", reliable(), [this](const robot_msgs::msg::RobotState& m) {
                if (!active_ || m.epoch != epoch_) {
                    return;
                }
                if (!finite(m)) {
                    zero();
                    return;
                }
                const auto ns = nanoseconds(m.header.stamp);
                if (ns != static_cast<int64_t>(epoch_) + std::llround(m.episode_time * 1e9) || ns <= last_) {
                    return;
                }
                received_ = std::chrono::steady_clock::now();
                states_[ns] = m;
                process();
            });
        odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
            "odom", reliable(), [this](const nav_msgs::msg::Odometry& m) {
                const auto ns = nanoseconds(m.header.stamp);
                if (!active_ || ns < static_cast<int64_t>(epoch_) || ns <= last_) {
                    return;
                }
                const auto& p = m.pose.pose.position;
                const auto& q = m.pose.pose.orientation;
                if (m.header.frame_id != "odom" || !std::isfinite(p.x) || !std::isfinite(p.y) ||
                    !std::isfinite(q.z) || !std::isfinite(q.w)) {
                    zero();
                    return;
                }
                poses_[ns] = m;
                process();
            });
        watchdog_ = node.create_wall_timer(std::chrono::milliseconds(10), [this] {
            if (active_ &&
                (std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() >
                     wall_timeout_ ||
                 (last_ >= 0 && !fresh(last_, clock_, timeout_)))) {
                zero();
            }
        });
    }

private:
    void zero() {
        if (!active_) {
            return;
        }
        robot_msgs::msg::WheelTorqueCommand message;
        message.header.stamp = stamp(clock_);
        message.state_stamp = stamp(clock_);
        message.epoch = epoch_;
        message.valid = false;
        commands_->publish(message);
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                             "Controller missing/stale/invalid state: zero command");
    }
    void process() {
        while (!states_.empty()) {
            const auto ns = states_.begin()->first;
            if (ns > clock_) {
                break;
            }
            auto pose = poses_.find(ns);
            if (pose == poses_.end()) {
                break;
            }
            const auto message = states_.begin()->second;
            if (!fresh(ns, clock_, timeout_)) {
                zero();
            } else if (std::llround(message.episode_time * 1000) % 2 == 0) {
                robot_msgs::msg::WheelTorqueCommand output;
                output.header.stamp = stamp(clock_);
                output.state_stamp = message.header.stamp;
                output.epoch = epoch_;
                try {
                    const auto state = from_message(message);
                    const auto& p = pose->second.pose.pose;
                    navigation::PlanarPose planar;
                    planar.x = p.position.x;
                    planar.y = p.position.y;
                    planar.yaw = state.yaw;
                    const auto torque = update_(state, planar);
                    if (!std::isfinite(torque.left_wheel_torque) ||
                        !std::isfinite(torque.right_wheel_torque)) {
                        throw std::runtime_error("Non-finite controller output.");
                    }
                    output.left = torque.left_wheel_torque;
                    output.right = torque.right_wheel_torque;
                    output.valid = true;
                } catch (const std::exception& e) {
                    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000, "Controller: %s",
                                          e.what());
                }
                commands_->publish(output);
            }
            last_ = ns;
            states_.erase(states_.begin());
            poses_.erase(pose);
            while (!poses_.empty() && poses_.begin()->first <= last_) {
                poses_.erase(poses_.begin());
            }
        }
        if (states_.size() > 32 || poses_.size() > 32) {
            states_.clear();
            poses_.clear();
            zero();
        }
    }
    rclcpp::Node& node_;
    Update update_;
    std::function<void()> reset_;
    double timeout_, wall_timeout_;
    bool active_ = false;
    uint64_t epoch_ = 0;
    int64_t clock_ = 0, last_ = -1;
    std::chrono::steady_clock::time_point received_;
    std::map<int64_t, robot_msgs::msg::RobotState> states_;
    std::map<int64_t, nav_msgs::msg::Odometry> poses_;
    rclcpp::Publisher<robot_msgs::msg::WheelTorqueCommand>::SharedPtr commands_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr ready_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr epoch_sub_;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
};
} // namespace robot_ros
