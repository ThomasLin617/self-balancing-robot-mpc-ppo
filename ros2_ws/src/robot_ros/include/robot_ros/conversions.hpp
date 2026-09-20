#pragma once

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/robot_state.hpp>
#include <robot_msgs/msg/wheel_torque_command.hpp>
#include <robot_interface/robot_state.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace robot_ros
{
template <typename T> T setting(rclcpp::Node& node, const std::string& name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(name, value, descriptor);
}

inline int64_t nanoseconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
inline builtin_interfaces::msg::Time stamp(int64_t ns) {
    return rclcpp::Time(ns);
}
inline bool fresh(int64_t measurement, int64_t now, double timeout) {
    return measurement <= now && now - measurement <= static_cast<int64_t>(timeout * 1e9);
}
inline robot_interface::RobotState from_message(const robot_msgs::msg::RobotState& message) {
    robot_interface::RobotState state;
    state.timestamp = message.episode_time;
    state.forward_position = message.forward_position;
    state.forward_velocity = message.forward_velocity;
    state.pitch = message.pitch;
    state.pitch_rate = message.pitch_rate;
    state.yaw = message.yaw;
    state.yaw_rate = message.yaw_rate;
    state.left_wheel_position = message.left_wheel_position;
    state.right_wheel_position = message.right_wheel_position;
    state.left_wheel_velocity = message.left_wheel_velocity;
    state.right_wheel_velocity = message.right_wheel_velocity;
    return state;
}
inline robot_msgs::msg::RobotState to_message(const robot_interface::RobotState& state, uint64_t epoch) {
    robot_msgs::msg::RobotState message;
    message.header.stamp = stamp(epoch + std::llround(state.timestamp * 1e9));
    message.header.frame_id = "odom";
    message.epoch = epoch;
    message.episode_time = state.timestamp;
    message.forward_position = state.forward_position;
    message.forward_velocity = state.forward_velocity;
    message.pitch = state.pitch;
    message.pitch_rate = state.pitch_rate;
    message.yaw = state.yaw;
    message.yaw_rate = state.yaw_rate;
    message.left_wheel_position = state.left_wheel_position;
    message.right_wheel_position = state.right_wheel_position;
    message.left_wheel_velocity = state.left_wheel_velocity;
    message.right_wheel_velocity = state.right_wheel_velocity;
    return message;
}
inline bool finite(const robot_msgs::msg::RobotState& m) {
    return std::isfinite(m.episode_time) && std::isfinite(m.forward_position) &&
           std::isfinite(m.forward_velocity) && std::isfinite(m.pitch) && std::isfinite(m.pitch_rate) &&
           std::isfinite(m.yaw) && std::isfinite(m.yaw_rate) && std::isfinite(m.left_wheel_position) &&
           std::isfinite(m.right_wheel_position) && std::isfinite(m.left_wheel_velocity) &&
           std::isfinite(m.right_wheel_velocity);
}
inline rclcpp::QoS reliable() {
    return rclcpp::QoS(8).reliable();
}
inline rclcpp::QoS retained() {
    return rclcpp::QoS(1).reliable().transient_local();
}
} // namespace robot_ros
