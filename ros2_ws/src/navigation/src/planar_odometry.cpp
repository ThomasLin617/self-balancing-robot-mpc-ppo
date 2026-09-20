#include <navigation/planar_odometry.hpp>
#include <robot_interface/angle.hpp>
#include <cmath>
#include <stdexcept>

namespace navigation
{

PlanarPose PlanarOdometry::update(const robot_interface::RobotState& state) {
    if (!std::isfinite(state.timestamp) || !std::isfinite(state.yaw)
        || !std::isfinite(state.forward_velocity)) {
        throw std::invalid_argument("Invalid odometry input.");
    }
    if (initialized_) {
        const double dt = state.timestamp - previous_timestamp_;
        if (dt <= 0.0) {
            throw std::runtime_error("Odometry dt <= 0.");
        }
        const double yaw_change = robot_interface::wrap_to_pi(state.yaw - pose_.yaw);
        const double midpoint_yaw = pose_.yaw + 0.5 * yaw_change;
        const double distance = 0.5 * (previous_velocity_ + state.forward_velocity) * dt;
        pose_.x += distance * std::cos(midpoint_yaw);
        pose_.y += distance * std::sin(midpoint_yaw);
    }
    pose_.yaw = robot_interface::wrap_to_pi(state.yaw);
    previous_timestamp_ = state.timestamp;
    previous_velocity_ = state.forward_velocity;
    initialized_ = true;
    return pose_;
}

void PlanarOdometry::reset() {
    pose_ = PlanarPose{};
    previous_timestamp_ = 0.0;
    previous_velocity_ = 0.0;
    initialized_ = false;
}

}
