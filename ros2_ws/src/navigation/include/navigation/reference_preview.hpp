#pragma once

#include <cmath>
#include <stdexcept>
#include <navigation/reference_trajectory.hpp>
#include <robot_interface/reference_horizon.hpp>

namespace navigation
{

inline void sample_reference_horizon(
    const ReferenceTrajectory& trajectory,
    double elapsed,
    double position_origin,
    double sample_time,
    robot_interface::ReferenceHorizon& references
) {
    if (references.empty() || !std::isfinite(sample_time) || sample_time <= 0.0
        || !std::isfinite(position_origin)) {
        throw std::invalid_argument("Invalid reference preview configuration.");
    }
    for (std::size_t i = 0; i < references.size(); ++i) {
        const double future_time = elapsed + (i + 1) * sample_time;
        const auto point = trajectory.sample(future_time);
        auto& reference = references[i];
        reference.forward_position = position_origin + point.forward_position_ref;
        reference.forward_velocity = point.forward_velocity_ref;
        reference.pitch = 0.0;
        reference.yaw = point.yaw_ref;
        reference.yaw_rate = point.yaw_rate_ref;
    }
}

}
