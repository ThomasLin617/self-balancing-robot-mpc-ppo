#include <algorithm>
#include <cmath>
#include <fstream>
#include <learning_policy/trajectory_policy.hpp>
#include <robot_interface/angle.hpp>
#include <stdexcept>

namespace learning_policy {
TrajectoryPolicy::TrajectoryPolicy(const std::string &path) {
    std::ifstream file(path);
    std::string magic;
    std::size_t input = 0, hidden1 = 0, hidden2 = 0;
    file >> magic >> input >> hidden1 >> hidden2;
    if (magic != "d18_mlp_v1" || input != 1256 || hidden1 != 128 || hidden2 != 128)
        throw std::runtime_error("Unsupported trajectory actor contract.");
    for (auto &scale : scales_) {
        file >> scale;
        if (!std::isfinite(scale) || scale <= 0)
            throw std::runtime_error("Invalid trajectory scale.");
    }
    file >> clip_ >> torque_limit_;
    if (!std::isfinite(clip_) || clip_ <= 0 || !std::isfinite(torque_limit_) ||
        torque_limit_ <= 0 || torque_limit_ > 2)
        throw std::runtime_error("Invalid trajectory actor limits.");
    for (auto shape :
         std::array<std::array<std::size_t, 2>, 3>{{{128, 1256}, {128, 128}, {2, 128}}}) {
        Layer layer;
        layer.rows = shape[0];
        layer.columns = shape[1];
        layer.weights.resize(layer.rows * layer.columns);
        layer.bias.resize(layer.rows);
        for (double &value : layer.weights) {
            file >> value;
            if (!std::isfinite(value))
                throw std::runtime_error("Invalid actor weights.");
        }
        for (double &value : layer.bias) {
            file >> value;
            if (!std::isfinite(value))
                throw std::runtime_error("Invalid actor bias.");
        }
        if (!file)
            throw std::runtime_error("Truncated trajectory actor.");
        layers_.push_back(std::move(layer));
    }
    std::string extra;
    if (file >> extra)
        throw std::runtime_error("Unexpected trajectory actor data.");
}

std::vector<double>
TrajectoryPolicy::observation(const robot_interface::RobotState &state,
                              const robot_interface::ReferenceHorizon &refs) const {
    if (refs.size() != 250)
        throw std::invalid_argument("Expected 250 trajectory references.");
    const std::array<double, 6> current{0, state.forward_velocity, state.pitch, state.pitch_rate,
                                        0, state.yaw_rate};
    if (!std::isfinite(state.forward_position) || !std::isfinite(state.yaw))
        throw std::invalid_argument("Invalid state origin.");
    std::vector<double> result;
    result.reserve(1256);
    auto append = [&](double value, double scale) {
        if (!std::isfinite(value))
            throw std::invalid_argument("Non-finite trajectory observation.");
        result.push_back(std::clamp(value / scale, -clip_, clip_));
    };
    for (std::size_t i = 0; i < 6; ++i)
        append(current[i], scales_[i]);
    double previous = state.yaw;
    for (const auto &ref : refs) {
        const double yaw = previous + robot_interface::wrap_to_pi(ref.yaw - previous);
        append(ref.forward_position - state.forward_position, scales_[0]);
        append(ref.forward_velocity, scales_[1]);
        append(ref.pitch, scales_[2]);
        append(yaw - state.yaw, scales_[4]);
        append(ref.yaw_rate, scales_[5]);
        previous = yaw;
    }
    return result;
}

std::array<double, 2>
TrajectoryPolicy::normalized_action(const std::vector<double> &observation) const {
    if (observation.size() != 1256)
        throw std::invalid_argument("Incorrect trajectory observation dimension.");
    auto values = observation;
    for (double x : values)
        if (!std::isfinite(x))
            throw std::invalid_argument("Non-finite actor input.");
    for (std::size_t i = 0; i < layers_.size(); ++i) {
        const auto &layer = layers_[i];
        std::vector<double> next(layer.rows);
        for (std::size_t row = 0; row < layer.rows; ++row) {
            double value = layer.bias[row];
            for (std::size_t col = 0; col < layer.columns; ++col)
                value += layer.weights[row * layer.columns + col] * values[col];
            next[row] = i + 1 < layers_.size() ? std::tanh(value) : std::clamp(value, -1.0, 1.0);
        }
        values = std::move(next);
    }
    return {values[0], values[1]};
}

std::array<double, 2>
TrajectoryPolicy::torque(const robot_interface::RobotState &state,
                         const robot_interface::ReferenceHorizon &refs) const {
    const auto action = normalized_action(observation(state, refs));
    return {torque_limit_ * action[0], torque_limit_ * action[1]};
}
} // namespace learning_policy
