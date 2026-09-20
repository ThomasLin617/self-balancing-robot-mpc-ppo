#include <learning_policy/ppo_policy.hpp>
#include <robot_interface/angle.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace learning_policy
{
namespace {

std::vector<double> values(std::istringstream& stream) {
    std::vector<double> result;
    double value = 0.0;
    while (stream >> value) result.push_back(value);
    return result;
}

void require_finite(const std::vector<double>& values_to_check) {
    for (double value : values_to_check) if (!std::isfinite(value)) throw std::runtime_error("Non-finite policy file.");
}

}

GoalObservationBuilder::GoalObservationBuilder(GoalObservationConfig config) : config_(config) {
    for (double scale : config_.scales) if (!(scale > 0.0) || !std::isfinite(scale)) throw std::invalid_argument("Invalid observation scale.");
    if (!(config_.clip > 0.0) || !std::isfinite(config_.clip)
        || !(config_.torque_limit > 0.0) || config_.torque_limit > 2.0) {
        throw std::invalid_argument("Invalid policy configuration.");
    }
}

std::array<double, 6> GoalObservationBuilder::build(const robot_interface::RobotState& state,
                                                       const navigation::PlanarPose& pose,
                                                       std::array<double, 2> goal) const {
    const double dx = goal[0] - pose.x;
    const double dy = goal[1] - pose.y;
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    const std::array<double, 6> raw{c * dx + s * dy, -s * dx + c * dy,
                                    state.forward_velocity, state.pitch,
                                    state.pitch_rate, state.yaw_rate};
    std::array<double, 6> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        if (!std::isfinite(raw[i])) throw std::invalid_argument("Non-finite policy observation.");
        result[i] = std::clamp(raw[i] / config_.scales[i], -config_.clip, config_.clip);
    }
    return result;
}

PPOPolicy::PPOPolicy(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open PPO policy.");
    std::string line;
    if (!std::getline(file, line) || line != "d15_mlp_v1") throw std::runtime_error("Unsupported PPO policy format.");
    while (std::getline(file, line)) {
        std::istringstream stream(line);
        std::string tag;
        stream >> tag;
        if (tag == "scales") {
            for (double& scale : config_.scales) stream >> scale;
        } else if (tag == "clip") stream >> config_.clip;
        else if (tag == "torque_limit") stream >> config_.torque_limit;
        else if (tag == "layer") {
            Layer layer;
            std::size_t layer_index = 0;
            stream >> layer_index >> layer.rows >> layer.columns;
            layer.weights.reserve(layer.rows * layer.columns);
            for (std::size_t row = 0; row < layer.rows; ++row) {
                if (!std::getline(file, line)) throw std::runtime_error("Truncated PPO layer.");
                std::istringstream row_stream(line);
                row_stream >> tag;
                if (tag != "row") throw std::runtime_error("Expected PPO layer row.");
                const auto row_values = values(row_stream);
                if (row_values.size() != layer.columns) throw std::runtime_error("Incorrect PPO layer width.");
                layer.weights.insert(layer.weights.end(), row_values.begin(), row_values.end());
            }
            if (!std::getline(file, line)) throw std::runtime_error("Truncated PPO layer bias.");
            std::istringstream bias_stream(line);
            bias_stream >> tag;
            if (tag != "bias") throw std::runtime_error("Expected PPO layer bias.");
            std::size_t bias_index = 0;
            bias_stream >> bias_index;
            layer.bias = values(bias_stream);
            if (layer.bias.size() != layer.rows) throw std::runtime_error("Incorrect PPO bias width.");
            require_finite(layer.weights);
            require_finite(layer.bias);
            hidden_.push_back(std::move(layer));
        } else if (tag == "action_weights") {
            action_.rows = 2;
            action_.columns = hidden_.empty() ? 0 : hidden_.back().rows;
            for (std::size_t row = 0; row < action_.rows; ++row) {
                if (!std::getline(file, line)) throw std::runtime_error("Truncated PPO action layer.");
                std::istringstream row_stream(line);
                row_stream >> tag;
                if (tag != "row") throw std::runtime_error("Expected PPO action row.");
                const auto row_values = values(row_stream);
                if (row_values.size() != action_.columns) throw std::runtime_error("Incorrect PPO action width.");
                action_.weights.insert(action_.weights.end(), row_values.begin(), row_values.end());
            }
        } else if (tag == "action_bias") {
            action_.bias = values(stream);
        }
    }
    if (hidden_.empty() || action_.weights.size() != 2 * action_.columns || action_.bias.size() != 2) throw std::runtime_error("Incomplete PPO policy.");
    require_finite(action_.weights);
    require_finite(action_.bias);
    const GoalObservationBuilder builder(config_);
    (void)builder;
}

const GoalObservationConfig& PPOPolicy::observation_config() const {
    return config_;
}

std::array<double, 2> PPOPolicy::normalized_action(const std::array<double, 6>& observation) const {
    std::vector<double> values_to_process(observation.begin(), observation.end());
    for (double value : values_to_process) if (!std::isfinite(value)) throw std::invalid_argument("Non-finite policy input.");
    for (const auto& layer : hidden_) {
        if (values_to_process.size() != layer.columns) throw std::runtime_error("PPO layer dimensions do not match.");
        std::vector<double> next(layer.rows, 0.0);
        for (std::size_t row = 0; row < layer.rows; ++row) {
            for (std::size_t column = 0; column < layer.columns; ++column) next[row] += layer.weights[row * layer.columns + column] * values_to_process[column];
            next[row] = std::tanh(next[row] + layer.bias[row]);
        }
        values_to_process = std::move(next);
    }
    std::array<double, 2> action{};
    for (std::size_t row = 0; row < 2; ++row) {
        double value = action_.bias[row];
        for (std::size_t column = 0; column < action_.columns; ++column) value += action_.weights[row * action_.columns + column] * values_to_process[column];
        action[row] = std::clamp(value, -1.0, 1.0);
    }
    return action;
}

std::array<double, 2> PPOPolicy::torque(const robot_interface::RobotState& state,
                                          const navigation::PlanarPose& pose,
                                          std::array<double, 2> goal,
                                          const GoalObservationBuilder& builder) const {
    const auto observation = builder.build(state, pose, goal);
    const auto action = normalized_action(observation);
    return {config_.torque_limit * action[0], config_.torque_limit * action[1]};
}

}
