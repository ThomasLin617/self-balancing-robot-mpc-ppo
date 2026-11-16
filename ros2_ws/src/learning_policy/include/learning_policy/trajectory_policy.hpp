#pragma once
#include <array>
#include <robot_interface/reference_horizon.hpp>
#include <robot_interface/robot_state.hpp>
#include <string>
#include <vector>

namespace learning_policy {

class TrajectoryPolicy {
  public:
    explicit TrajectoryPolicy(const std::string &path);
    std::vector<double> observation(const robot_interface::RobotState &state,
                                    const robot_interface::ReferenceHorizon &references) const;
    std::array<double, 2> normalized_action(const std::vector<double> &observation) const;
    std::array<double, 2> torque(const robot_interface::RobotState &state,
                                 const robot_interface::ReferenceHorizon &references) const;

  private:
    struct Layer {
        std::size_t rows = 0, columns = 0;
        std::vector<double> weights, bias;
    };
    std::array<double, 6> scales_{};
    double clip_ = 5.0, torque_limit_ = 2.0;
    std::vector<Layer> layers_;
};
} // namespace learning_policy
