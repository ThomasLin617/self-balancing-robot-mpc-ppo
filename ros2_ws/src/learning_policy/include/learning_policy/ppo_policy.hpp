#pragma once

#include <array>
#include <string>
#include <vector>
#include <robot_interface/robot_state.hpp>
#include <navigation/planar_odometry.hpp>

namespace learning_policy
{

struct GoalObservationConfig
{
    std::array<double, 6> scales{1.0, 1.0, 1.0, 0.35, 2.0, 2.0};
    double clip = 5.0;
    double torque_limit = 2.0;
};

class GoalObservationBuilder
{
public:
    explicit GoalObservationBuilder(GoalObservationConfig config = {});
    std::array<double, 6> build(const robot_interface::RobotState& state,
                                 const navigation::PlanarPose& pose,
                                 std::array<double, 2> goal) const;

private:
    GoalObservationConfig config_;
};

class PPOPolicy
{
public:
    explicit PPOPolicy(const std::string& path);
    const GoalObservationConfig& observation_config() const;
    std::array<double, 2> normalized_action(const std::array<double, 6>& observation) const;
    std::array<double, 2> torque(const robot_interface::RobotState& state,
                                 const navigation::PlanarPose& pose,
                                 std::array<double, 2> goal,
                                 const GoalObservationBuilder& builder) const;

private:
    struct Layer { std::size_t rows = 0; std::size_t columns = 0; std::vector<double> weights; std::vector<double> bias; };
    std::vector<Layer> hidden_;
    Layer action_;
    GoalObservationConfig config_;
};

}
