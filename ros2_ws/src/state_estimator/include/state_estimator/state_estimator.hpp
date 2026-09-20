#pragma once
#include <robot_interface/sensor_data.hpp>
#include <robot_interface/robot_state.hpp>
namespace state_estimator
{

class StateEstimator
{
public:
    explicit StateEstimator(double pitch_filter_time_constant, double wheel_radius);

    robot_interface::RobotState update(
        const robot_interface::SensorData& sensor_data
    );
    void reset();


private:
    double pitch_filter_time_constant_ = 0.0;
    double previous_timestamp_ = 0.0;
    double pitch_estimate_ = 0.0;
    double yaw_estimate_ = 0.0;
    bool initialized_ = false;

    double wheel_radius_ = 0.0;

    double initial_forward_position_ = 0.0;

};

}