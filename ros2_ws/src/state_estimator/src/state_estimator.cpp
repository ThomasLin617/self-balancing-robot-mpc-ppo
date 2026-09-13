#include <state_estimator/state_estimator.hpp>
#include <stdexcept>
#include <cmath>

namespace state_estimator
{
    StateEstimator::StateEstimator(double pitch_filter_time_constant, double radius)
        : pitch_filter_time_constant_(pitch_filter_time_constant),wheel_radius_(radius)
    {
        if (pitch_filter_time_constant <= 0.0){
            throw std::invalid_argument("Pitch filter time constant must be positive.");
        }
        if (radius <= 0.0){
            throw std::invalid_argument("Wheel radius must be positive.");
        }
    }
    void StateEstimator::reset(){
        initialized_ = false;
        pitch_estimate_ = 0.0;
        previous_timestamp_ = 0.0;
        initial_forward_position_ = 0.0;
    }
    robot_interface::RobotState  StateEstimator::update(
        const robot_interface::SensorData& sensor_data)
    {
        robot_interface::RobotState state;
        if(!initialized_){
            initialized_ = true;
            pitch_estimate_ = 0.0;
            initial_forward_position_ = 
                wheel_radius_ *
                (
                    (sensor_data.left_wheel_position
                    + sensor_data.right_wheel_position) / 2.0
                    + pitch_estimate_
                );
            previous_timestamp_ = sensor_data.timestamp;
        }
        else{
            double pitch_from_accel = std::atan2(-sensor_data.accelerometer[0],sensor_data.accelerometer[2]);
            double dt = sensor_data.timestamp - previous_timestamp_;
            if (dt <= 0.0){
                throw std::runtime_error("dt <= 0.");
            }
            double alpha = pitch_filter_time_constant_ / (pitch_filter_time_constant_ + dt);
            double pitch_from_gyro = pitch_estimate_ + sensor_data.gyroscope[1] * dt;
            pitch_estimate_ = alpha * pitch_from_gyro + (1.0 - alpha) * pitch_from_accel;
            previous_timestamp_ = sensor_data.timestamp;
        }
        double current_forward_position =
            wheel_radius_ *
            (
                (sensor_data.left_wheel_position
                + sensor_data.right_wheel_position) / 2.0
                + pitch_estimate_
            );
        state.forward_position = current_forward_position - initial_forward_position_;
        state.timestamp = sensor_data.timestamp;
        state.pitch = pitch_estimate_;
        state.left_wheel_position = sensor_data.left_wheel_position;
        state.left_wheel_velocity = sensor_data.left_wheel_velocity;
        state.right_wheel_position = sensor_data.right_wheel_position;
        state.right_wheel_velocity = sensor_data.right_wheel_velocity;
        state.pitch_rate = sensor_data.gyroscope[1];
        state.yaw_rate = sensor_data.gyroscope[2];
        state.forward_velocity = wheel_radius_* ((sensor_data.left_wheel_velocity + sensor_data.right_wheel_velocity)/2.0 + sensor_data.gyroscope[1]);
        return state;
    }
}