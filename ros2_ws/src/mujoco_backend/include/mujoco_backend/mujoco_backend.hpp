#pragma once
#include <string>
#include <memory>
#include <mujoco/mujoco.h>
#include <robot_interface/sensor_data.hpp>
#include <robot_interface/robot_command.hpp>
namespace mujoco_backend
{
class MujocoViewer;

class MujocoBackend
{
public:
    explicit MujocoBackend(const std::string& model_path);
    double simulation_time() const;

    void step();

    robot_interface::SensorData read_sensor_data() const;
    void apply_command(const robot_interface::RobotCommand& command);
private:


    friend class MujocoViewer;


    std::string model_path_;
    struct ModelDeleter
    {
        void operator()(mjModel* model) const;
    };
    struct DataDeleter
    {
        void operator()(mjData* data) const;
    };

    int find_sensor_address(const char* sensor_name, int expected_dim) const;
    int find_actuator_id(const char* actuator_name) const;
    std::unique_ptr<mjModel, ModelDeleter> model_{nullptr};
    std::unique_ptr<mjData, DataDeleter> data_{nullptr};

    int gyro_adr_ = -1;
    int accel_adr_ = -1;
    int left_wheel_pos_adr_ = -1;
    int right_wheel_pos_adr_ = -1;
    int left_wheel_vel_adr_ = -1;
    int right_wheel_vel_adr_ = -1;

    int left_wheel_motor_id_ = -1;
    int right_wheel_motor_id_ = -1;
};

} // namespace mujoco_backend