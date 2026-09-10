#include "mujoco_backend/mujoco_backend.hpp"
#include "robot_interface/sensor_data.hpp"
#include <stdexcept>
#include <algorithm>
namespace mujoco_backend
{

    void MujocoBackend::ModelDeleter::operator()(mjModel* model) const {
        mj_deleteModel(model);
    }
    void MujocoBackend::DataDeleter::operator()(mjData* data) const {
        mj_deleteData(data);
    }



    MujocoBackend::MujocoBackend(const std::string& model_path)
        :model_path_(model_path){
            char error[1024] = {};
            
            mjModel* raw_model = mj_loadXML(
                model_path_.c_str(),
                nullptr,
                error,
                sizeof(error)
            );
            if(raw_model == nullptr){
                throw std::runtime_error(error);
            }
            model_.reset(raw_model);

            mjData* raw_data = mj_makeData(model_.get());
            if(raw_data == nullptr){
                throw std::runtime_error("Failed to create MuJoCo data.");
            }

            data_.reset(raw_data);
            gyro_adr_ = find_sensor_address("imu_gyro", 3);
            accel_adr_ = find_sensor_address("imu_accel", 3);
            left_wheel_pos_adr_ = find_sensor_address("left_wheel_pos", 1);
            right_wheel_pos_adr_ = find_sensor_address("right_wheel_pos", 1);
            left_wheel_vel_adr_ = find_sensor_address("left_wheel_vel", 1);
            right_wheel_vel_adr_ = find_sensor_address("right_wheel_vel", 1);

            left_wheel_motor_id_ = find_actuator_id("left_wheel_motor");
            right_wheel_motor_id_ = find_actuator_id("right_wheel_motor");
        }

        
    double MujocoBackend::simulation_time() const{
        return data_.get()->time;
    }
    void MujocoBackend::step(){
        mj_step(model_.get(), data_.get());
    }
    robot_interface::SensorData MujocoBackend::read_sensor_data() const {
        robot_interface::SensorData sensor_data;
        sensor_data.timestamp = simulation_time();
//gyro
        sensor_data.gyroscope[0] = data_->sensordata[MujocoBackend::gyro_adr_];
        sensor_data.gyroscope[1] = data_->sensordata[MujocoBackend::gyro_adr_+1];
        sensor_data.gyroscope[2] = data_->sensordata[MujocoBackend::gyro_adr_+2];

// accelerometer
        sensor_data.accelerometer[0] = data_->sensordata[MujocoBackend::accel_adr_];
        sensor_data.accelerometer[1] = data_->sensordata[MujocoBackend::accel_adr_+1];
        sensor_data.accelerometer[2] = data_->sensordata[MujocoBackend::accel_adr_+2];
//positions
        sensor_data.left_wheel_position = data_->sensordata[MujocoBackend::left_wheel_pos_adr_];
        sensor_data.right_wheel_position = data_->sensordata[MujocoBackend::right_wheel_pos_adr_];

//velocity
        sensor_data.left_wheel_velocity = data_->sensordata[MujocoBackend::left_wheel_vel_adr_];
        sensor_data.right_wheel_velocity = data_->sensordata[MujocoBackend::right_wheel_vel_adr_];
        
        return sensor_data;
    }


    int MujocoBackend::find_sensor_address(const char* sensor_name, int expected_dim) const{
        int id = mj_name2id(
            model_.get(),
            mjOBJ_SENSOR,
            sensor_name
        );
        if (id < 0){
            throw std::runtime_error("Sensor not found.");
        }
        if(model_->sensor_dim[id] != expected_dim){
            throw std::runtime_error("Incorrect sensor dimension.");
        }
        int adr = model_->sensor_adr[id];
        return adr;
    }
    int MujocoBackend::find_actuator_id(const char* actuator_name) const{
        int id = mj_name2id(
            model_.get(),
            mjOBJ_ACTUATOR,
            actuator_name
        );
        if(id < 0){
            throw std::runtime_error("Actuator not found.");
        }
        return id;
    }

    void MujocoBackend::apply_command(const robot_interface::RobotCommand & command){
    data_->ctrl[left_wheel_motor_id_] = std::clamp(command.left_wheel_torque, -2.0, 2.0);
    data_->ctrl[right_wheel_motor_id_] = std::clamp(command.right_wheel_torque, -2.0, 2.0);
    }
    void MujocoBackend::reset(){
        mj_resetData(model_.get(), data_.get());
        mj_forward(model_.get(), data_.get());
    }
}



