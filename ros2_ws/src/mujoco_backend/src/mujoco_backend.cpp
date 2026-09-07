#include "mujoco_backend/mujoco_backend.hpp"
#include <stdexcept>
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
        }
}