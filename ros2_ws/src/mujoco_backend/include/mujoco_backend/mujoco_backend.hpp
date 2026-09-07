#pragma once
#include <string>
#include <memory>
#include <mujoco/mujoco.h>

namespace mujoco_backend
{
class MujocoBackend
{
public:
    explicit MujocoBackend(const std::string& model_path);


private:
    std::string model_path_;
    struct ModelDeleter
    {
        void operator()(mjModel* model) const;
    };
    struct DataDeleter
    {
        void operator()(mjData* data) const;
    };
    
    std::unique_ptr<mjModel, ModelDeleter> model_{nullptr};
    std::unique_ptr<mjData, DataDeleter> data_{nullptr};
};

} // namespace mujoco_backend