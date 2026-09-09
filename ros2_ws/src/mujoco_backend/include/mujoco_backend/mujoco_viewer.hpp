#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

namespace mujoco_backend
{

class MujocoBackend;

class MujocoViewer
{
public:
    explicit MujocoViewer(MujocoBackend& backend);
    ~MujocoViewer();

    void render();
    bool should_close() const;

private:
    MujocoBackend& backend_;

    GLFWwindow* window_ = nullptr;

    mjvCamera camera_{};
    mjvOption option_{};
    mjvScene scene_{};
    mjrContext context_{};
};

}  // namespace mujoco_backend