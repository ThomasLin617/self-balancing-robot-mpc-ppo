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
    void track_body(const char* body_name);
    void set_camera_azimuth(double degrees);
    void enable_studio_lighting();
    bool should_close() const;

private:
    MujocoBackend& backend_;

    GLFWwindow* window_ = nullptr;

    mjvCamera camera_{};
    mjvOption option_{};
    mjvScene scene_{};
    mjrContext context_{};
    bool studio_lighting_ = false;
};

}  // namespace mujoco_backend
