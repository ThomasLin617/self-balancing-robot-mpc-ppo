#pragma once

#include <array>
#include <vector>
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
    void set_camera_overview(const std::array<double, 3>& center, double distance);
    void enable_studio_lighting();
    void set_reference_path(
        const std::vector<std::array<double, 3>>& points,
        const std::array<double, 3>& target
    );
    void set_planned_route(
        const std::vector<std::array<double, 3>>& raw_path,
        const std::vector<std::array<double, 3>>& waypoints
    );
    bool should_close() const;

private:
    MujocoBackend& backend_;

    GLFWwindow* window_ = nullptr;

    mjvCamera camera_{};
    mjvOption option_{};
    mjvScene scene_{};
    mjrContext context_{};
    bool studio_lighting_ = false;
    std::vector<std::array<double, 3>> reference_path_;
    std::array<double, 3> target_{};
    std::vector<std::array<double, 3>> raw_path_;
    std::vector<std::array<double, 3>> waypoints_;
};

}  // namespace mujoco_backend
