#include "mujoco_backend/mujoco_viewer.hpp"
#include "mujoco_backend/mujoco_backend.hpp"

#include <stdexcept>

namespace mujoco_backend
{

MujocoViewer::MujocoViewer(MujocoBackend& backend)
    : backend_(backend)
{
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW.");
    }

    window_ = glfwCreateWindow(
        1200,
        900,
        "Robot1 MuJoCo Viewer",
        nullptr,
        nullptr
    );

    if (window_ == nullptr) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window.");
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_makeScene(
        backend_.model_.get(),
        &scene_,
        2000
    );

    mjr_makeContext(
        backend_.model_.get(),
        &context_,
        mjFONTSCALE_150
    );

    camera_.lookat[0] = backend_.model_->stat.center[0];
    camera_.lookat[1] = backend_.model_->stat.center[1];
    camera_.lookat[2] = backend_.model_->stat.center[2];

    camera_.distance = 1.5 * backend_.model_->stat.extent;
    camera_.azimuth = 90.0;
    camera_.elevation = -20.0;
}


MujocoViewer::~MujocoViewer()
{
    if (window_ != nullptr) {
        glfwMakeContextCurrent(window_);

        mjv_freeScene(&scene_);
        mjr_freeContext(&context_);

        glfwDestroyWindow(window_);
        window_ = nullptr;
    }

    glfwTerminate();
}


void MujocoViewer::track_body(const char* body_name)
{
    const int body_id = mj_name2id(backend_.model_.get(), mjOBJ_BODY, body_name);
    if (body_id <= 0) {
        throw std::invalid_argument("Camera tracking requires a non-world body.");
    }
    camera_.type = mjCAMERA_TRACKING;
    camera_.trackbodyid = body_id;
}


void MujocoViewer::render()
{
    mjrRect viewport{0, 0, 0, 0};

    glfwGetFramebufferSize(
        window_,
        &viewport.width,
        &viewport.height
    );

    mjv_updateScene(
        backend_.model_.get(),
        backend_.data_.get(),
        &option_,
        nullptr,
        &camera_,
        mjCAT_ALL,
        &scene_
    );

    mjr_render(
        viewport,
        &scene_,
        &context_
    );

    glfwSwapBuffers(window_);
    glfwPollEvents();
}


bool MujocoViewer::should_close() const
{
    return glfwWindowShouldClose(window_) != 0;
}

}  // namespace mujoco_backend
