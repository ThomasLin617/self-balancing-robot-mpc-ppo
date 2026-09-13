#include "mujoco_backend/mujoco_viewer.hpp"
#include "mujoco_backend/mujoco_backend.hpp"

#include <stdexcept>
#include <cmath>

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


void MujocoViewer::enable_studio_lighting()
{
    studio_lighting_ = true;
}

void MujocoViewer::set_camera_azimuth(double degrees)
{
    camera_.azimuth = degrees;
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

    if (studio_lighting_) {
        // Reapply after updateScene, which rebuilds the scene lights each frame.
        // Directional lights keep their brightness as the robot travels.
        scene_.nlight = 3;
        scene_.flags[mjRND_SHADOW] = 1;
        const float directions[3][3] = {
            {-0.6f, -0.8f, -1.0f},  // warm key light, above the camera side
            {0.8f, 0.4f, -0.5f},    // cool fill light
            {-0.3f, 1.0f, -0.6f}    // rear rim light
        };
        const float colors[3][3] = {
            {0.48f, 0.46f, 0.42f},
            {0.23f, 0.26f, 0.30f},
            {0.20f, 0.22f, 0.25f}
        };
        for (int i = 0; i < 3; ++i) {
            auto& light = scene_.lights[i];
            light = mjvLight{};
            light.id = -1;
            light.texid = -1;
            light.type = mjLIGHT_DIRECTIONAL;
            light.attenuation[0] = 1.0f;
            light.castshadow = (i == 0);
            float norm = 0.0f;
            for (int j = 0; j < 3; ++j) {
                norm += directions[i][j] * directions[i][j];
            }
            norm = std::sqrt(norm);
            for (int j = 0; j < 3; ++j) {
                light.dir[j] = directions[i][j] / norm;
                light.pos[j] = static_cast<float>(camera_.lookat[j]) - 3.0f * light.dir[j];
                light.ambient[j] = (i == 0) ? 0.25f : 0.0f;
                light.diffuse[j] = colors[i][j];
                light.specular[j] = (i == 0) ? 0.08f : 0.03f;
            }
        }
    }

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
