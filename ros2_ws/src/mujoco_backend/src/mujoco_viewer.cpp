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


void MujocoViewer::set_camera_overview(const std::array<double, 3>& center, double distance) {
    if (!std::isfinite(distance) || distance <= 0.0) throw std::invalid_argument("Invalid camera distance.");
    for (double value : center) {
        if (!std::isfinite(value)) throw std::invalid_argument("Invalid camera center.");
    }
    camera_.type = mjCAMERA_FREE;
    for (int i = 0; i < 3; ++i) camera_.lookat[i] = center[i];
    camera_.distance = distance;
    camera_.elevation = -60.0;
}


void MujocoViewer::set_reference_path(
    const std::vector<std::array<double, 3>>& points,
    const std::array<double, 3>& target
) {
    if (points.size() > 500) {
        throw std::invalid_argument("Too many reference path points.");
    }
    reference_path_ = points;
    target_ = target;
}


void MujocoViewer::set_planned_route(
    const std::vector<std::array<double, 3>>& raw_path,
    const std::vector<std::array<double, 3>>& waypoints
) {
    if (raw_path.size() + waypoints.size() > 800) throw std::invalid_argument("Too many planner debug points.");
    raw_path_ = raw_path;
    waypoints_ = waypoints;
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

    if (!reference_path_.empty()) {
        const float path_color[4] = {0.1f, 0.65f, 1.0f, 1.0f};
        const float target_color[4] = {1.0f, 0.3f, 0.05f, 1.0f};
        if (scene_.ngeom + static_cast<int>(reference_path_.size()) > scene_.maxgeom) {
            throw std::runtime_error("Viewer scene has no room for the reference path.");
        }
        for (std::size_t i = 1; i < reference_path_.size(); ++i) {
            auto& geom = scene_.geoms[scene_.ngeom++];
            mjv_initGeom(&geom, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, path_color);
            mjv_connector(&geom, mjGEOM_CAPSULE, 0.008,
                          reference_path_[i - 1].data(), reference_path_[i].data());
            geom.category = mjCAT_DECOR;
        }
        const mjtNum size[3] = {0.05, 0.05, 0.05};
        auto& target = scene_.geoms[scene_.ngeom++];
        mjv_initGeom(&target, mjGEOM_SPHERE, size, target_.data(), nullptr, target_color);
        target.category = mjCAT_DECOR;
    }

    if (scene_.ngeom + static_cast<int>(raw_path_.size() + waypoints_.size()) > scene_.maxgeom) {
        throw std::runtime_error("Viewer scene has no room for planner geometry.");
    }
    const float raw_color[4] = {0.9f, 0.8f, 0.1f, 1.0f};
    const float waypoint_color[4] = {0.9f, 0.9f, 0.9f, 1.0f};
    for (std::size_t i = 1; i < raw_path_.size(); ++i) {
        auto& geom = scene_.geoms[scene_.ngeom++];
        mjv_initGeom(&geom, mjGEOM_LINE, nullptr, nullptr, nullptr, raw_color);
        mjv_connector(&geom, mjGEOM_LINE, 2.0, raw_path_[i - 1].data(), raw_path_[i].data());
        geom.category = mjCAT_DECOR;
    }
    for (const auto& waypoint : waypoints_) {
        const mjtNum size[3] = {0.025, 0.025, 0.025};
        auto& geom = scene_.geoms[scene_.ngeom++];
        mjv_initGeom(&geom, mjGEOM_SPHERE, size, waypoint.data(), nullptr, waypoint_color);
        geom.category = mjCAT_DECOR;
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
