#include <mujoco_backend/known_map_world.hpp>
#include <navigation/known_map.hpp>
#include <navigation/path_trajectory.hpp>
#include <navigation/reference_preview.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace navigation;

std::array<double, 7> point_values(const TrajectoryPoint &p) {
    return {p.x_ref, p.y_ref,   p.forward_position_ref, p.forward_velocity_ref,
            0.0,     p.yaw_ref, p.yaw_rate_ref};
}

class StraightReference {
  public:
    explicit StraightReference(double distance)
        : positive_(0.16, 0.06), sign_(distance < 0 ? -1.0 : 1.0) {
        positive_.generate({0, 0, 0}, std::abs(distance), 0);
    }
    TrajectoryPoint sample(double time) const {
        auto p = positive_.sample(time);
        p.x_ref *= sign_;
        p.forward_position_ref *= sign_;
        p.forward_velocity_ref *= sign_;
        return p;
    }
    double duration() const {
        return positive_.duration();
    }

  private:
    ReferenceTrajectory positive_;
    double sign_;
};

PYBIND11_MODULE(_trajectory, module) {
    py::class_<StraightReference>(module, "StraightReference")
        .def(py::init<double>())
        .def("sample",
             [](const StraightReference &t, double time) { return point_values(t.sample(time)); })
        .def_property_readonly("duration", &StraightReference::duration)
        .def("horizon", [](const StraightReference &t, double time, double origin) {
            py::array_t<double> output({250, 5});
            auto a = output.mutable_unchecked<2>();
            for (int i = 0; i < 250; ++i) {
                auto p = t.sample(time + (i + 1) * 0.002);
                a(i, 0) = origin + p.forward_position_ref;
                a(i, 1) = p.forward_velocity_ref;
                a(i, 2) = 0;
                a(i, 3) = p.yaw_ref;
                a(i, 4) = p.yaw_rate_ref;
            }
            return output;
        });
    py::class_<ReferenceTrajectory>(module, "Trajectory")
        .def_static("hold",
                    [](double yaw) {
                        TrajectoryPoint point;
                        point.yaw_ref = yaw;
                        return ReferenceTrajectory(TimedTrajectory({{point, 0.0}, {point, 6.0}}));
                    })
        .def_static("turn",
                    [](double yaw) {
                        TrajectoryPoint start;
                        return ReferenceTrajectory(
                            TimedTrajectory(profile_turn(start, yaw, MotionLimits())));
                    })
        .def_static("turn_then_move",
                    [](double yaw, double distance) {
                        TrajectoryPoint start, end;
                        end.yaw_ref = yaw;
                        ReferenceTrajectory geometry(0.16, 0.06);
                        geometry.generate({0, 0, yaw}, distance * std::cos(yaw),
                                          distance * std::sin(yaw));
                        return ReferenceTrajectory(time_parameterize(
                            {{{start, end}, true}, {geometry.points(), false}}, MotionLimits()));
                    })
        .def_static("profiled_curve",
                    [](double x, double y, double yaw, double gx, double gy) {
                        ReferenceTrajectory geometry(0.16, 0.06);
                        geometry.generate({x, y, yaw}, gx, gy);
                        return ReferenceTrajectory(
                            TimedTrajectory(profile_motion(geometry.points(), MotionLimits())));
                    })
        .def_static("point_to_point",
                    [](double x, double y, double yaw, double gx, double gy) {
                        ReferenceTrajectory trajectory(0.16, 0.06);
                        trajectory.generate({x, y, yaw}, gx, gy);
                        return trajectory;
                    })
        .def("sample",
             [](const ReferenceTrajectory &t, double time) { return point_values(t.sample(time)); })
        .def("horizon",
             [](const ReferenceTrajectory &t, double time, double origin) {
                 robot_interface::ReferenceHorizon refs(250);
                 sample_reference_horizon(t, time, origin, 0.002, refs);
                 py::array_t<double> output({250, 5});
                 auto a = output.mutable_unchecked<2>();
                 for (int i = 0; i < 250; ++i) {
                     a(i, 0) = refs[i].forward_position;
                     a(i, 1) = refs[i].forward_velocity;
                     a(i, 2) = refs[i].pitch;
                     a(i, 3) = refs[i].yaw;
                     a(i, 4) = refs[i].yaw_rate;
                 }
                 return output;
             })
        .def_property_readonly("duration", &ReferenceTrajectory::duration);
    py::class_<KnownMap>(module, "KnownMap")
        .def_static("load", &KnownMap::load)
        .def_property_readonly("goal", [](const KnownMap &map) { return map.goal; })
        .def("direct_free",
             [](const KnownMap &map, double x, double y, double gx, double gy) {
                 MotionLimits limits;
                 return map.grid.inflated(limits.robot_radius + limits.safety_margin)
                     .segment_free({x, y}, {gx, gy});
             })
        .def("inside", [](const KnownMap &map, double x,
                          double y) { return map.grid.contains(map.grid.world_to_grid({x, y})); })
        .def("plan",
             [](const KnownMap &map, double x, double y, double yaw, double gx, double gy) {
                 MotionLimits limits;
                 const auto safe = map.grid.inflated(limits.robot_radius + limits.safety_margin);
                 const auto route = map.grid.inflated(limits.robot_radius + limits.safety_margin +
                                                      limits.smoothing_clearance);
                 const auto raw = AStarPlanner().plan(route, {x, y}, {gx, gy});
                 const auto waypoints = simplify_path(route, raw);
                 const auto timed =
                     time_parameterize(smooth_path(safe, waypoints, yaw, limits), limits);
                 validate_trajectory(safe, timed);
                 return ReferenceTrajectory(timed);
             })
        .def("footprint_free", [](const KnownMap &map, double x, double y) {
            return map.grid.inflated(MotionLimits().robot_radius).point_free({x, y});
        });
    py::class_<mujoco_backend::KnownMapWorld>(module, "World")
        .def(py::init<const std::string &, const KnownMap &>())
        .def_property_readonly("path", &mujoco_backend::KnownMapWorld::path);
}
