#include <chrono>
#include <learning_policy/trajectory_policy.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using learning_policy::TrajectoryPolicy;

std::pair<robot_interface::RobotState, robot_interface::ReferenceHorizon>
inputs(const std::array<double, 6> &values,
       py::array_t<double, py::array::c_style | py::array::forcecast> horizon) {
    if (horizon.ndim() != 2 || horizon.shape(0) != 250 || horizon.shape(1) != 5)
        throw std::invalid_argument("Invalid horizon.");
    robot_interface::RobotState state;
    state.forward_position = values[0];
    state.forward_velocity = values[1];
    state.pitch = values[2];
    state.pitch_rate = values[3];
    state.yaw = values[4];
    state.yaw_rate = values[5];
    robot_interface::ReferenceHorizon refs(250);
    auto input = horizon.unchecked<2>();
    for (int i = 0; i < 250; ++i) {
        refs[i].forward_position = input(i, 0);
        refs[i].forward_velocity = input(i, 1);
        refs[i].pitch = input(i, 2);
        refs[i].yaw = input(i, 3);
        refs[i].yaw_rate = input(i, 4);
    }
    return {state, refs};
}
PYBIND11_MODULE(_trajectory_policy, module) {
    py::class_<TrajectoryPolicy>(module, "TrajectoryPolicy")
        .def(py::init<const std::string &>())
        .def("normalized_action", &TrajectoryPolicy::normalized_action)
        .def("observation",
             [](const TrajectoryPolicy &p, const std::array<double, 6> &state,
                py::array_t<double> horizon) {
                 const auto in = inputs(state, horizon);
                 return p.observation(in.first, in.second);
             })
        .def("compute", [](const TrajectoryPolicy &p, const std::array<double, 6> &state,
                           py::array_t<double> horizon) {
            const auto in = inputs(state, horizon);
            const auto begin = std::chrono::steady_clock::now();
            const auto torque = p.torque(in.first, in.second);
            const auto end = std::chrono::steady_clock::now();
            return std::array<double, 3>{
                torque[0], torque[1],
                std::chrono::duration<double, std::micro>(end - begin).count()};
        });
}
