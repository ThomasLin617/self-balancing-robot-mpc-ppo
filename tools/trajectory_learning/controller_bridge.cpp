#include <chrono>
#include <cmath>
#include <controller/mpc_controller.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_controllers, module) {
    py::class_<controller::MPCController>(module, "MPC")
        .def(py::init<int, double>(), py::arg("horizon") = 250, py::arg("torque_limit") = 2.0)
        .def("compute",
             [](controller::MPCController &controller, const std::array<double, 6> &values,
                py::array_t<double, py::array::c_style | py::array::forcecast> horizon) {
                 if (horizon.ndim() != 2 || horizon.shape(0) != controller.horizon_size() ||
                     horizon.shape(1) != 5)
                     throw std::invalid_argument("Incorrect MPC reference dimensions.");
                 for (double value : values)
                     if (!std::isfinite(value))
                         throw std::invalid_argument("Non-finite MPC state.");
                 robot_interface::RobotState state;
                 state.forward_position = values[0];
                 state.forward_velocity = values[1];
                 state.pitch = values[2];
                 state.pitch_rate = values[3];
                 state.yaw = values[4];
                 state.yaw_rate = values[5];
                 robot_interface::ReferenceHorizon references(controller.horizon_size());
                 auto input = horizon.unchecked<2>();
                 for (int i = 0; i < controller.horizon_size(); ++i) {
                     for (int j = 0; j < 5; ++j)
                         if (!std::isfinite(input(i, j)))
                             throw std::invalid_argument("Non-finite MPC reference.");
                     references[i].forward_position = input(i, 0);
                     references[i].forward_velocity = input(i, 1);
                     references[i].pitch = input(i, 2);
                     references[i].yaw = input(i, 3);
                     references[i].yaw_rate = input(i, 4);
                 }
                 const auto begin = std::chrono::steady_clock::now();
                 const auto command = controller.compute_command(state, references);
                 const auto end = std::chrono::steady_clock::now();
                 return std::array<double, 3>{
                     command.left_wheel_torque, command.right_wheel_torque,
                     std::chrono::duration<double, std::micro>(end - begin).count()};
             });
}
