cmake_minimum_required(VERSION 3.18)
project(trajectory_bridge LANGUAGES CXX)
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)
set(CORE ${CMAKE_CURRENT_SOURCE_DIR}/../../ros2_ws/src)
pybind11_add_module(_trajectory bridge.cpp
  ${CORE}/navigation/src/planar_odometry.cpp
  ${CORE}/navigation/src/reference_trajectory.cpp
  ${CORE}/navigation/src/occupancy_grid.cpp
  ${CORE}/navigation/src/global_planner.cpp
  ${CORE}/navigation/src/timed_trajectory.cpp
  ${CORE}/navigation/src/path_trajectory.cpp
  ${CORE}/navigation/src/known_map.cpp
  ${CORE}/mujoco_backend/src/known_map_world.cpp)
target_compile_features(_trajectory PRIVATE cxx_std_17)
target_include_directories(_trajectory PRIVATE
  ${CORE}/navigation/include ${CORE}/robot_interface/include ${CORE}/mujoco_backend/include)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
pybind11_add_module(_controllers controller_bridge.cpp ${CORE}/controller/src/mpc_controller.cpp)
target_compile_features(_controllers PRIVATE cxx_std_17)
target_include_directories(_controllers PRIVATE ${CORE}/controller/include ${CORE}/robot_interface/include)
target_link_libraries(_controllers PRIVATE Eigen3::Eigen OsqpEigen::OsqpEigen)
pybind11_add_module(_trajectory_policy policy_bridge.cpp ${CORE}/learning_policy/src/trajectory_policy.cpp)
target_compile_features(_trajectory_policy PRIVATE cxx_std_17)
target_include_directories(_trajectory_policy PRIVATE ${CORE}/learning_policy/include ${CORE}/robot_interface/include)
add_executable(d18_compute_timing compute_timing.cpp
  ${CORE}/controller/src/mpc_controller.cpp
  ${CORE}/learning_policy/src/trajectory_policy.cpp
  ${CORE}/navigation/src/reference_trajectory.cpp
  ${CORE}/navigation/src/timed_trajectory.cpp)
target_compile_features(d18_compute_timing PRIVATE cxx_std_17)
target_include_directories(d18_compute_timing PRIVATE
  ${CORE}/controller/include ${CORE}/learning_policy/include
  ${CORE}/navigation/include ${CORE}/robot_interface/include)
target_link_libraries(d18_compute_timing PRIVATE Eigen3::Eigen OsqpEigen::OsqpEigen)
