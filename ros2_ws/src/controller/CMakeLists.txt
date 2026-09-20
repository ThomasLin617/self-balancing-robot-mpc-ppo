cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

add_library(
  controller_lib
  src/pd_controller.cpp
  src/lqr_controller.cpp
  src/mpc_controller.cpp
)
target_include_directories(
  controller_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(
  controller_lib
  PUBLIC
    cxx_std_17
)
target_link_libraries(
  controller_lib
  PUBLIC
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)
ament_target_dependencies(
  controller_lib
  robot_interface
)
install(
  TARGETS controller_lib
  EXPORT export_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_targets(
  export_controller
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  robot_interface
  Eigen3
  OsqpEigen
)
if(BUILD_TESTING)
  add_executable(mpc_preview_test src/mpc_preview_test.cpp)
  target_link_libraries(mpc_preview_test controller_lib)
  add_test(NAME mpc_preview_math COMMAND mpc_preview_test)
  add_executable(yaw_mpc_test src/yaw_mpc_test.cpp)
  target_link_libraries(yaw_mpc_test controller_lib)
  add_test(NAME yaw_mpc_math COMMAND yaw_mpc_test)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
