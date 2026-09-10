cmake_minimum_required(VERSION 3.8)
project(mujoco_backend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_interface REQUIRED)
find_package(PkgConfig REQUIRED)



find_package(state_estimator REQUIRED)


pkg_check_modules(GLFW3 REQUIRED IMPORTED_TARGET glfw3)

set(MUJOCO_FRAMEWORK_DIR
  "/Applications/MuJoCo.app/Contents/Frameworks"
)
set(MUJOCO_LIBRARY
  "${MUJOCO_FRAMEWORK_DIR}/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
)

# The library's install name starts with @rpath/mujoco.framework.
set(CMAKE_BUILD_RPATH "${MUJOCO_FRAMEWORK_DIR}")

add_library(
  mujoco_backend_lib
  src/mujoco_backend.cpp
  src/mujoco_viewer.cpp
)
target_compile_features(
  mujoco_backend_lib
  PUBLIC
    cxx_std_17
)
target_include_directories(
  mujoco_backend_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(
  mujoco_backend_lib
  PUBLIC
    "-F${MUJOCO_FRAMEWORK_DIR}"
)
target_link_libraries(
  mujoco_backend_lib
  PUBLIC
    "${MUJOCO_LIBRARY}"
    "PkgConfig::GLFW3"
)
ament_target_dependencies(
  mujoco_backend_lib
  robot_interface
)
add_executable(
  backend_smoke_test
  src/backend_smoke_test.cpp
)
target_link_libraries(
  backend_smoke_test
  PRIVATE
    mujoco_backend_lib
)


add_executable(
  estimator_smoke_test
  src/estimator_smoke_test.cpp
)

target_link_libraries(
  estimator_smoke_test
  mujoco_backend_lib
)

ament_target_dependencies(
  estimator_smoke_test
  robot_interface
  state_estimator
)
install(
  TARGETS estimator_smoke_test
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
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
