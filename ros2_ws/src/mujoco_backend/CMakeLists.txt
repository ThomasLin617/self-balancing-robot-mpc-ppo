cmake_minimum_required(VERSION 3.8)
project(mujoco_backend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_interface REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(controller REQUIRED)
find_package(navigation REQUIRED)
find_package(state_estimator REQUIRED)
find_package(learning_policy REQUIRED)


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
add_executable(
  pd_smoke_test
  src/pd_smoke_test.cpp
)

target_link_libraries(
  pd_smoke_test
  mujoco_backend_lib
)

ament_target_dependencies(
  pd_smoke_test
  robot_interface
  state_estimator
  controller
)

add_executable(
  lqr_smoke_test
  src/lqr_smoke_test.cpp
)

target_link_libraries(
  lqr_smoke_test
  mujoco_backend_lib
)

ament_target_dependencies(
  lqr_smoke_test
  robot_interface
  state_estimator
  controller
)
add_executable(
  mpc_smoke_test
  src/mpc_smoke_test.cpp
)

target_link_libraries(
  mpc_smoke_test
  mujoco_backend_lib
)

ament_target_dependencies(
  mpc_smoke_test
  robot_interface
  state_estimator
  controller
)

install(
  TARGETS
    pd_smoke_test
    lqr_smoke_test
    mpc_smoke_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(yaw_sign_smoke_test src/yaw_sign_smoke_test.cpp)
target_link_libraries(yaw_sign_smoke_test mujoco_backend_lib)

add_executable(yaw_mpc_smoke_test src/yaw_mpc_smoke_test.cpp)
target_link_libraries(yaw_mpc_smoke_test mujoco_backend_lib)
ament_target_dependencies(yaw_mpc_smoke_test robot_interface state_estimator controller)

install(
  TARGETS yaw_sign_smoke_test yaw_mpc_smoke_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(navigation_smoke_test src/navigation_smoke_test.cpp)
target_link_libraries(navigation_smoke_test mujoco_backend_lib)
ament_target_dependencies(navigation_smoke_test robot_interface state_estimator controller navigation)
install(TARGETS navigation_smoke_test DESTINATION lib/${PROJECT_NAME})
set_target_properties(yaw_sign_smoke_test yaw_mpc_smoke_test navigation_smoke_test
  PROPERTIES INSTALL_RPATH "${MUJOCO_FRAMEWORK_DIR};${GLFW3_LIBRARY_DIRS}"
)

add_executable(known_map_smoke_test src/known_map_smoke_test.cpp src/known_map_world.cpp)
target_link_libraries(known_map_smoke_test mujoco_backend_lib)
ament_target_dependencies(known_map_smoke_test robot_interface state_estimator controller navigation)
set_target_properties(known_map_smoke_test
  PROPERTIES INSTALL_RPATH "${MUJOCO_FRAMEWORK_DIR};${GLFW3_LIBRARY_DIRS}"
)
install(TARGETS known_map_smoke_test DESTINATION lib/${PROJECT_NAME})

add_executable(ppo_smoke_test src/ppo_smoke_test.cpp)
target_link_libraries(ppo_smoke_test mujoco_backend_lib)
ament_target_dependencies(ppo_smoke_test robot_interface state_estimator navigation learning_policy)
set_target_properties(ppo_smoke_test PROPERTIES
  INSTALL_RPATH "${MUJOCO_FRAMEWORK_DIR};${GLFW3_LIBRARY_DIRS}")
install(TARGETS ppo_smoke_test DESTINATION lib/${PROJECT_NAME})

add_executable(controller_benchmark src/controller_benchmark.cpp)
target_link_libraries(controller_benchmark mujoco_backend_lib)
ament_target_dependencies(controller_benchmark robot_interface state_estimator navigation controller learning_policy)

add_executable(benchmark_audit_probe ../../../tools/benchmark/audit_probe.cpp)
ament_target_dependencies(benchmark_audit_probe robot_interface controller navigation)

if(BUILD_TESTING)
  add_test(NAME known_map_smoke COMMAND known_map_smoke_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../model/Robot1.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../model/known_map.txt --headless)
  add_test(NAME ppo_policy_parity COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../../ros2_ws/build/learning_policy/policy_parity_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/learning/selected_policy.txt
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/learning/selected_policy.parity.txt
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/learning/selected_policy.observations.txt)
  add_test(NAME ppo_smoke COMMAND ppo_smoke_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../model/Robot1.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/learning/selected_policy.txt
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/learning/selected_policy.scenario.txt --headless)
  add_test(NAME navigation_smoke COMMAND navigation_smoke_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../model/Robot1.xml --headless --duration 80)
  add_test(NAME yaw_sign COMMAND yaw_sign_smoke_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../model/Robot1.xml)
  foreach(mode forward yaw combined)
    add_test(NAME mpc_${mode} COMMAND yaw_mpc_smoke_test
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../model/Robot1.xml ${mode} --headless)
  endforeach()
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

add_executable(trajectory_policy_smoke_test src/trajectory_policy_smoke_test.cpp src/known_map_world.cpp)
target_link_libraries(trajectory_policy_smoke_test mujoco_backend_lib)
ament_target_dependencies(trajectory_policy_smoke_test robot_interface state_estimator navigation controller learning_policy)
install(TARGETS trajectory_policy_smoke_test DESTINATION lib/${PROJECT_NAME})

ament_package()
