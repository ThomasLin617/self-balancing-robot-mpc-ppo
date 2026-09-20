cmake_minimum_required(VERSION 3.8)
project(learning_policy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_interface REQUIRED)
find_package(navigation REQUIRED)

add_library(learning_policy_lib src/ppo_policy.cpp)
target_compile_features(learning_policy_lib PUBLIC cxx_std_17)
target_include_directories(learning_policy_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(learning_policy_lib robot_interface navigation)

install(TARGETS learning_policy_lib EXPORT export_learning_policy
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_learning_policy HAS_LIBRARY_TARGET)
ament_export_dependencies(robot_interface navigation)

if(BUILD_TESTING)
  add_executable(policy_parity_test src/policy_parity_test.cpp)
  target_link_libraries(policy_parity_test learning_policy_lib)
  ament_target_dependencies(policy_parity_test robot_interface navigation)
  add_test(NAME policy_parity COMMAND policy_parity_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/learning/selected_policy.txt
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/learning/selected_policy.parity.txt
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/learning/selected_policy.observations.txt)
endif()

ament_package()
