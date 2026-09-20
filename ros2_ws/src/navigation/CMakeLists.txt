cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_interface REQUIRED)

add_library(navigation_lib
  src/planar_odometry.cpp
  src/reference_trajectory.cpp
  src/occupancy_grid.cpp
  src/global_planner.cpp
  src/timed_trajectory.cpp
  src/path_trajectory.cpp
  src/known_map.cpp
)
target_compile_features(navigation_lib PUBLIC cxx_std_17)
target_include_directories(navigation_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(navigation_lib robot_interface)

install(TARGETS navigation_lib EXPORT export_navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_navigation HAS_LIBRARY_TARGET)
ament_export_dependencies(robot_interface)

if(BUILD_TESTING)
  add_executable(planning_test src/planning_test.cpp)
  target_link_libraries(planning_test navigation_lib)
  add_test(NAME known_map_planning COMMAND planning_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../model/known_map.txt)
  add_executable(navigation_test src/navigation_test.cpp)
  target_link_libraries(navigation_test navigation_lib)
  add_test(NAME navigation_math COMMAND navigation_test)
endif()

ament_package()
