cmake_minimum_required(VERSION 3.8)
project(mowgli_coverage_planner)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(mowgli_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# ---------------------------------------------------------------------------
# Library: mowgli_coverage_planner_lib
# Separating the node implementation from main.cpp lets gtest link the
# library without spinning up an executable. Plan 01-05 ships the action
# server skeleton + checkpoint .kv I/O; Plan 01-07 extends with validators
# and the plan builder.
# ---------------------------------------------------------------------------

add_library(mowgli_coverage_planner_lib STATIC
  src/coverage_planner_node.cpp
  src/checkpoint_io.cpp
  src/validators/validator_pipeline.cpp
  src/plan_builder/outline_generator.cpp
  src/plan_builder/boustrophedon_sweeper.cpp
  src/plan_builder/narrow_area_strategy.cpp
  src/plan_builder/plan_builder.cpp
)

target_include_directories(mowgli_coverage_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mowgli_coverage_planner_lib PUBLIC cxx_std_17)
target_link_libraries(mowgli_coverage_planner_lib PUBLIC
  mowgli_geometry
  Eigen3::Eigen
)

ament_target_dependencies(mowgli_coverage_planner_lib PUBLIC
  rclcpp
  rclcpp_action
  mowgli_interfaces
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# ---------------------------------------------------------------------------
# Executable: coverage_planner_node
# ---------------------------------------------------------------------------

add_executable(coverage_planner_node src/main.cpp)
target_link_libraries(coverage_planner_node mowgli_coverage_planner_lib)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(TARGETS coverage_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mowgli_coverage_planner_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(mowgli_coverage_planner_lib)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  mowgli_interfaces
  mowgli_geometry
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_package()
