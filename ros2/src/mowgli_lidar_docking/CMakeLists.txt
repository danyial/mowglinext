cmake_minimum_required(VERSION 3.8)
project(mowgli_lidar_docking)

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
#
# `kinematic_icp` is the in-tree submodule populated by Plan 02-01 and
# built as an ament package by colcon. However, the upstream kinematic_icp
# ROS wrapper (ros2/src/kinematic_icp/ros/CMakeLists.txt) does NOT install
# nor ament_export the underlying C++ library headers — it only exports
# the kinematic_icp_ros wrapper headers under include/kinematic_icp_ros/.
# The cpp/ headers we actually consume from KinematicIcpDockMatcher
# (kinematic_icp/pipeline/KinematicICP.hpp) and confidence_metrics
# (kiss_icp/core/VoxelHashMap.hpp via FetchContent) are NOT reachable via
# `ament_target_dependencies(... kinematic_icp)`.
#
# Plan 02-02 PROBE.md (lines 47-52) flagged this exact integration risk;
# Plans 02-02 and 02-04 used the naive ament pattern anyway and it
# compiled-on-host but failed at the phase-end podman build with:
#
#   fatal error: kinematic_icp/pipeline/KinematicICP.hpp: No such file
#   fatal error: kiss_icp/core/VoxelHashMap.hpp: No such file
#
# See .planning/phases/02-lidar-dock-pose-estimation/02-VERIFICATION.md
# § "Build Gaps" GAP-02 for the full root-cause analysis and the three
# candidate fix paths (A self-add_subdirectory / B patch submodule /
# C vendor headers). This file implements Path A.
#
# Self-add_subdirectory pulls the kinematic_icp cpp/ source tree into
# OUR build directory; that tree's top-level CMakeLists already does
# `include(kiss_icp/kiss-icp.cmake)` which FetchContent's kiss_icp at
# the version pinned by the kinematic_icp submodule (v1.2.0 at HEAD).
# The resulting STATIC target `kinematic_icp_pipeline` exposes both
# the kinematic_icp/ headers (via its own target_include_directories)
# and the kiss_icp/ headers (via its PUBLIC link to kiss_icp_pipeline,
# which transitively pulls kiss_icp_core's INTERFACE_INCLUDE_DIRECTORIES).
#
# We KEEP find_package(kinematic_icp REQUIRED) and
# ament_target_dependencies(... kinematic_icp) below for the ROS-wrapper
# symbols that may surface in future plans, but we do NOT rely on them
# for the cpp/ headers — those flow exclusively through
# target_link_libraries(... kinematic_icp_pipeline).
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(mowgli_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(laser_geometry REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kinematic_icp REQUIRED)

# ---------------------------------------------------------------------------
# Pull the kinematic_icp cpp/ source tree into this build (GAP-02 fix).
#
# EXCLUDE_FROM_ALL: kinematic_icp's cpp tree only declares libraries
#   (kinematic_icp_pipeline, kinematic_icp_registration,
#   kinematic_icp_threshold) plus transitive kiss_icp libs — no
#   executables — but EXCLUDE_FROM_ALL is defence-in-depth against
#   future upstream changes that might add an example/test executable.
#
# Scoped binary directory `${CMAKE_CURRENT_BINARY_DIR}/kinematic_icp_cpp_for_mowgli`:
#   prevents collision with any other in-tree add_subdirectory of the
#   same source tree (currently none, but future-proof).
#
# CMAKE_SOURCE_DIR is colcon's <package>/ source root, so
#   ${CMAKE_SOURCE_DIR}/../kinematic_icp/cpp/kinematic_icp resolves to
#   ros2/src/kinematic_icp/cpp/kinematic_icp regardless of where colcon
#   places the build directory.
#
# USE_SYSTEM_* options: kinematic_icp's cpp top-level CMakeLists declares
#   USE_SYSTEM_EIGEN3 / USE_SYSTEM_SOPHUS / USE_SYSTEM_TSL-ROBIN-MAP /
#   USE_SYSTEM_TBB defaulted ON, which causes find_package(Eigen3) /
#   find_package(Sophus) / find_package(TBB). The Dockerfile Stage 1 apt
#   block already installs all four (libpcl-dev pulls Eigen3, libtbb-dev
#   pulls TBB, libsophus-dev was replaced by ros-kilted-sophus per
#   GAP-01 fix in commit 0e8b2038, and tsl-robin-map is FetchContent-fallback
#   if not found). No additional apt packages needed.
# ---------------------------------------------------------------------------
add_subdirectory(${CMAKE_SOURCE_DIR}/../kinematic_icp/cpp/kinematic_icp ${CMAKE_CURRENT_BINARY_DIR}/kinematic_icp_cpp_for_mowgli EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------------------
# Library: mowgli_lidar_docking_lib
#
# Static library that every gtest binary links against, and that Plan
# 02-04's executable will link against once
# kinematic_icp_dock_matcher.cpp lands. Splitting "lib + tests" (this plan)
# from "node + matcher impl" (Plan 02-04) keeps both within the ~50%
# context budget and lets Plan 02-04 freeze the contract surface up
# front.
# ---------------------------------------------------------------------------

add_library(mowgli_lidar_docking_lib STATIC
  src/dock_scan_io.cpp
  src/confidence_metrics.cpp
  src/dock_approach_loader.cpp
  src/dock_scan_meta_loader.cpp
  src/kinematic_icp_dock_matcher.cpp
  src/dock_scan_match_node.cpp
)

target_include_directories(mowgli_lidar_docking_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(mowgli_lidar_docking_lib PUBLIC cxx_std_17)

# Direct CMake links for libraries that are not ament packages or whose
# include paths must be picked up via target_link_libraries (PCL).
# Sophus::Sophus and Eigen3::Eigen are header-only IMPORTED targets;
# they propagate include paths via INTERFACE_INCLUDE_DIRECTORIES.
#
# kinematic_icp_pipeline is wrapped in $<BUILD_INTERFACE:...> on purpose:
#
#   - In the BUILD context (this colcon package's own .cpp files, our
#     dock_scan_match executable, and the regression smoke gtest in test/),
#     the link AND the propagated INTERFACE_INCLUDE_DIRECTORIES flow PUBLIC
#     so that <kinematic_icp/pipeline/KinematicICP.hpp> + (transitively via
#     kiss_icp_pipeline -> kiss_icp_core) <kiss_icp/core/VoxelHashMap.hpp>
#     resolve. That is what fixes GAP-02.
#
#   - In the INSTALL context (downstream packages like mowgli_behavior that
#     consume mowgli_lidar_docking_lib via ament's exported package config),
#     the link is intentionally stripped because kinematic_icp_pipeline is
#     pulled in via add_subdirectory + EXCLUDE_FROM_ALL and is therefore NOT
#     in any install(EXPORT) set. Without the BUILD_INTERFACE wrap, CMake
#     refuses install(EXPORT export_mowgli_lidar_docking_lib ...) with:
#       install(EXPORT ...) includes target "mowgli_lidar_docking_lib" which
#       requires target "kinematic_icp_pipeline" that is not in any export set.
#     This was the failure mode of the first Plan 02-09 podman build run; the
#     BUILD_INTERFACE wrapper is the surgical fix.
#
# Why this is safe for downstream consumers: mowgli_behavior only consumes
# the kinematic_icp-FREE public headers (dock_approach_loader.hpp,
# dock_scan_io.hpp, dock_scan_meta_loader.hpp). The kinematic_icp-bearing
# headers (kinematic_icp_dock_matcher.hpp, dock_scan_match_node.hpp) are
# only referenced by this package's own translation units. The static
# archive linker pulls in object files only for referenced symbols, so
# downstream linkers do not need to resolve kinematic_icp_pipeline symbols.
target_link_libraries(mowgli_lidar_docking_lib PUBLIC
  Sophus::Sophus
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  $<BUILD_INTERFACE:kinematic_icp_pipeline>
)

# ament dependencies — propagates include paths and downstream linkage
# for ROS2 packages. mowgli_geometry is header-only (Phase 1 D-02); its
# INTERFACE_INCLUDE_DIRECTORIES flow through here. kinematic_icp brings
# in the in-tree pipeline + transitively kiss_icp::core via FetchContent
# (Plan 02-01 PROBE.md).
ament_target_dependencies(mowgli_lidar_docking_lib PUBLIC
  rclcpp
  mowgli_interfaces
  mowgli_geometry
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  laser_geometry
  kinematic_icp
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS mowgli_lidar_docking_lib
  EXPORT export_mowgli_lidar_docking_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ---------------------------------------------------------------------------
# Executable: dock_scan_match (Plan 02-04 SPEC R-2 + R-3)
#
# Production binary spawned by mowgli_bringup/launch/navigation.launch.py
# under IfCondition(use_lidar). Subscribes /scan_kicp; publishes
# /dock_match/pose + /dock_match/confidence at 10 Hz (D-07). All trust
# thresholds + topic names + frame names are runtime-overridable via
# config/dock_scan_match.yaml.
# ---------------------------------------------------------------------------

add_executable(dock_scan_match src/main.cpp)
target_link_libraries(dock_scan_match mowgli_lidar_docking_lib)

install(TARGETS dock_scan_match
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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

ament_export_targets(export_mowgli_lidar_docking_lib HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  mowgli_interfaces
  mowgli_geometry
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  laser_geometry
  Sophus
  PCL
  Eigen3
  kinematic_icp
)

ament_package()
