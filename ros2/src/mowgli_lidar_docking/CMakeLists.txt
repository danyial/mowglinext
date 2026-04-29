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
# `kinematic_icp` is the in-tree submodule populated by Plan 02-01. It
# transitively pulls kiss_icp via FetchContent_Declare (see
# .planning/phases/02-lidar-dock-pose-estimation/02-01-PROBE.md "Submodule
# layout note"). The kiss_icp::core target — which confidence_metrics.cpp
# links against in Task 2 — is wired through `ament_target_dependencies(...
# kinematic_icp)`; we deliberately avoid hand-naming
# `kinematic_icp::pipeline` here because Plan 02-01 PROBE.md confirmed
# kiss_icp's struct-default-public access pattern means we can use the
# headers without claiming a per-component CMake target that may not
# exist on every checkout.
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
  src/dock_approach_loader.cpp
  src/dock_scan_meta_loader.cpp
  # src/confidence_metrics.cpp added by Plan 02-02 Task 2.
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
target_link_libraries(mowgli_lidar_docking_lib PUBLIC
  Sophus::Sophus
  Eigen3::Eigen
  ${PCL_LIBRARIES}
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
# Tests
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # add_subdirectory(test) — enabled by Plan 02-02 Task 2 once gtest
  # binaries land alongside confidence_metrics.cpp.
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
