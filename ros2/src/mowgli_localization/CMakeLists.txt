cmake_minimum_required(VERSION 3.8)
project(mowgli_localization)

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

find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(std_srvs           REQUIRED)
find_package(tf2                REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(mowgli_interfaces  REQUIRED)

# ---------------------------------------------------------------------------
# Common include directory macro
# ---------------------------------------------------------------------------

set(INCLUDE_DIR
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Executable: wheel_odometry_node
# ---------------------------------------------------------------------------

add_executable(wheel_odometry_node
  src/wheel_odometry_node.cpp
)

target_include_directories(wheel_odometry_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(wheel_odometry_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: navsat_to_absolute_pose_node
# ---------------------------------------------------------------------------

add_executable(navsat_to_absolute_pose_node
  src/navsat_to_absolute_pose_node.cpp
)

target_include_directories(navsat_to_absolute_pose_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(navsat_to_absolute_pose_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: localization_monitor_node
# ---------------------------------------------------------------------------

add_executable(localization_monitor_node
  src/localization_monitor_node.cpp
)

target_include_directories(localization_monitor_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(localization_monitor_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    wheel_odometry_node
    navsat_to_absolute_pose_node
    localization_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Python nodes (installed executable, run via `ros2 run mowgli_localization ...`).
# calibrate_imu_yaw_node.py: upstream Python with a single-bool service
# response + full-result topic publish (~/calibrate_status, transient_local) —
# the topic-split is what dodges the foxglove_bridge ↔ rmw_cyclonedds
# typesupport-dispatch bug that originally motivated the rclcpp port (#19).
# Replaces the rclcpp port (Decision A → A2 revision 2026-04-27) so we
# regain upstream's dock pre-phase + future mag-fitting code paths.
install(
  PROGRAMS
    scripts/calibrate_imu_yaw_node.py
    scripts/cog_to_imu.py
    scripts/dock_scan_capture.py
    scripts/dock_yaw_to_set_pose.py
    scripts/kinematic_icp_encoder_adapter.py
    scripts/kinematic_icp_scan_frame_relay.py
    scripts/mag_yaw_publisher.py
    scripts/wheel_kicp_blend.py
    scripts/wheel_odom_tf_node.py
  DESTINATION lib/${PROJECT_NAME}
)
# dock_scan_capture.py is a LIBRARY module imported by
# calibrate_imu_yaw_node (Plan 02-03 SPEC R-1), not a node entry point.
# Installing under lib/${PROJECT_NAME}/ alongside calibrate_imu_yaw_node
# keeps the flat `import dock_scan_capture` import path working at
# runtime (ros2 run prepends that directory to sys.path before exec'ing
# the script).

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto      REQUIRED)
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # ---- test_wheel_odometry ----
  # The kinematics test is pure C++ with no ROS2 dependencies; link only gtest.
  ament_add_gtest(test_wheel_odometry
    test/test_wheel_odometry.cpp
  )

  target_include_directories(test_wheel_odometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  # ---- Python tests (Plan 02-03: dock_scan_capture + calibrate hook-up) ----
  # conftest.py prepends scripts/ to sys.path so `import dock_scan_capture`
  # works without a setup.py (this package is ament_cmake, not
  # ament_python). The tests pytest.importorskip("rclpy") so they self-
  # disable in environments where rclpy is not on PYTHONPATH.
  ament_add_pytest_test(test_dock_scan_capture
    test/test_dock_scan_capture.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_calibrate_imu_yaw_capture
    test/test_calibrate_imu_yaw_capture.py
    TIMEOUT 60
  )
  # Plan 02-05: cascade extension for dock_yaw_to_set_pose seeder.
  ament_add_pytest_test(test_dock_yaw_seeder_cascade
    test/test_dock_yaw_seeder_cascade.py
    TIMEOUT 60
  )
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mowgli_interfaces
)

ament_package()
