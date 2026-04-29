find_package(ament_cmake_gtest REQUIRED)

# Round-trip + corruption-detection + atomic-write coverage for the
# Checkpoint .kv format (SPEC R-10, RESEARCH §7).
ament_add_gtest(test_checkpoint test_checkpoint.cpp)
target_link_libraries(test_checkpoint mowgli_coverage_planner_lib)

# Skeleton smoke test: action endpoint live + node constructs cleanly
# (R-1 + R-2 acceptance).
ament_add_gtest(test_coverage_planner_skeleton test_coverage_planner_skeleton.cpp)
target_link_libraries(test_coverage_planner_skeleton mowgli_coverage_planner_lib)
ament_target_dependencies(test_coverage_planner_skeleton rclcpp rclcpp_action)
