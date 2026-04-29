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

# ---------------------------------------------------------------------------
# Plan 01-07 unit tests (geometry building blocks + validator pipeline).
# All tests share the polygon fixtures in test/fixtures/polygons.hpp.
# ---------------------------------------------------------------------------

# Include path for fixtures/ headers.
set(_test_include_dirs ${CMAKE_CURRENT_SOURCE_DIR})

ament_add_gtest(test_outline_generator test_outline_generator.cpp)
target_link_libraries(test_outline_generator mowgli_coverage_planner_lib)
target_include_directories(test_outline_generator PRIVATE ${_test_include_dirs})

ament_add_gtest(test_aabb_sweep test_aabb_sweep.cpp)
target_link_libraries(test_aabb_sweep mowgli_coverage_planner_lib)
target_include_directories(test_aabb_sweep PRIVATE ${_test_include_dirs})

ament_add_gtest(test_validation_pipeline test_validation_pipeline.cpp)
target_link_libraries(test_validation_pipeline mowgli_coverage_planner_lib)
target_include_directories(test_validation_pipeline PRIVATE ${_test_include_dirs})

ament_add_gtest(test_narrow_area_strategies test_narrow_area_strategies.cpp)
target_link_libraries(test_narrow_area_strategies mowgli_coverage_planner_lib)
target_include_directories(test_narrow_area_strategies PRIVATE ${_test_include_dirs})

ament_add_gtest(test_segment_type_invariants test_segment_type_invariants.cpp)
target_link_libraries(test_segment_type_invariants mowgli_coverage_planner_lib)
target_include_directories(test_segment_type_invariants PRIVATE ${_test_include_dirs})

# ---------------------------------------------------------------------------
# Plan 01-07 Task 2 unit tests (auto-rotate + resume).
# ---------------------------------------------------------------------------

ament_add_gtest(test_auto_rotate test_auto_rotate.cpp)
target_link_libraries(test_auto_rotate mowgli_coverage_planner_lib)
target_include_directories(test_auto_rotate PRIVATE ${_test_include_dirs})

ament_add_gtest(test_resume test_resume.cpp)
target_link_libraries(test_resume mowgli_coverage_planner_lib)
target_include_directories(test_resume PRIVATE ${_test_include_dirs})
