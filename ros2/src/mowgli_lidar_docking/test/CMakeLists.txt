find_package(ament_cmake_gtest REQUIRED)

# ---------------------------------------------------------------------------
# Plan 02-02 Task 2 unit tests.
#
# Five gtest binaries pinning the contract surface of mowgli_lidar_docking.
# Every test links the static mowgli_lidar_docking_lib so the production
# code path (including kiss_icp + PCL transitive deps) is exercised.
# ---------------------------------------------------------------------------

# PCL ASCII round-trip + atomic-write + missing-file path (D-06, R-11).
ament_add_gtest(test_dock_scan_io test_dock_scan_io.cpp)
target_link_libraries(test_dock_scan_io mowgli_lidar_docking_lib)

# Inlier ratio + RMSE math; trust gating across 4 scenarios (R-3).
ament_add_gtest(test_confidence_metrics test_confidence_metrics.cpp)
target_link_libraries(test_confidence_metrics mowgli_lidar_docking_lib)

# dock_approach.yaml flat key=value round-trip + invalid source +
# missing field (D-04, T-02-01).
ament_add_gtest(test_dock_approach_loader test_dock_approach_loader.cpp)
target_link_libraries(test_dock_approach_loader mowgli_lidar_docking_lib)

# dock_scan_meta.yaml flat key=value round-trip + missing field +
# 7-day age threshold (D-17, R-11, T-02-06).
ament_add_gtest(test_dock_scan_meta_loader test_dock_scan_meta_loader.cpp)
target_link_libraries(test_dock_scan_meta_loader mowgli_lidar_docking_lib)

# IDockMatcher contract: a Mock implementation derives cleanly from the
# abstract base and returns operator-supplied fixed pose + confidence
# (D-16). Plan 02-06 BT tests reuse this pattern.
ament_add_gtest(test_idock_matcher_mock test_idock_matcher_mock.cpp)
target_link_libraries(test_idock_matcher_mock mowgli_lidar_docking_lib)
