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

# ---------------------------------------------------------------------------
# Plan 02-04 Task 1 unit tests.
#
# KinematicIcpDockMatcher production path: constructor seeding (PROBE.md A1
# AddPoints), 80%-noise rejection, 180° flip detection (Pitfall 1), Reload()
# swap correctness (Open Q4 mtime watcher), concurrent Match+Reload race-free
# (std::mutex protects kicp_).
# ---------------------------------------------------------------------------
ament_add_gtest(test_kinematic_icp_dock_matcher test_kinematic_icp_dock_matcher.cpp)
target_link_libraries(test_kinematic_icp_dock_matcher mowgli_lidar_docking_lib)

# ---------------------------------------------------------------------------
# Plan 02-04 Task 2 unit tests.
#
# DockScanMatchNode contract: degraded mode on missing PCD (Open Q3),
# publish-rate baseline (R-2), corrupted-scan trust drop (R-3 acceptance),
# mtime watcher path (Open Q4), TF distance gate (Pitfall 5), Pitfall 6
# baseline trusted=false on first tick. Uses MockMatcher to avoid the
# kiss_icp dependency in the gtest harness.
# ---------------------------------------------------------------------------
ament_add_gtest(test_dock_scan_match_node test_dock_scan_match_node.cpp)
target_link_libraries(test_dock_scan_match_node mowgli_lidar_docking_lib)

# ---------------------------------------------------------------------------
# Plan 02-09 (gap closure) regression smoke test.
#
# Pins the GAP-02 fix: kinematic_icp/pipeline/KinematicICP.hpp +
# kiss_icp/core/VoxelHashMap.hpp must remain reachable from any future
# mowgli_lidar_docking translation unit. If a future CMakeLists
# "simplification" removes the add_subdirectory of kinematic_icp cpp/
# or removes the kinematic_icp_pipeline link, THIS test fails to build
# loudly with the same error that originally surfaced GAP-02.
# ---------------------------------------------------------------------------
ament_add_gtest(test_kinematic_icp_headers_reachable test_kinematic_icp_headers_reachable.cpp)
target_link_libraries(test_kinematic_icp_headers_reachable mowgli_lidar_docking_lib)
