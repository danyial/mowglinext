// Plan 02-09 (gap closure) regression smoke test.
//
// Pins the GAP-02 fix from
// .planning/phases/02-lidar-dock-pose-estimation/02-VERIFICATION.md
// § "Build Gaps".
//
// If a future CMakeLists "simplification" removes the add_subdirectory
// of kinematic_icp/cpp/kinematic_icp/ or removes the explicit link to
// kinematic_icp_pipeline, this test fails to compile with the same
// `No such file or directory` errors that originally surfaced GAP-02:
//
//   fatal error: kinematic_icp/pipeline/KinematicICP.hpp: No such file
//   fatal error: kiss_icp/core/VoxelHashMap.hpp: No such file
//
// Loud, deterministic, runs in <1 ms. Do not delete without first
// re-reading the gap-closure plan.

#include <gtest/gtest.h>

// The two headers that GAP-02 originally could not find. Direct includes
// (NOT through any mowgli wrapper) so the test fails at THIS file's
// translation unit if the include paths regress.
#include <kinematic_icp/pipeline/KinematicICP.hpp>
#include <kiss_icp/core/VoxelHashMap.hpp>

#include <Eigen/Core>

#include <vector>

namespace mowgli_lidar_docking::test {

// kiss_icp::VoxelHashMap public ctor signature per
// 02-01-PROBE.md § "kiss_icp::VoxelHashMap public API verdict":
//   explicit VoxelHashMap(double voxel_size,
//                         double max_distance,
//                         unsigned int max_points_per_voxel);
TEST(KinematicIcpHeadersReachable, KissIcpVoxelHashMapConstructible) {
  kiss_icp::VoxelHashMap voxel_map(/*voxel_size=*/0.10,
                                   /*max_distance=*/3.0,
                                   /*max_points_per_voxel=*/20);
  EXPECT_TRUE(voxel_map.Empty()) << "Freshly-constructed VoxelHashMap must be empty";

  // AddPoints — PROBE.md A1 confirmed public.
  std::vector<Eigen::Vector3d> points{
      {0.1, 0.0, 0.0}, {0.2, 0.0, 0.0}, {0.3, 0.0, 0.0}};
  voxel_map.AddPoints(points);
  EXPECT_FALSE(voxel_map.Empty()) << "After AddPoints, VoxelHashMap must be non-empty";

  // GetClosestNeighbor — PROBE.md A2 confirmed public.
  //
  // Note: PROBE.md (and the original Plan 02-09 author) believed the second
  // tuple element is a SQUARED distance. It is not. kiss_icp v1.2.0's
  // implementation in cpp/kiss_icp/core/VoxelHashMap.cpp returns
  // `(neighbor - query).norm()` — i.e. the Euclidean L2 distance. This was
  // confirmed empirically when this test first ran in-container against the
  // BUILD_TESTING=ON build of mowgli-phase2:test (commit fe96dea8): seeded
  // points at x=0.1/0.2/0.3, query at x=0.15, observed dist=0.05 (matches
  // ‖0.15-0.10‖=0.05) and not 0.0025 (=0.05^2).
  Eigen::Vector3d query(0.15, 0.0, 0.0);
  auto [neighbor, dist] = voxel_map.GetClosestNeighbor(query);
  // Euclidean distance from (0.15,0,0) to nearest of {0.1,0.2,0.3} is 0.05.
  EXPECT_NEAR(dist, 0.05, 1e-6);
  // Sanity check that the returned neighbor is one of the seeded points.
  const double tol = 1e-9;
  const bool match_p1 = std::abs(neighbor.x() - 0.1) < tol;
  const bool match_p2 = std::abs(neighbor.x() - 0.2) < tol;
  EXPECT_TRUE(match_p1 || match_p2)
      << "GetClosestNeighbor must return one of the seeded points (got x="
      << neighbor.x() << ")";
}

// kinematic_icp/pipeline/KinematicICP.hpp must compile-time include cleanly.
// We do NOT instantiate kinematic_icp::pipeline::KinematicICP here because
// its constructor takes a non-trivial Config struct that pulls in adaptive
// thresholding and registration internals. Plan 02-04 already exercises the
// full constructor path in test_kinematic_icp_dock_matcher.cpp; this smoke
// test only needs to assert the header is REACHABLE (the GAP-02 symptom).
TEST(KinematicIcpHeadersReachable, KinematicIcpHeaderCompiles) {
  // The mere fact that this file compiled (with the
  // `#include <kinematic_icp/pipeline/KinematicICP.hpp>` directive at the
  // top) is the test. We add a runtime assertion for clarity and so the
  // test produces a real PASS line in colcon test output.
  static_assert(sizeof(void*) >= 4,
                "Sanity: a pointer must be at least 4 bytes, otherwise "
                "the build environment is unfit for this assertion.");
  SUCCEED() << "kinematic_icp/pipeline/KinematicICP.hpp + "
               "kiss_icp/core/VoxelHashMap.hpp both reachable.";
}

}  // namespace mowgli_lidar_docking::test
