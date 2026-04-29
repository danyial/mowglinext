// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
#pragma once

// Post-hoc inlier_ratio + RMSE computation for the LiDAR dock-match step.
//
// kiss_icp's RegisterFrame does not publish a confidence — the metric is
// reconstructed here from the registered scan + the seeded VoxelHashMap
// (RESEARCH §Pattern 1). Plan 02-04's dock_scan_match_node calls this
// once per cycle to populate /dock_match/confidence (SPEC R-3 trust gate).
//
// Two overloads:
//
//   * compute_confidence(frame, voxel_map, max_d)
//       Production path. Uses the public kiss_icp::VoxelHashMap API
//       (Plan 02-01 PROBE.md A1/A2 confirmed both AddPoints and
//       GetClosestNeighbor are public on the struct). Constant-time
//       per query thanks to the hash map.
//
//   * compute_confidence_brute_force(frame, reference_cloud, max_d)
//       Test-only path. Linear scan over the reference cloud; same
//       inlier/RMSE math but with no VoxelHashMap setup. The 5 gtest
//       binaries below use this so they don't have to build the kiss_icp
//       infrastructure into the test harness.
//
// is_trusted(): R-3 gating helper. Returns true iff inlier_ratio is
// above min and RMSE is below max AND finite. Used by Plan 02-04's
// matcher to drive the `trusted` field of DockMatchConfidence.msg.
//
// Note: the production overload is declared here to avoid pulling
// kiss_icp/core/VoxelHashMap.hpp into every translation unit. The
// implementation lives in confidence_metrics.cpp where the include is
// scoped.

#include <Eigen/Core>

#include <limits>
#include <vector>

// Forward declare to avoid pulling kiss_icp into every translation unit.
namespace kiss_icp { struct VoxelHashMap; }

namespace mowgli_lidar_docking
{

/// Result of a single compute_confidence pass.
///
/// Defaults represent the "degraded matcher" state: inlier_ratio = 0,
/// rmse = +inf. is_trusted() must return false for these defaults.
struct ConfidenceResult
{
  double inlier_ratio{0.0};
  double rmse_m{std::numeric_limits<double>::infinity()};
};

/// Production overload. Iterates `registered_frame`; for each point
/// calls `voxel_map.GetClosestNeighbor(p)` and accumulates inliers /
/// SSE for points whose squared distance to their nearest neighbour
/// is at most `max_correspondence_distance²`.
///
/// Returns {inlier_ratio = inliers/N, rmse = sqrt(SSE/inliers)}.
/// Empty frame or zero inliers → {0.0, +inf}.
///
/// Pre-conditions:
///   * registered_frame is already in the voxel map's coordinate frame
///     (Plan 02-04 transforms via lidar_to_base + the ICP delta before
///     calling this);
///   * voxel_map has been seeded via AddPoints from dock_scan.pcd.
ConfidenceResult compute_confidence(
    const std::vector<Eigen::Vector3d>& registered_frame,
    const kiss_icp::VoxelHashMap& voxel_map,
    double max_correspondence_distance);

/// Test-only overload. Same math as the production path but uses a
/// brute-force linear scan over `reference_cloud` instead of a
/// VoxelHashMap lookup. O(N*M); fine for unit-test sizes (≤ 1000 points).
ConfidenceResult compute_confidence_brute_force(
    const std::vector<Eigen::Vector3d>& registered_frame,
    const std::vector<Eigen::Vector3d>& reference_cloud,
    double max_correspondence_distance);

/// SPEC R-3 trust gate. Returns true iff:
///   * r.rmse_m is finite (rules out the "no inliers" sentinel),
///   * r.inlier_ratio >= min_inlier_ratio,
///   * r.rmse_m <= max_rmse_m.
bool is_trusted(const ConfidenceResult& r,
                double min_inlier_ratio,
                double max_rmse_m);

}  // namespace mowgli_lidar_docking
