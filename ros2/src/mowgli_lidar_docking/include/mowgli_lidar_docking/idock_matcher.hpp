// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
#pragma once

// Abstract base contract for the LiDAR dock-match step (Phase 2 D-16).
//
// Plan 02-02 ships only this header + a MockMatcher in unit tests.
// Plan 02-04 derives `KinematicIcpDockMatcher : IDockMatcher` and wires
// it into the dock_scan_match ROS2 node.
// Plan 02-06 BT tests reuse the same abstract base via locally-defined
// MockMatchers so trust-gating logic can be asserted without standing up
// the real kiss_icp::VoxelHashMap.

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <limits>
#include <vector>

namespace mowgli_lidar_docking
{

/// One ICP registration result from an IDockMatcher.
///
/// Fields:
///   pose_in_map   - body pose in `map` frame after the ICP step. The matcher
///                   composes this from (initial guess) ⊕ (registration delta)
///                   ⊕ (lidar_to_base inverse). The dock_scan_match node
///                   publishes this on /dock_match/pose for the seeder cascade.
///   inlier_ratio  - fraction of registered_frame points whose nearest
///                   neighbour in the seeded VoxelHashMap is within
///                   `max_correspondence_distance`. Range [0, 1].
///   rmse_m        - sqrt(SSE / inliers) over the inlier subset, in metres.
///                   +inf when no inliers (matcher is in a degraded state).
///   valid         - false iff the matcher cannot run at all (e.g. dock_scan
///                   not yet captured, voxel map empty, scan empty). When
///                   false the other fields are at their default values and
///                   downstream code MUST treat the result as untrusted.
struct MatchResult
{
  Sophus::SE3d pose_in_map;
  double inlier_ratio{0.0};
  double rmse_m{std::numeric_limits<double>::infinity()};
  bool valid{false};
};

/// Abstract dock-pose matcher.
///
/// Implementations are responsible for:
///   * holding the seeded VoxelHashMap (production: kiss_icp::VoxelHashMap;
///     mock: arbitrary fixed return value);
///   * running one registration step against `live_frame`;
///   * returning a MatchResult with map-frame pose + confidence.
///
/// Caller responsibilities:
///   * provide `live_frame` already de-cropped to a sensible window around
///     the expected dock pose (D-05: ±3 m);
///   * provide `lidar_to_base`: the static SE3 transform from the lidar
///     frame (where the points live) to the body frame. Mirrors the
///     parallel-tree extrinsic owned by mowgli_localization's
///     kinematic_icp_scan_frame_relay (Architecture Invariant #1).
///
/// The interface is intentionally minimal: no streaming setters, no
/// per-call configuration, no publishers. Production state (voxel map,
/// ICP params) is constructor-injected by the matcher implementation.
class IDockMatcher
{
public:
  virtual ~IDockMatcher() = default;

  virtual MatchResult Match(
      const std::vector<Eigen::Vector3d>& live_frame,
      const Sophus::SE3d& lidar_to_base) = 0;
};

}  // namespace mowgli_lidar_docking
