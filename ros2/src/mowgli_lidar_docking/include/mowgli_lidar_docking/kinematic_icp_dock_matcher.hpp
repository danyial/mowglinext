// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
#pragma once

// Production IDockMatcher backed by kinematic_icp::pipeline::KinematicICP.
//
// Construction: takes a kiss_icp config + the dock_scan points (already
// in the dock-anchor frame, conventionally map-frame at the dock pose) +
// the dock_anchor SE3 in the map frame. Calls SetPose(dock_anchor) and
// then seeds the local VoxelHashMap directly via AddPoints — Plan 02-01
// PROBE.md A1 confirmed AddPoints is public on kiss_icp::VoxelHashMap.
//
// Match(live_frame, lidar_to_base): single-threads access to the kicp_
// pipeline behind a std::mutex (Open Q4 — Reload() may swap the pipeline
// out from under a Match call). RegisterFrame is called with identity
// for relative_odometry per CLAUDE.md AI #1 + RESEARCH Anti-Pattern: we
// never feed robot_localization output back into a LiDAR-derived signal.
// After registration, confidence is computed post-hoc via
// compute_confidence(registered_frame, voxel_map, max_corr_dist) — kiss_icp
// does not expose a confidence on RegisterFrame so we reconstruct it from
// nearest-neighbour queries on the public VoxelHashMap API (PROBE.md A2).
//
// Pitfall 1 (180° flip): yaw delta vs the dock_anchor is checked
// post-registration; > π/2 → MatchResult.valid = false (consumers gate on
// `trusted` derived from valid + confidence threshold). Throttled WARN log
// surfaces the rejection without spamming the journal.
//
// Reload(dock_points, dock_anchor): for the file-mtime watcher in
// dock_scan_match_node — atomically swaps the voxel map under the same
// mutex. Internally we destroy + reconstruct the kicp_ pipeline via
// std::optional<KinematicICP> rather than poking VoxelHashMap::Clear()
// (which IS public per kiss_icp v1.2.0, but using the constructor reset
// also resets the correspondence-threshold internals back to the same
// pristine state SetPose() would). This avoids mid-Match data corruption
// even in the unlikely event of a partial Reload.
//
// Architecture Invariant #1 honoured: this matcher does NOT publish TF,
// does NOT subscribe to /odometry/filtered_map, and does NOT store any
// state derived from the EKF output. The dock_anchor passed in at
// construction is a static map-frame measurement (loaded from
// dock_calibration.yaml on disk).

#include "mowgli_lidar_docking/idock_matcher.hpp"

#include <kinematic_icp/pipeline/KinematicICP.hpp>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <mutex>
#include <optional>
#include <vector>

namespace mowgli_lidar_docking
{

class KinematicIcpDockMatcher : public IDockMatcher
{
public:
  /// Construct + seed the voxel map.
  ///
  /// @param cfg            kiss_icp config (voxel_size, max_iters, etc.)
  /// @param dock_scan_points  Static dock geometry, expressed in the map
  ///                          frame at `dock_anchor_in_map`. The seeded
  ///                          voxel map will hold these points after the
  ///                          constructor returns.
  /// @param dock_anchor_in_map  The dock pose in the map frame. After
  ///                            construction `kicp_.pose() == dock_anchor`.
  /// @param max_correspondence_distance  Used for the post-hoc confidence
  ///                          metric (compute_confidence). Saved here
  ///                          because kicp_ does not expose its own
  ///                          correspondence-distance accessor.
  KinematicIcpDockMatcher(const kinematic_icp::pipeline::Config& cfg,
                          const std::vector<Eigen::Vector3d>& dock_scan_points,
                          const Sophus::SE3d& dock_anchor_in_map,
                          double max_correspondence_distance);

  // Non-copyable, non-movable — owns a std::mutex.
  KinematicIcpDockMatcher(const KinematicIcpDockMatcher&) = delete;
  KinematicIcpDockMatcher& operator=(const KinematicIcpDockMatcher&) = delete;
  KinematicIcpDockMatcher(KinematicIcpDockMatcher&&) = delete;
  KinematicIcpDockMatcher& operator=(KinematicIcpDockMatcher&&) = delete;

  /// IDockMatcher contract.
  MatchResult Match(const std::vector<Eigen::Vector3d>& live_frame,
                    const Sophus::SE3d& lidar_to_base) override;

  /// Replace the seeded voxel map with a fresh dock cloud at a new anchor.
  /// Thread-safe with respect to Match() — both share the same std::mutex.
  /// Used by dock_scan_match_node's mtime watcher (Open Q4): when
  /// dock_scan.pcd is rewritten on disk by the GUI Recapture button (Plan
  /// 02-07) or the auto-refresh trigger (Plan 02-06 R-11), the watcher
  /// reloads both files and calls Reload() to swap the voxel map without
  /// restarting the process.
  void Reload(const std::vector<Eigen::Vector3d>& dock_scan_points,
              const Sophus::SE3d& dock_anchor_in_map);

private:
  /// Helper used by both the constructor and Reload() — rebuilds the
  /// pipeline from cfg_ and seeds it. Caller must hold mtx_.
  void RebuildLocked(const std::vector<Eigen::Vector3d>& dock_scan_points,
                     const Sophus::SE3d& dock_anchor_in_map);

  kinematic_icp::pipeline::Config cfg_;
  double max_corr_;
  Sophus::SE3d dock_anchor_in_map_;
  std::optional<kinematic_icp::pipeline::KinematicICP> kicp_;
  std::mutex mtx_;
};

}  // namespace mowgli_lidar_docking
