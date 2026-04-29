// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Implementation notes:
//
//   * SetPose() vs AddPoints() ordering: SetPose() calls
//     local_map_.Clear() internally (KinematicICP.hpp:88), so we MUST
//     call AddPoints AFTER SetPose, not before. Doing it in the other
//     order silently drops the dock points.
//
//   * relative_odometry == identity per RESEARCH §Anti-Pattern: feeding
//     /odometry/filtered_map back here would violate Architecture
//     Invariant #1 (single-localizer rule). The kinematic prior's
//     regularisation keeps the registration honest at near-stationary
//     speeds, which is the dock-approach regime.
//
//   * Pitfall 1 (180° flip): yaw delta against dock_anchor is checked
//     post-registration. We use atan2 + shortest-angular-distance
//     manually (no rclcpp/tf2/angles deps in this lib — keep it as a
//     pure-C++ ICP wrapper).
//
//   * Reload() reconstructs the pipeline via std::optional::reset() +
//     emplace() so the correspondence-threshold internal state also
//     resets. This is cheaper than a "reset every member by hand"
//     approach and avoids any risk of stale adaptive-threshold history
//     poisoning a fresh capture.

#include "mowgli_lidar_docking/kinematic_icp_dock_matcher.hpp"

#include "mowgli_lidar_docking/confidence_metrics.hpp"

#include <cmath>
#include <limits>

namespace mowgli_lidar_docking
{

namespace
{

// Extract yaw (rotation about Z) from a Sophus::SE3d. Mirrors tf2::getYaw
// without bringing tf2 into this lib's surface.
double yaw_of(const Sophus::SE3d& pose)
{
  // SE3 → rotation matrix → yaw via standard atan2(R(1,0), R(0,0)).
  // Robust against the ±π discontinuity because atan2 returns in (-π, π].
  const Eigen::Matrix3d R = pose.rotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

// Wrap an angle delta to (-π, π]. Used for the 180° flip detector.
double shortest_angular_distance(double a, double b)
{
  double d = a - b;
  while (d > M_PI) d -= 2.0 * M_PI;
  while (d <= -M_PI) d += 2.0 * M_PI;
  return d;
}

}  // namespace

KinematicIcpDockMatcher::KinematicIcpDockMatcher(
    const kinematic_icp::pipeline::Config& cfg,
    const std::vector<Eigen::Vector3d>& dock_scan_points,
    const Sophus::SE3d& dock_anchor_in_map,
    double max_correspondence_distance)
    : cfg_(cfg),
      max_corr_(max_correspondence_distance),
      dock_anchor_in_map_(dock_anchor_in_map)
{
  std::lock_guard<std::mutex> lock(mtx_);
  RebuildLocked(dock_scan_points, dock_anchor_in_map);
}

void KinematicIcpDockMatcher::RebuildLocked(
    const std::vector<Eigen::Vector3d>& dock_scan_points,
    const Sophus::SE3d& dock_anchor_in_map)
{
  // Destroy + reconstruct the pipeline. Resets correspondence-threshold,
  // registration weights, the adaptive odometry regulariser — everything
  // back to a known fresh state.
  kicp_.reset();
  kicp_.emplace(cfg_);

  // PROBE.md A1: kiss_icp::VoxelHashMap::AddPoints is public on the
  // struct (default access). The flow is:
  //   1. SetPose(anchor)   — sets last_pose_ = anchor; clears local_map_.
  //   2. AddPoints(dock)   — populates the now-empty local_map_ with the
  //                          dock geometry.
  // Reversing the order would silently drop the dock points (Clear()
  // wipes them).
  kicp_->SetPose(dock_anchor_in_map);
  kicp_->VoxelMap().AddPoints(dock_scan_points);

  dock_anchor_in_map_ = dock_anchor_in_map;
}

MatchResult KinematicIcpDockMatcher::Match(
    const std::vector<Eigen::Vector3d>& live_frame,
    const Sophus::SE3d& lidar_to_base)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (live_frame.empty() || !kicp_)
  {
    return MatchResult{Sophus::SE3d{}, 0.0,
                       std::numeric_limits<double>::infinity(), false};
  }

  // RESEARCH §Anti-Pattern: relative_odometry MUST be identity. Feeding
  // /odometry/filtered_map here would feed robot_localization output
  // back into a LiDAR-derived signal — violates AI #1.
  auto [registered_frame, kpoints] = kicp_->RegisterFrame(
      live_frame,
      /*timestamps=*/std::vector<double>{},
      lidar_to_base,
      /*relative_odometry=*/Sophus::SE3d{});
  (void)kpoints;  // unused — confidence comes from registered_frame ↔ map

  const Sophus::SE3d pose_in_map = kicp_->pose();
  const auto conf =
      compute_confidence(registered_frame, kicp_->VoxelMap(), max_corr_);

  // Pitfall 1: 180° flip detector. If the post-registration yaw deviates
  // by more than π/2 from the dock anchor's yaw, the matcher has
  // hallucinated a flipped pose. Reject by setting valid=false; the
  // confidence stays low organically because a flipped frame won't
  // actually align well in inlier_ratio either, but we belt-and-brace
  // the trust gate.
  const double yaw_pose = yaw_of(pose_in_map);
  const double yaw_anchor = yaw_of(dock_anchor_in_map_);
  const double dyaw = std::abs(shortest_angular_distance(yaw_pose, yaw_anchor));
  const bool flipped = (dyaw > M_PI / 2.0);

  return MatchResult{
      pose_in_map,
      conf.inlier_ratio,
      conf.rmse_m,
      /*valid=*/!flipped,
  };
}

void KinematicIcpDockMatcher::Reload(
    const std::vector<Eigen::Vector3d>& dock_scan_points,
    const Sophus::SE3d& dock_anchor_in_map)
{
  std::lock_guard<std::mutex> lock(mtx_);
  RebuildLocked(dock_scan_points, dock_anchor_in_map);
}

}  // namespace mowgli_lidar_docking
