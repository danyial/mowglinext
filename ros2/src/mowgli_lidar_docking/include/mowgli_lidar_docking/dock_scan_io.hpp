// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
#pragma once

// PCD save/load for dock_scan.pcd (Phase 2 D-06: PCL ASCII).
//
// Three entry points:
//   * save_dock_scan_pcd        - direct PCL write; used by Plan 02-03's
//                                 first-capture path where atomicity is not
//                                 required because the file is being created
//                                 fresh (operator-driven from GUI button).
//   * save_dock_scan_pcd_atomic - render PCL ASCII to a std::string and
//                                 hand it to mowgli_geometry::atomic_write
//                                 (temp file + fsync + rename + dir fsync).
//                                 Used by Plan 02-06's RecordDockApproachPose
//                                 auto-refresh path so a power loss mid-write
//                                 cannot corrupt the live PCD (SPEC R-11).
//   * load_dock_scan_pcd        - PCL ASCII read; returns false on any
//                                 I/O failure or malformed header.
//
// All three operate on `std::vector<Eigen::Vector3d>` so the kiss_icp /
// VoxelHashMap-feeding code path needs no PCL include.

#include <Eigen/Core>

#include <string>
#include <vector>

namespace mowgli_lidar_docking
{

/// Write `points` to `path` as a PCL ASCII PCD file (binary=false, D-06).
/// Returns true iff `pcl::io::savePCDFile` returns 0.
///
/// The file is created via PCL's standard I/O, so a power loss mid-write
/// can leave a truncated file on disk. Use `save_dock_scan_pcd_atomic`
/// for hot-path refresh writes (R-11).
bool save_dock_scan_pcd(const std::string& path,
                        const std::vector<Eigen::Vector3d>& points);

/// Atomic variant of `save_dock_scan_pcd`. Renders the same PCL ASCII
/// content into an in-memory std::string and writes it via
/// `mowgli_geometry::atomic_write`. Returns true iff atomic_write succeeds.
///
/// Used by Plan 02-06's auto-refresh trigger so a power loss between
/// "writer started" and "writer fsynced" cannot leave dock_scan.pcd in a
/// half-written state — at any reboot point the file is either the
/// previous valid PCD or the new valid PCD, never both.
bool save_dock_scan_pcd_atomic(const std::string& path,
                               const std::vector<Eigen::Vector3d>& points);

/// Load `path` as a PCL ASCII PCD into `out_points`. Returns true on
/// success. On failure (file missing, bad header, parse error) returns
/// false and `out_points` is left unchanged.
bool load_dock_scan_pcd(const std::string& path,
                        std::vector<Eigen::Vector3d>& out_points);

}  // namespace mowgli_lidar_docking
