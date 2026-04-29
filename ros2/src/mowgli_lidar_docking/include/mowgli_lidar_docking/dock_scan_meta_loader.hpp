// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
#pragma once

// dock_scan_meta.yaml reader/writer (Phase 2 D-17 schema, flat key=value).
//
// File schema (10 fields, all required):
//   dock_scan_pcd_path: <string, full path to the .pcd>
//   dock_pose_x: <double, metres in map frame>
//   dock_pose_y: <double, metres in map frame>
//   dock_pose_yaw_rad: <double, radians, dock yaw in map frame>
//   sensor_extrinsic_x: <double, metres, base→lidar X>
//   sensor_extrinsic_y: <double, metres, base→lidar Y>
//   sensor_extrinsic_yaw_rad: <double, radians, base→lidar yaw>
//   point_count: <int64, scan size at capture>
//   captured_at: <ISO-8601 string>
//   fix_type: <string, RTK_FIXED | RTK_FLOAT | etc.>
//
// The reader is `inline`; the writer + age-check helpers live in the .cpp
// because they pull in `mowgli_geometry/atomic_write.hpp` and the C++20
// chrono day-arithmetic we use to mitigate T-02-06 (year-rollover).

#include <mowgli_geometry/key_value_parser.hpp>

#include <cstdint>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

namespace mowgli_lidar_docking
{

/// 10-field record loaded from /ros2_ws/maps/dock_scan_meta.yaml.
/// Mirrors the D-17 schema and is round-trip-tested by Plan 02-02.
struct DockScanMeta
{
  std::string dock_scan_pcd_path;     ///< full path to the .pcd
  double dock_pose_x{0.0};            ///< map-frame X (metres)
  double dock_pose_y{0.0};            ///< map-frame Y (metres)
  double dock_pose_yaw_rad{0.0};      ///< map-frame yaw (radians)
  double sensor_extrinsic_x{0.0};     ///< base→lidar X (metres)
  double sensor_extrinsic_y{0.0};     ///< base→lidar Y (metres)
  double sensor_extrinsic_yaw_rad{0.0};  ///< base→lidar yaw (radians)
  std::int64_t point_count{0};        ///< number of points in the .pcd
  std::string captured_at;            ///< ISO-8601 (YYYY-MM-DDThh:mm:ssZ)
  std::string fix_type;               ///< "RTK_FIXED" | "RTK_FLOAT" | ...
};

/// Read `path` as a flat key=value YAML and parse a DockScanMeta. Returns
/// nullopt if any of the 10 required keys is absent or unparseable.
inline std::optional<DockScanMeta> load_dock_scan_meta_yaml(const std::string& path)
{
  std::ifstream f(path);
  if (!f.good()) return std::nullopt;
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string c = ss.str();

  const auto pcd = mowgli_geometry::parse_yaml_string(c, "dock_scan_pcd_path");
  const auto px = mowgli_geometry::parse_yaml_double(c, "dock_pose_x");
  const auto py = mowgli_geometry::parse_yaml_double(c, "dock_pose_y");
  const auto pyaw = mowgli_geometry::parse_yaml_double(c, "dock_pose_yaw_rad");
  const auto sx = mowgli_geometry::parse_yaml_double(c, "sensor_extrinsic_x");
  const auto sy = mowgli_geometry::parse_yaml_double(c, "sensor_extrinsic_y");
  const auto syaw =
      mowgli_geometry::parse_yaml_double(c, "sensor_extrinsic_yaw_rad");
  const auto pc = mowgli_geometry::parse_yaml_int(c, "point_count");
  const auto cap = mowgli_geometry::parse_yaml_string(c, "captured_at");
  const auto fix = mowgli_geometry::parse_yaml_string(c, "fix_type");

  if (!pcd || !px || !py || !pyaw || !sx || !sy || !syaw || !pc || !cap || !fix)
  {
    return std::nullopt;
  }

  return DockScanMeta{
      *pcd, *px, *py, *pyaw, *sx, *sy, *syaw,
      static_cast<std::int64_t>(*pc), *cap, *fix};
}

/// Write `meta` to `path` as a flat key=value YAML via
/// `mowgli_geometry::atomic_write`. Returns true on success.
///
/// Doubles use `std::fixed << std::setprecision(6)` (1e-6 m / rad
/// round-trip stability). `point_count` is written as a plain decimal
/// integer. Strings are written as bare tokens (no quotes).
bool save_dock_scan_meta_yaml(const std::string& path, const DockScanMeta& meta);

/// Helper for SPEC R-11 auto-refresh policy.
///
/// Returns true iff `meta.captured_at` is at least `min_age_days` calendar
/// days older than `now_iso`. Both timestamps are parsed via real day
/// arithmetic (not lex compare) so a January-1 boundary cannot trip a
/// false negative — see threat T-02-06 in the Plan 02-02 register.
///
/// Inputs accept either full ISO-8601 (`YYYY-MM-DDThh:mm:ssZ`) or the
/// date-only prefix (`YYYY-MM-DD`); only the first 10 characters are
/// consumed. Returns false on any parse failure (defensive default — a
/// failed parse must NOT silently trigger a re-capture).
bool dock_scan_meta_age_exceeds(const DockScanMeta& meta,
                                int min_age_days,
                                const std::string& now_iso);

}  // namespace mowgli_lidar_docking
