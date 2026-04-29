// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
#pragma once

// dock_approach.yaml reader/writer (Phase 2 D-04 schema, flat key=value).
//
// File schema (5 fields, all required):
//   dock_approach_x: <double, metres in map frame>
//   dock_approach_y: <double, metres in map frame>
//   dock_approach_yaw_to_dock_rad: <double, radians, body→dock yaw>
//   source: lidar | tf
//   captured_at: <ISO-8601 string, e.g. 2026-04-29T15:30:00Z>
//
// The reader is `inline` and lives in this header so callers don't need a
// .cpp dependency. The writer lives in `dock_approach_loader.cpp` because
// the atomic-write path needs `mowgli_geometry/atomic_write.hpp`, which
// pulls in POSIX headers we'd rather keep out of the public header.
//
// Mitigates T-02-01: any `source` value other than "lidar" or "tf" is
// rejected as a load failure (returns nullopt).

#include <mowgli_geometry/key_value_parser.hpp>

#include <fstream>
#include <optional>
#include <sstream>
#include <string>

namespace mowgli_lidar_docking
{

/// 5-field record loaded from /ros2_ws/maps/dock_approach.yaml. All fields
/// are required; any missing or unparseable field yields nullopt.
struct DockApproach
{
  double x{0.0};                 ///< map-frame X (metres)
  double y{0.0};                 ///< map-frame Y (metres)
  double yaw_to_dock_rad{0.0};   ///< body→dock yaw (radians)
  std::string source;            ///< "lidar" or "tf" (T-02-01 mitigation)
  std::string captured_at;       ///< ISO-8601 timestamp string
};

/// Read `path` as a flat key=value YAML and parse a DockApproach.
/// Returns nullopt if:
///   * the file cannot be opened;
///   * any of the 5 required keys is absent or unparseable;
///   * `source` is anything other than "lidar" or "tf".
inline std::optional<DockApproach> load_dock_approach_yaml(const std::string& path)
{
  std::ifstream f(path);
  if (!f.good()) return std::nullopt;
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string content = ss.str();

  const auto x = mowgli_geometry::parse_yaml_double(content, "dock_approach_x");
  const auto y = mowgli_geometry::parse_yaml_double(content, "dock_approach_y");
  const auto yaw =
      mowgli_geometry::parse_yaml_double(content, "dock_approach_yaw_to_dock_rad");
  const auto source = mowgli_geometry::parse_yaml_string(content, "source");
  const auto captured = mowgli_geometry::parse_yaml_string(content, "captured_at");

  if (!x || !y || !yaw || !source || !captured) return std::nullopt;
  if (*source != "lidar" && *source != "tf") return std::nullopt;

  return DockApproach{*x, *y, *yaw, *source, *captured};
}

/// Write `approach` to `path` as a flat key=value YAML via
/// `mowgli_geometry::atomic_write`. Returns true on success, false on any
/// I/O failure or rename failure (T-02-05 mitigation).
///
/// Doubles are formatted with `std::fixed << std::setprecision(6)` so a
/// round-trip is bit-stable to ~1e-6 m / rad.
bool save_dock_approach_yaml(const std::string& path,
                             const DockApproach& approach);

}  // namespace mowgli_lidar_docking
