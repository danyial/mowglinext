// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
#pragma once
// Shared key=value flat YAML scanner used by all dock_*.yaml loaders
// (Phase 2 D-17 / Phase 1 D-05 family).
//
// Replaces the inline parse_yaml_double copies that used to live in
// hardware_bridge_node.cpp and map_server_node.cpp. New consumers
// (dock_approach_loader, dock_scan_meta_loader added by later Phase 2 plans)
// live in mowgli_lidar_docking and use this header.
//
// Format contract: one `key: value\n` per line, no nesting, no anchors,
// no quotes. Values are bare tokens. The parser anchors on beginning-of-
// line so distinct keys with shared prefixes (e.g. dock_pose_x vs
// dock_pose_x_offset, or dock_pose_x vs sensor_extrinsic_x) do not alias.
//
// Note on cross-format compatibility: yaml-cpp-style nested files written
// as `dock_calibration:\n  dock_pose_x: 1.0\n` (i.e. with leading
// whitespace before the key) are NOT readable by this parser by design.
// The Phase 2 plan migrates `dock_calibration.yaml` writers to flat key=value
// in step with promoting this parser. See Phase 2 SUMMARY 02-01 for the
// migration coordination notes.
//
// All four functions are header-only / inline (mowgli_geometry is a
// header-only INTERFACE library, Phase 1 D-02 convention).

#include <cstdint>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace mowgli_geometry {

namespace detail {

/// Find the byte offset of the value portion immediately after `key:` in
/// `content`, requiring `key:` to be either at content start or directly
/// preceded by `\n`. Returns std::nullopt if no anchored match exists.
inline std::optional<std::size_t> find_key_pos(const std::string& content,
                                               const std::string& key)
{
  const std::string needle = key + ":";

  // Anchor at content start.
  if (content.size() >= needle.size() &&
      content.compare(0, needle.size(), needle) == 0)
  {
    return needle.size();
  }

  // Otherwise must be preceded by a newline.
  const std::string anchored = "\n" + needle;
  const auto pos = content.find(anchored);
  if (pos == std::string::npos)
  {
    return std::nullopt;
  }
  return pos + anchored.size();
}

/// Trim leading horizontal whitespace and locate end-of-line (CR or LF).
/// Returns [value_start, value_end) byte offsets with trailing whitespace
/// also trimmed.
inline std::pair<std::size_t, std::size_t> trim_to_eol(
    const std::string& content, std::size_t pos)
{
  while (pos < content.size() &&
         (content[pos] == ' ' || content[pos] == '\t'))
  {
    ++pos;
  }
  std::size_t end = pos;
  while (end < content.size() && content[end] != '\n' && content[end] != '\r')
  {
    ++end;
  }
  while (end > pos &&
         (content[end - 1] == ' ' || content[end - 1] == '\t'))
  {
    --end;
  }
  return {pos, end};
}

}  // namespace detail

/// Parse the value of `key` from `content` as a double. Returns nullopt if
/// the key is absent, the value is empty, or std::stod throws.
inline std::optional<double> parse_yaml_double(const std::string& content,
                                               const std::string& key)
{
  const auto pos_opt = detail::find_key_pos(content, key);
  if (!pos_opt) return std::nullopt;
  const auto [pos, end] = detail::trim_to_eol(content, *pos_opt);
  if (pos == end) return std::nullopt;
  try
  {
    return std::stod(content.substr(pos, end - pos));
  }
  catch (...)
  {
    return std::nullopt;
  }
}

/// Parse the value of `key` from `content` as an unquoted string token.
/// Returns nullopt if the key is absent or its value is empty.
inline std::optional<std::string> parse_yaml_string(const std::string& content,
                                                    const std::string& key)
{
  const auto pos_opt = detail::find_key_pos(content, key);
  if (!pos_opt) return std::nullopt;
  const auto [pos, end] = detail::trim_to_eol(content, *pos_opt);
  if (pos == end) return std::nullopt;
  return content.substr(pos, end - pos);
}

/// Parse the value of `key` from `content` as a 64-bit signed integer.
/// Returns nullopt if the key is absent, the value is empty, or
/// std::stoll throws.
inline std::optional<std::int64_t> parse_yaml_int(const std::string& content,
                                                  const std::string& key)
{
  const auto pos_opt = detail::find_key_pos(content, key);
  if (!pos_opt) return std::nullopt;
  const auto [pos, end] = detail::trim_to_eol(content, *pos_opt);
  if (pos == end) return std::nullopt;
  try
  {
    return static_cast<std::int64_t>(std::stoll(content.substr(pos, end - pos)));
  }
  catch (...)
  {
    return std::nullopt;
  }
}

/// Three-field dock pose loaded from /ros2_ws/maps/dock_calibration.yaml
/// (or any equivalent flat key=value file with `dock_pose_x`,
/// `dock_pose_y`, `dock_pose_yaw_rad`). Verbatim relocation from the
/// inline copies that used to live in hardware_bridge_node.cpp and
/// map_server_node.cpp.
struct DockCalibrationFile
{
  double x{0.0};
  double y{0.0};
  double yaw_rad{0.0};
};

/// Load a `DockCalibrationFile` from a flat key=value file at `path`.
/// Returns nullopt if the file cannot be opened, or if any of the three
/// required keys (`dock_pose_x`, `dock_pose_y`, `dock_pose_yaw_rad`) is
/// missing or unparseable.
inline std::optional<DockCalibrationFile> load_dock_calibration_file(
    const std::string& path)
{
  std::ifstream f(path);
  if (!f.good()) return std::nullopt;
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string content = ss.str();
  const auto x = parse_yaml_double(content, "dock_pose_x");
  const auto y = parse_yaml_double(content, "dock_pose_y");
  const auto yaw = parse_yaml_double(content, "dock_pose_yaw_rad");
  if (!x || !y || !yaw) return std::nullopt;
  return DockCalibrationFile{*x, *y, *yaw};
}

}  // namespace mowgli_geometry
