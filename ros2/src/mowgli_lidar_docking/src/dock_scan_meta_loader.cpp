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
//   * save_dock_scan_meta_yaml writes 10 key=value lines via
//     mowgli_geometry::atomic_write so a power loss mid-write cannot leave
//     a half-formed file (T-02-05 mitigation, mirrors save_dock_approach).
//
//   * dock_scan_meta_age_exceeds parses the date prefix (YYYY-MM-DD) of
//     both timestamps via std::tm + std::mktime → time_t day-arithmetic.
//     Lex compare on the raw ISO strings (RESEARCH §A5) is the documented
//     failure mode we explicitly avoid: it gives the wrong answer the
//     moment one timestamp is on a leap year and the other isn't, or when
//     the operator's clock is skewed across a year boundary. Real day
//     arithmetic is the safe path. We use the C-style API rather than
//     std::chrono::sys_days because Kilted's apt toolchain (g++-13 on
//     Debian-bookworm) shipped libstdc++ before <chrono>'s
//     `from_stream`/year_month_day was complete on all platforms.

#include "mowgli_lidar_docking/dock_scan_meta_loader.hpp"

#include <mowgli_geometry/atomic_write.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace mowgli_lidar_docking
{

namespace
{

/// Parse the leading YYYY-MM-DD of `iso` into a time_t representing
/// midnight UTC on that day. Returns std::nullopt if the prefix is too
/// short, malformed, or rejected by std::mktime.
std::optional<std::time_t> parse_iso_date_to_time_t(const std::string& iso)
{
  if (iso.size() < 10) return std::nullopt;

  // Manual numeric extraction — avoids locale dependencies in
  // std::get_time. The format is fixed at YYYY-MM-DD (chars 0..9).
  if (iso[4] != '-' || iso[7] != '-') return std::nullopt;
  for (std::size_t i : {0u, 1u, 2u, 3u, 5u, 6u, 8u, 9u})
  {
    if (iso[i] < '0' || iso[i] > '9') return std::nullopt;
  }

  const int year = std::stoi(iso.substr(0, 4));
  const int month = std::stoi(iso.substr(5, 2));
  const int day = std::stoi(iso.substr(8, 2));

  std::tm tm_utc{};
  tm_utc.tm_year = year - 1900;
  tm_utc.tm_mon = month - 1;
  tm_utc.tm_mday = day;
  tm_utc.tm_hour = 0;
  tm_utc.tm_min = 0;
  tm_utc.tm_sec = 0;
  tm_utc.tm_isdst = 0;

  // timegm is non-standard but available on glibc / musl which is what
  // the Kilted base image uses. It interprets tm as UTC, avoiding the
  // local-timezone bias that std::mktime would introduce.
  const std::time_t t = ::timegm(&tm_utc);
  if (t == static_cast<std::time_t>(-1)) return std::nullopt;
  return t;
}

}  // namespace

bool save_dock_scan_meta_yaml(const std::string& path, const DockScanMeta& meta)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(6);
  os << "dock_scan_pcd_path: " << meta.dock_scan_pcd_path << "\n";
  os << "dock_pose_x: " << meta.dock_pose_x << "\n";
  os << "dock_pose_y: " << meta.dock_pose_y << "\n";
  os << "dock_pose_yaw_rad: " << meta.dock_pose_yaw_rad << "\n";
  os << "sensor_extrinsic_x: " << meta.sensor_extrinsic_x << "\n";
  os << "sensor_extrinsic_y: " << meta.sensor_extrinsic_y << "\n";
  os << "sensor_extrinsic_yaw_rad: " << meta.sensor_extrinsic_yaw_rad << "\n";
  // point_count is an integer; the std::fixed+setprecision applies only
  // to floating-point operands, so this prints as a plain decimal.
  os << "point_count: " << meta.point_count << "\n";
  os << "captured_at: " << meta.captured_at << "\n";
  os << "fix_type: " << meta.fix_type << "\n";
  return mowgli_geometry::atomic_write(path, os.str());
}

bool dock_scan_meta_age_exceeds(const DockScanMeta& meta,
                                int min_age_days,
                                const std::string& now_iso)
{
  if (min_age_days < 0) return false;  // defensive

  const auto captured = parse_iso_date_to_time_t(meta.captured_at);
  const auto now = parse_iso_date_to_time_t(now_iso);
  if (!captured || !now) return false;  // T-02-06: parse failure → no auto-refresh

  // 86400 s/day; integer arithmetic avoids drift.
  const std::time_t delta_s = *now - *captured;
  if (delta_s < 0) return false;  // clock skew → don't refresh
  const long long delta_days = static_cast<long long>(delta_s) / 86400LL;
  return delta_days >= static_cast<long long>(min_age_days);
}

}  // namespace mowgli_lidar_docking
