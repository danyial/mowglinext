// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/checkpoint_io.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <mowgli_interfaces/msg/checkpoint.hpp>

#include <mowgli_geometry/atomic_write.hpp>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Checkpoint .kv format (RESEARCH §7.1):
//   current_outline_index=<uint32>
//   current_swath_index=<uint32>
//   swath_direction=FORWARD|REVERSE
//   last_completed_swath_index=<uint32>
//   next_open_swath_index=<uint32>
//   last_mow_angle_deg=<float, 6 decimals>
//   last_swath_endpoint_x=<float, 6 decimals>
//   last_swath_endpoint_y=<float, 6 decimals>
//   last_swath_endpoint_yaw=<float, 6 decimals>
//
// area_index is encoded in the filename, NOT the body.

namespace mowgli_coverage_planner
{

namespace
{
using Checkpoint = mowgli_interfaces::msg::Checkpoint;

const char* swath_direction_to_token(std::uint8_t v)
{
  if (v == Checkpoint::SWATH_DIRECTION_FORWARD) return "FORWARD";
  if (v == Checkpoint::SWATH_DIRECTION_REVERSE) return "REVERSE";
  return nullptr;  // serializer caller-side guards against unknown values.
}

std::optional<std::uint8_t> swath_direction_from_token(const std::string& tok)
{
  if (tok == "FORWARD") return Checkpoint::SWATH_DIRECTION_FORWARD;
  if (tok == "REVERSE") return Checkpoint::SWATH_DIRECTION_REVERSE;
  return std::nullopt;
}

std::string trim(const std::string& s)
{
  std::size_t b = 0;
  while (b < s.size() &&
         (s[b] == ' ' || s[b] == '\t' || s[b] == '\r' || s[b] == '\n'))
    ++b;
  std::size_t e = s.size();
  while (e > b &&
         (s[e - 1] == ' ' || s[e - 1] == '\t' ||
          s[e - 1] == '\r' || s[e - 1] == '\n'))
    --e;
  return s.substr(b, e - b);
}

}  // namespace

std::string serialize_checkpoint(const Checkpoint& ck)
{
  // Extract yaw from the quaternion via tf2 (covers the orientation -> yaw
  // path; T-05-02 guard at the write_checkpoint_file layer rejects NaN).
  const double yaw = tf2::getYaw(ck.last_swath_endpoint.orientation);

  std::ostringstream os;
  os << std::fixed << std::setprecision(6);
  os << "current_outline_index=" << ck.current_outline_index << '\n';
  os << "current_swath_index=" << ck.current_swath_index << '\n';
  const char* dir = swath_direction_to_token(ck.swath_direction);
  // Fallback to FORWARD if the field is somehow out of range — the parser
  // would otherwise reject the file. The write_checkpoint_file layer
  // validates the input first, so this branch is defensive only.
  os << "swath_direction=" << (dir ? dir : "FORWARD") << '\n';
  os << "last_completed_swath_index=" << ck.last_completed_swath_index << '\n';
  os << "next_open_swath_index=" << ck.next_open_swath_index << '\n';
  os << "last_mow_angle_deg=" << ck.last_mow_angle_deg << '\n';
  os << "last_swath_endpoint_x=" << ck.last_swath_endpoint.position.x << '\n';
  os << "last_swath_endpoint_y=" << ck.last_swath_endpoint.position.y << '\n';
  os << "last_swath_endpoint_yaw=" << yaw << '\n';
  return os.str();
}

std::optional<Checkpoint> parse_kv(const std::string& content)
{
  std::map<std::string, std::string> kv;
  std::istringstream is(content);
  std::string line;
  while (std::getline(is, line))
  {
    const auto eq = line.find('=');
    if (eq == std::string::npos) continue;
    const std::string key = trim(line.substr(0, eq));
    const std::string val = trim(line.substr(eq + 1));
    if (key.empty()) continue;
    kv[key] = val;
  }

  // Required keys per RESEARCH §7.1.
  static const char* const required_keys[] = {
      "current_outline_index",
      "current_swath_index",
      "swath_direction",
      "last_completed_swath_index",
      "next_open_swath_index",
      "last_mow_angle_deg",
      "last_swath_endpoint_x",
      "last_swath_endpoint_y",
      "last_swath_endpoint_yaw",
  };
  for (const char* k : required_keys)
  {
    if (kv.find(k) == kv.end()) return std::nullopt;
  }

  Checkpoint ck;
  ck.area_index = 0u;  // caller fills from the filename.
  try
  {
    ck.current_outline_index =
        static_cast<std::uint32_t>(std::stoul(kv["current_outline_index"]));
    ck.current_swath_index =
        static_cast<std::uint32_t>(std::stoul(kv["current_swath_index"]));
    auto dir = swath_direction_from_token(kv["swath_direction"]);
    if (!dir.has_value()) return std::nullopt;
    ck.swath_direction = *dir;
    ck.last_completed_swath_index = static_cast<std::uint32_t>(
        std::stoul(kv["last_completed_swath_index"]));
    ck.next_open_swath_index = static_cast<std::uint32_t>(
        std::stoul(kv["next_open_swath_index"]));
    ck.last_mow_angle_deg = std::stod(kv["last_mow_angle_deg"]);

    const double x = std::stod(kv["last_swath_endpoint_x"]);
    const double y = std::stod(kv["last_swath_endpoint_y"]);
    const double yaw = std::stod(kv["last_swath_endpoint_yaw"]);
    ck.last_swath_endpoint.position.x = x;
    ck.last_swath_endpoint.position.y = y;
    ck.last_swath_endpoint.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    ck.last_swath_endpoint.orientation.x = q.x();
    ck.last_swath_endpoint.orientation.y = q.y();
    ck.last_swath_endpoint.orientation.z = q.z();
    ck.last_swath_endpoint.orientation.w = q.w();
  }
  catch (...)
  {
    return std::nullopt;
  }

  return ck;
}

bool write_checkpoint_file(const std::string& areas_dir,
                           const Checkpoint& ck,
                           std::string* error_out)
{
  // T-05-02: refuse NaN / Inf endpoint coordinates before serialising.
  // (Yaw is derived from the quaternion via tf2::getYaw at serialise time;
  // a non-finite quaternion would also produce NaN there, so guard the
  // endpoint coords explicitly here.)
  if (!std::isfinite(ck.last_swath_endpoint.position.x) ||
      !std::isfinite(ck.last_swath_endpoint.position.y) ||
      !std::isfinite(ck.last_mow_angle_deg))
  {
    if (error_out)
      *error_out = "non-finite checkpoint values rejected";
    return false;
  }

  // T-05-01: area_index is uint32 -> std::to_string yields digit-only output.
  const std::string path =
      areas_dir + "/coverage_" + std::to_string(ck.area_index) + ".kv";
  const std::string content = serialize_checkpoint(ck);

  if (!mowgli_geometry::atomic_write(path, content))
  {
    if (error_out)
      *error_out = std::string("atomic_write failed for path: ") + path;
    return false;
  }
  return true;
}

std::optional<Checkpoint> read_checkpoint_file(const std::string& areas_dir,
                                               std::uint32_t area_index)
{
  const std::string path =
      areas_dir + "/coverage_" + std::to_string(area_index) + ".kv";
  std::ifstream f(path);
  if (!f.good()) return std::nullopt;
  std::stringstream ss;
  ss << f.rdbuf();
  auto parsed = parse_kv(ss.str());
  if (!parsed.has_value()) return std::nullopt;
  parsed->area_index = area_index;
  return parsed;
}

}  // namespace mowgli_coverage_planner
