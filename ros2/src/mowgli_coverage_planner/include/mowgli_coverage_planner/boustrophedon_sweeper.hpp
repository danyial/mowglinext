// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// SPEC R-8 footprint-centric AABB sweep with obstacle clipping (RESEARCH §3.3).
//
// 1. rot = pi/2 - mow_angle_rad   (rotate so strips run along Y in scan-frame)
// 2. rotated_area = rotate(area, rot); rotated_obstacles = rotate(obs, rot)
// 3. expanded_obstacles = offset_polygon_inward(o, -(robot_width/2 + outline_offset))
// 4. AABB(rotated_area) -> [min_x, max_x] x [min_y, max_y]
// 5. step = tool_width - strip_overlap
//    x_inset_innermost = robot_width/2 + outline_offset + (outline_passes-1)*step
//    x_first = AABB.min_x + x_inset_innermost + step/2
// 6. For each scan-line x in x_first .. AABB.max_x - x_inset_innermost step step:
//    - Compute y intersections of vertical line @ x with rotated_area edges
//    - Compute y intersections with each expanded_obstacle
//    - Sort by y, even-odd-fill pair the resulting endpoints
//    - For each pair (y_lo, y_hi): apply y_inset, alternate direction by
//      column index (boustrophedon), or invoke narrow-strategy callback
//
// Output is a vector of CoverageWaypoint pairs (start + end of each swath)
// in MAP frame, segment_type=SEGMENT_MOWING_BOUSTROPHEDON, blade_enabled=true.

#include <cstdint>
#include <functional>
#include <vector>

#include <geometry_msgs/msg/polygon.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>

#include "mowgli_coverage_planner/narrow_area_strategy.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"

namespace mowgli_coverage_planner
{

/// Callback invoked when a swath is shorter than `robot.footprint.robot_length`.
/// The callback receives the would-be ScanSegment (in MAP frame) plus context
/// and returns the waypoints to emit instead (may be empty for SKIP).
using NarrowStrategyCallback = std::function<NarrowAreaResult(
    const ScanSegment& segment,
    const geometry_msgs::msg::Polygon& area,
    const std::vector<geometry_msgs::msg::Polygon>& obstacles,
    const RobotGeometry& robot,
    double mow_angle_rad)>;

struct SweepResult
{
  std::vector<mowgli_interfaces::msg::CoverageWaypoint> waypoints;
  std::vector<std::string> warnings;  ///< One per narrow strip handled.
};

/// AABB sweep at `mow_angle_rad` over `area` minus `obstacles`. The narrow
/// strategy callback handles segments shorter than robot_length; if not
/// supplied, narrow segments are dropped silently (used by tests).
SweepResult sweep(const geometry_msgs::msg::Polygon& area,
                  const std::vector<geometry_msgs::msg::Polygon>& obstacles,
                  double mow_angle_rad, const RobotGeometry& robot,
                  double mowing_speed,
                  const NarrowStrategyCallback& narrow_cb = {});

}  // namespace mowgli_coverage_planner
