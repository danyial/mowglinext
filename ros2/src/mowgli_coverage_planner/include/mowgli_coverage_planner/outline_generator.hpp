// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// SPEC R-7 outline generator. Working-area outlines run outside-in
// (positive inward inset) and obstacle outlines run inside-out (negative
// inset, i.e. outward expansion). Both share the inset formula:
//   inset_p = robot_width/2 + outline_offset + p * (tool_width - strip_overlap)
// for p in 0..outline_passes-1.
//
// All emitted CoverageWaypoints carry blade_enabled=true; segment_type is
// SEGMENT_OUTLINE_WORKING_AREA for the area variant and SEGMENT_OUTLINE_OBSTACLE
// for the obstacle variant. Yaw at each vertex is derived from the
// direction of travel to the next vertex.

#include <string>
#include <vector>

#include <geometry_msgs/msg/polygon.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>

#include "mowgli_coverage_planner/plan_context.hpp"

namespace mowgli_coverage_planner
{

/// Result of one OutlineGenerator call. Empty `waypoints` plus a non-empty
/// `warning` indicates the offset polygon collapsed (e.g. inset > polygon
/// minimum half-width). `failed_offset` is true iff offset_polygon_inward
/// returned a degenerate result (caller maps to ERROR_OBSTACLE_OFFSET_FAILED
/// for obstacle outlines).
struct OutlineResult
{
  std::vector<mowgli_interfaces::msg::CoverageWaypoint> waypoints;
  std::string warning;        ///< Empty on full success.
  bool failed_offset{false};  ///< True iff offset returned a degenerate polygon.
};

/// Generate `outline_passes` working-area outlines, outside-in.
/// Each pass produces one waypoint per polygon vertex with
/// segment_type=SEGMENT_OUTLINE_WORKING_AREA and blade_enabled=true.
/// `mowing_speed` is written into every waypoint's speed field.
/// Outlines start with the smallest inset (closest to the polygon edge)
/// and progress inward.
OutlineResult generate_working_area_outlines(
    const geometry_msgs::msg::Polygon& area, const RobotGeometry& robot,
    double mowing_speed);

/// Generate `outline_passes` obstacle outlines, inside-out (outward
/// expansion). segment_type=SEGMENT_OUTLINE_OBSTACLE.
/// Returns failed_offset=true if any pass produces an empty polygon
/// (the caller treats this as ERROR_OBSTACLE_OFFSET_FAILED).
OutlineResult generate_obstacle_outlines(
    const geometry_msgs::msg::Polygon& obstacle, const RobotGeometry& robot,
    double mowing_speed);

}  // namespace mowgli_coverage_planner
