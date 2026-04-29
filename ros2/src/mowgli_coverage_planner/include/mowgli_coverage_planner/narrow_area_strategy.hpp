// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// SPEC R-13 narrow-area strategies (D-10):
//   SKIP            -> emit nothing; caller appends a warning
//   OUTLINE_ONLY    -> emit one extra inward-offset traversal that fills the strip
//   SPECIAL_PATTERN -> centerline along the strip's PCA principal axis
//                      (mowgli_geometry::pca_principal_axis), with
//                      footprint validation per pose
//
// All emitted waypoints are SEGMENT_MOWING_BOUSTROPHEDON with blade_enabled=true.
// The OUTLINE_ONLY variant emits SEGMENT_OUTLINE_WORKING_AREA waypoints since
// the operator's intent is "treat this strip as an outline-only pass".

#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>

#include "mowgli_coverage_planner/plan_context.hpp"

namespace mowgli_coverage_planner
{

/// One pair of scan-line endpoints (in MAP frame) representing the narrow
/// strip the strategy must handle. The two points are the would-be
/// MOWING_BOUSTROPHEDON start + end of the swath.
struct ScanSegment
{
  geometry_msgs::msg::Point32 start;
  geometry_msgs::msg::Point32 end;
};

/// Output of the strategy. `warning` is populated for SKIP / OUTLINE_ONLY /
/// SPECIAL_PATTERN with a human-readable description that gets appended
/// to PlanContext.warnings.
struct NarrowAreaResult
{
  std::vector<mowgli_interfaces::msg::CoverageWaypoint> waypoints;
  std::string warning;
};

/// Dispatch on the per-area `MapArea.narrow_area_strategy` enum (uint8).
///   strategy_value: 0=SKIP, 1=OUTLINE_ONLY, 2=SPECIAL_PATTERN
///   segment:        the narrow scan segment in MAP frame
///   area:           the working-area polygon (used for footprint validation)
///   obstacles:      obstacles in the area (used for footprint validation)
///   robot:          robot geometry (used for footprint construction + spacing)
///   mow_angle_rad:  the area's mow angle (used for OUTLINE_ONLY waypoint yaw)
///   mowing_speed:   speed written into every emitted waypoint
NarrowAreaResult apply_strategy(std::uint8_t strategy_value,
                                const ScanSegment& segment,
                                const geometry_msgs::msg::Polygon& area,
                                const std::vector<geometry_msgs::msg::Polygon>& obstacles,
                                const RobotGeometry& robot,
                                double mow_angle_rad,
                                double mowing_speed);

}  // namespace mowgli_coverage_planner
