// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// Internal struct passed between phases of CoveragePlannerNode::execute().
// Plan 01-05 lands the skeleton fields (goal, areas, robot params, plan
// container, error / metadata accumulators). Plan 01-07 will extend this
// struct with intermediate per-area outline / sweep state used by the
// validator pipeline + plan builder.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <mowgli_geometry/footprint.hpp>
#include <mowgli_interfaces/action/plan_coverage.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <mowgli_interfaces/msg/map_area.hpp>
#include <mowgli_interfaces/msg/plan_error.hpp>

namespace mowgli_coverage_planner
{

/// Robot footprint + outline / sweep parameters consumed by the planner.
/// Wraps mowgli_geometry::FootprintParams (the chassis dimensions used by
/// the geometry helpers) plus the planner-specific knobs (outline_offset,
/// strip_overlap, outline_passes, angle_increment_deg) that map to the
/// `robot_geometry:` section of mowgli_robot.yaml + the planner overrides.
struct RobotGeometry
{
  mowgli_geometry::FootprintParams footprint;  ///< robot_length / robot_width / drive_axis_*
  double blade_x_offset{0.0};                  ///< blade-disc centre, X (m).
  double blade_y_offset{0.0};                  ///< blade-disc centre, Y (m).
  double tool_width{0.0};                      ///< effective blade cut width (m).
  double outline_offset{0.05};                 ///< gap polygon edge -> robot body edge (m).
  double strip_overlap{0.02};                  ///< path_spacing = tool_width - strip_overlap.
  std::uint32_t outline_passes{2};             ///< number of outline passes (>=1).
  double angle_increment_deg{30.0};            ///< auto-mow-angle increment per SPEC R-9.
};

/// Mutable per-goal state. Filled phase-by-phase in execute(). On failure,
/// the `error` field is populated and the action result is built from
/// `error` + the partial metadata. Plan 01-07 extends with intermediate
/// per-area state (rotated polygons, swath segments, narrow-area fallout).
struct PlanContext
{
  /// Snapshot of the action goal.
  mowgli_interfaces::action::PlanCoverage::Goal goal;

  /// Areas pulled from /map_server_node/get_all_areas at the start of execute().
  std::vector<mowgli_interfaces::msg::MapArea> areas;

  /// Robot footprint + planner knobs, copied from the node's parameters.
  RobotGeometry robot;

  /// Filesystem location of the per-area .kv checkpoints.
  std::string areas_dir;

  /// Sparse plan that the validator pipeline + plan builder will fill in.
  std::vector<mowgli_interfaces::msg::CoverageWaypoint> plan;

  /// Set by validators or the plan builder when a fatal failure occurs.
  std::optional<mowgli_interfaces::msg::PlanError> error;

  /// Per-area accounting consumed by PlanMetadata at result-build time.
  std::vector<std::uint32_t> processed_area_indices;
  std::vector<std::uint32_t> skipped_area_indices;
  std::vector<std::string> skip_reasons;
  std::vector<std::string> warnings;

  /// Mow-angle actually used for the plan (may differ from goal.mow_angle_offset_deg
  /// when -1 = auto-rotate or when narrow-area strategies override per-area).
  double mow_angle_used_deg{0.0};
};

}  // namespace mowgli_coverage_planner
