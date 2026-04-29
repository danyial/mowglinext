// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// PlanBuilder composes the full coverage plan: UNDOCK -> per-area outlines
// + boustrophedon sweep + narrow-area strategies -> RETURN_TO_DOCK ->
// DOCK_APPROACH -> DOCKING. Auto-mow-angle rotation per SPEC R-9 and
// resume-from-checkpoint per SPEC R-11 happen here.
//
// Plan 01-07 Task 1 lands the header + a stub implementation that returns
// false (PlanContext.error gets ERROR_INTERNAL "not implemented"). Task 2
// replaces the stub with the real composition logic.

#include <string>

#include "mowgli_coverage_planner/plan_context.hpp"

namespace mowgli_coverage_planner
{

class PlanBuilder
{
public:
  PlanBuilder(const RobotGeometry& robot, std::string areas_dir);

  /// Returns true on success and fills ctx.plan + ctx.processed_area_indices.
  /// Returns false on failure with ctx.error populated.
  bool build(PlanContext& ctx);

private:
  RobotGeometry robot_;
  std::string areas_dir_;
};

/// Mow-angle derivation for one area. Extracted as a free function so tests
/// can exercise it without instantiating PlanBuilder.
///   - If ctx.goal.mow_angle_offset_deg != -1 -> use it.
///   - Else if resume_from_checkpoint and a checkpoint exists -> rotate
///     by ctx.robot.angle_increment_deg from last_mow_angle_deg.
///   - Else (first run) -> mowgli_geometry::compute_optimal_mow_angle (MBR).
/// Returned angle is in degrees, normalized to [0, 180).
double derive_mow_angle(const PlanContext& ctx, std::uint32_t area_index);

}  // namespace mowgli_coverage_planner
