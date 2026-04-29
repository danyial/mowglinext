// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/plan_builder.hpp"

#include <utility>

#include <mowgli_interfaces/msg/plan_error.hpp>

// Plan 01-07 Task 1 stub. Real implementation lands in Task 2.
namespace mowgli_coverage_planner
{

PlanBuilder::PlanBuilder(const RobotGeometry& robot, std::string areas_dir)
    : robot_(robot), areas_dir_(std::move(areas_dir))
{
}

bool PlanBuilder::build(PlanContext& ctx)
{
  mowgli_interfaces::msg::PlanError err;
  err.error_code = mowgli_interfaces::msg::PlanError::ERROR_INTERNAL;
  err.human_readable =
      "PlanBuilder::build not yet implemented (Plan 01-07 Task 2)";
  ctx.error = err;
  return false;
}

double derive_mow_angle(const PlanContext& /*ctx*/,
                        std::uint32_t /*area_index*/)
{
  return 0.0;
}

}  // namespace mowgli_coverage_planner
