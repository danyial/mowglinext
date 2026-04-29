// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/validators.hpp"

// Plan 01-07 RED stub. The real implementation lands in the GREEN phase.
namespace mowgli_coverage_planner
{

void ValidatorPipeline::add_pre_geometry_validators() {}
void ValidatorPipeline::add_post_geometry_validators() {}

std::optional<mowgli_interfaces::msg::PlanError> ValidatorPipeline::run(
    const PlanContext& /*ctx*/)
{
  run_log_.clear();
  return std::nullopt;
}

}  // namespace mowgli_coverage_planner
