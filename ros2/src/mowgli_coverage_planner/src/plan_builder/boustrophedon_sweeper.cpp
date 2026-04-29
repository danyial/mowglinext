// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/boustrophedon_sweeper.hpp"

// Plan 01-07 RED stub. The real implementation lands in the GREEN phase.
namespace mowgli_coverage_planner
{

SweepResult sweep(const geometry_msgs::msg::Polygon& /*area*/,
                  const std::vector<geometry_msgs::msg::Polygon>& /*obstacles*/,
                  double /*mow_angle_rad*/, const RobotGeometry& /*robot*/,
                  double /*mowing_speed*/,
                  const NarrowStrategyCallback& /*narrow_cb*/)
{
  return {};
}

}  // namespace mowgli_coverage_planner
