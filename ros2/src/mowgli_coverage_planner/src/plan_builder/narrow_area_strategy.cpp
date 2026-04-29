// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/narrow_area_strategy.hpp"

// Plan 01-07 RED stub. The real implementation lands in the GREEN phase.
namespace mowgli_coverage_planner
{

NarrowAreaResult apply_strategy(
    std::uint8_t /*strategy_value*/, const ScanSegment& /*segment*/,
    const geometry_msgs::msg::Polygon& /*area*/,
    const std::vector<geometry_msgs::msg::Polygon>& /*obstacles*/,
    const RobotGeometry& /*robot*/, double /*mow_angle_rad*/,
    double /*mowing_speed*/)
{
  return {};
}

}  // namespace mowgli_coverage_planner
