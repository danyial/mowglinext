// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// SPEC R-8 acceptance: AABB sweep with obstacle clipping. Square area + circular
// obstacle -> consecutive scan-line directions alternate, no swath crosses the
// obstacle band, plan size respects AC-3 (50 <= plan.size() <= 200 for 500 m^2
// area + 0.13 m spacing).

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <mowgli_interfaces/msg/coverage_waypoint.hpp>

#include "fixtures/polygons.hpp"
#include "mowgli_coverage_planner/boustrophedon_sweeper.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"

using mowgli_coverage_planner::RobotGeometry;
using mowgli_coverage_planner::sweep;
using mowgli_coverage_planner::test::make_circle;
using mowgli_coverage_planner::test::make_square;
using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;

namespace
{

RobotGeometry make_robot()
{
  RobotGeometry r;
  r.footprint.robot_length = 0.60;
  r.footprint.robot_width = 0.40;
  r.footprint.drive_axis_x_offset = -0.20;
  r.footprint.drive_axis_y_offset = 0.0;
  // For SPEC AC-3 we want path_spacing = 0.13 m -> tool_width - strip_overlap = 0.13.
  r.tool_width = 0.15;
  r.strip_overlap = 0.02;
  r.outline_offset = 0.05;
  r.outline_passes = 1;
  return r;
}

}  // namespace

// SPEC R-8: square area, no obstacles, mow_angle=0 -> swaths run along Y.
// Consecutive swath direction alternates (boustrophedon).
TEST(AABBSweep, SquareAlternatesDirections)
{
  auto area = make_square(0.0, 0.0, 10.0);
  std::vector<geometry_msgs::msg::Polygon> obstacles;

  auto result = sweep(area, obstacles, /*mow_angle_rad=*/0.0, make_robot(), 0.5);

  ASSERT_GE(result.waypoints.size(), 4u);
  // Pairs are (start, end). With no obstacles, every other pair has its
  // y-coordinate ordering flipped.
  ASSERT_EQ(result.waypoints.size() % 2u, 0u)
      << "swath waypoints must come in start+end pairs";
  std::size_t pairs = result.waypoints.size() / 2u;
  ASSERT_GE(pairs, 2u);
  bool last_was_low_to_high = false;
  for (std::size_t k = 0; k < pairs; ++k)
  {
    const auto& a = result.waypoints[2 * k];
    const auto& b = result.waypoints[2 * k + 1];
    const bool low_to_high = a.pose.pose.position.y < b.pose.pose.position.y;
    if (k > 0)
    {
      EXPECT_NE(low_to_high, last_was_low_to_high)
          << "swath " << k << " direction did not alternate";
    }
    last_was_low_to_high = low_to_high;
  }
}

// SPEC R-8: square area + circular obstacle -> swaths still emitted; obstacle
// band is excluded by the per-scan-line clipping. Test asserts no swath
// passes through the obstacle interior.
TEST(AABBSweep, ObstacleBandIsClipped)
{
  auto area = make_square(0.0, 0.0, 10.0);
  std::vector<geometry_msgs::msg::Polygon> obstacles{
      make_circle(0.0, 0.0, 1.5, 32)};

  auto result = sweep(area, obstacles, /*mow_angle_rad=*/0.0, make_robot(), 0.5);

  ASSERT_GE(result.waypoints.size(), 2u);

  // No swath segment may have BOTH endpoints inside the obstacle disc, and
  // for swaths whose x lies inside the obstacle's x-range, the y range must
  // be clipped above OR below the obstacle.
  const double obs_r = 1.5;
  for (std::size_t i = 0; i + 1 < result.waypoints.size(); i += 2)
  {
    const auto& a = result.waypoints[i].pose.pose.position;
    const auto& b = result.waypoints[i + 1].pose.pose.position;
    // Same x for both endpoints (vertical scan line in mow-angle-0 case).
    EXPECT_NEAR(a.x, b.x, 1e-6);
    if (std::abs(a.x) < obs_r)
    {
      // Swath x is inside obstacle x-range. The (y_lo, y_hi) interval must
      // lie entirely above OR below the obstacle's circle band at this x.
      const double half_band =
          std::sqrt(std::max(0.0, obs_r * obs_r - a.x * a.x));
      const double y_lo = std::min(a.y, b.y);
      const double y_hi = std::max(a.y, b.y);
      const bool above = y_lo > +half_band - 1e-6;
      const bool below = y_hi < -half_band + 1e-6;
      EXPECT_TRUE(above || below)
          << "swath at x=" << a.x << " crosses obstacle band [" << -half_band
          << ", " << half_band << "]: y in [" << y_lo << ", " << y_hi << "]";
    }
  }
}

// SPEC AC-3 budget: 500 m^2 square (~22.36 m side) + path_spacing=0.13 ->
// 50 <= plan.size() <= 200. plan.size() = 2 * num_swaths.
TEST(AABBSweep, AC3PlanSizeBudget)
{
  // Square of area 500 m^2.
  const double side = std::sqrt(500.0);
  auto area = make_square(0.0, 0.0, side);
  std::vector<geometry_msgs::msg::Polygon> obstacles;

  auto result = sweep(area, obstacles, /*mow_angle_rad=*/0.0, make_robot(), 0.5);

  // SPEC AC-3 regression guard: sparse plan, NOT pixel-densified.
  EXPECT_GE(result.waypoints.size(), 50u);
  EXPECT_LE(result.waypoints.size(), 200u)
      << "plan.size() exceeds SPEC AC-3 ceiling of 200 for 500 m^2 + 0.13 m spacing";
}
