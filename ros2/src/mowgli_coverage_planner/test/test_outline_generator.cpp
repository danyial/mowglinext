// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// SPEC R-7 acceptance: working-area outlines outside-in + obstacle outlines
// inside-out, both footprint-aware (inset = robot_width/2 + outline_offset
// + p * step). All emitted waypoints carry blade_enabled=true and a
// vertex-direction yaw. Empty offset polygon (inset > polygon half-width)
// yields a warning, not a crash.

#include <algorithm>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <mowgli_interfaces/msg/coverage_waypoint.hpp>

#include "fixtures/polygons.hpp"
#include "mowgli_coverage_planner/outline_generator.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"

using mowgli_coverage_planner::generate_obstacle_outlines;
using mowgli_coverage_planner::generate_working_area_outlines;
using mowgli_coverage_planner::RobotGeometry;
using mowgli_coverage_planner::test::make_circle;
using mowgli_coverage_planner::test::make_square;
using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;

namespace
{

RobotGeometry make_robot(std::uint32_t outline_passes = 1)
{
  RobotGeometry r;
  r.footprint.robot_length = 0.60;
  r.footprint.robot_width = 0.40;
  r.footprint.drive_axis_x_offset = -0.20;
  r.footprint.drive_axis_y_offset = 0.0;
  r.tool_width = 0.18;
  r.outline_offset = 0.05;
  r.strip_overlap = 0.02;
  r.outline_passes = outline_passes;
  return r;
}

}  // namespace

// SPEC R-7: working-area outline. 10x10 square, 1 outline pass. Every emitted
// waypoint carries blade_enabled=true and segment_type=SEGMENT_OUTLINE_WORKING_AREA.
TEST(OutlineGenerator, WorkingAreaSquareSinglePass)
{
  auto poly = make_square(0.0, 0.0, 10.0);
  auto robot = make_robot(1);

  auto result = generate_working_area_outlines(poly, robot, /*mowing_speed=*/0.5);

  ASSERT_TRUE(result.warning.empty()) << result.warning;
  EXPECT_FALSE(result.failed_offset);
  // 4 distinct vertices at minimum (closing vertex deduped by offset_polygon_inward).
  ASSERT_GE(result.waypoints.size(), 4u);
  for (const auto& wp : result.waypoints)
  {
    EXPECT_EQ(wp.segment_type, CoverageWaypoint::SEGMENT_OUTLINE_WORKING_AREA);
    EXPECT_TRUE(wp.blade_enabled);
    EXPECT_EQ(wp.pose.header.frame_id, "map");
    EXPECT_FLOAT_EQ(wp.speed, 0.5F);
  }
}

// SPEC R-7: obstacle outline. Circle of radius 1.5 at (5, 5), 1 outline pass.
// Every emitted waypoint carries blade_enabled=true and segment_type=
// SEGMENT_OUTLINE_OBSTACLE.
TEST(OutlineGenerator, ObstacleCircleSinglePass)
{
  auto obs = make_circle(5.0, 5.0, 1.5, /*n_segments=*/16);
  auto robot = make_robot(1);

  auto result = generate_obstacle_outlines(obs, robot, /*mowing_speed=*/0.5);

  ASSERT_TRUE(result.warning.empty()) << result.warning;
  EXPECT_FALSE(result.failed_offset);
  ASSERT_GE(result.waypoints.size(), 16u);
  for (const auto& wp : result.waypoints)
  {
    EXPECT_EQ(wp.segment_type, CoverageWaypoint::SEGMENT_OUTLINE_OBSTACLE);
    EXPECT_TRUE(wp.blade_enabled);
  }
}

// Multi-pass outline: 2 passes -> exactly twice as many waypoints
// (each pass emits one waypoint per polygon vertex).
TEST(OutlineGenerator, WorkingAreaTwoPassesDoublesCount)
{
  auto poly = make_square(0.0, 0.0, 10.0);

  auto one = generate_working_area_outlines(poly, make_robot(1), 0.5);
  auto two = generate_working_area_outlines(poly, make_robot(2), 0.5);

  ASSERT_TRUE(one.warning.empty());
  ASSERT_TRUE(two.warning.empty());
  // Two passes emit 2x as many waypoints as one pass.
  EXPECT_EQ(two.waypoints.size(), 2 * one.waypoints.size());
}

// Operator preference: working-area outlines must be CCW (signed_area<0).
// Independent of the input polygon's winding order, the emitted waypoint
// sequence must satisfy the shoelace test for counter-clockwise traversal
// — that's how the blade auswurf consistently throws into the unmown
// interior. Tests both orientations of the input polygon to guarantee
// the normalisation, not just a happy-path passthrough.
TEST(OutlineGenerator, WorkingAreaOutlinesAreCCW)
{
  // Helper: signed area of a waypoint sequence treated as a closed loop.
  // Returns >0 for CW, <0 for CCW (matches outline_generator.cpp's convention).
  auto signed_area_of_waypoints =
      [](const std::vector<CoverageWaypoint>& wps) {
        double a = 0.0;
        const std::size_t n = wps.size();
        for (std::size_t i = 0; i < n; ++i)
        {
          const std::size_t j = (i + 1) % n;
          const double xi = wps[i].pose.pose.position.x;
          const double yi = wps[i].pose.pose.position.y;
          const double xj = wps[j].pose.pose.position.x;
          const double yj = wps[j].pose.pose.position.y;
          a += (xj - xi) * (yj + yi);
        }
        return a;
      };

  // make_square emits CCW vertices by construction. Verify the working-area
  // path stays CCW.
  {
    auto poly_ccw = make_square(0.0, 0.0, 10.0);
    auto result = generate_working_area_outlines(poly_ccw, make_robot(1), 0.5);
    ASSERT_TRUE(result.warning.empty()) << result.warning;
    ASSERT_GE(result.waypoints.size(), 4u);
    EXPECT_LT(signed_area_of_waypoints(result.waypoints), 0.0)
        << "CCW input should remain CCW (signed_area < 0)";
  }

  // Reverse the polygon → input is now CW. The generator must still emit
  // CCW waypoints (this is the actual normalisation under test).
  {
    auto poly_cw = make_square(0.0, 0.0, 10.0);
    std::reverse(poly_cw.points.begin(), poly_cw.points.end());
    auto result = generate_working_area_outlines(poly_cw, make_robot(1), 0.5);
    ASSERT_TRUE(result.warning.empty()) << result.warning;
    ASSERT_GE(result.waypoints.size(), 4u);
    EXPECT_LT(signed_area_of_waypoints(result.waypoints), 0.0)
        << "CW input must be normalised to CCW (signed_area < 0)";
  }
}

// Empty offset polygon -> warning, no crash.
// Inset much larger than polygon half-width collapses the offset polygon.
TEST(OutlineGenerator, EmptyOffsetEmitsWarning)
{
  auto poly = make_square(0.0, 0.0, 0.20);  // 20 cm side
  auto robot = make_robot(1);
  // robot_width/2 = 0.20, outline_offset = 0.05 -> total inset = 0.25.
  // The 20 cm polygon offset by 0.25 m collapses entirely.

  auto result = generate_working_area_outlines(poly, robot, 0.5);

  EXPECT_FALSE(result.warning.empty())
      << "expected a warning when offset polygon collapses";
  EXPECT_TRUE(result.waypoints.empty());
}
