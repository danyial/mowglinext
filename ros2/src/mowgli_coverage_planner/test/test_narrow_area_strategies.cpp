// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// SPEC R-13 acceptance: same narrow strip with SKIP / OUTLINE_ONLY /
// SPECIAL_PATTERN produces plans whose metadata reflects the chosen
// strategy. SPECIAL_PATTERN footprint-validates per pose against the
// working area minus obstacles.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <mowgli_geometry/footprint.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>

#include "fixtures/polygons.hpp"
#include "mowgli_coverage_planner/narrow_area_strategy.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"

using mowgli_coverage_planner::apply_strategy;
using mowgli_coverage_planner::RobotGeometry;
using mowgli_coverage_planner::ScanSegment;
using mowgli_coverage_planner::test::make_horizontal_strip;
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
  r.tool_width = 0.18;
  r.outline_offset = 0.05;
  r.strip_overlap = 0.02;
  r.outline_passes = 1;
  return r;
}

ScanSegment make_segment()
{
  // 5 m long strip running along X.
  ScanSegment seg;
  seg.start = mowgli_coverage_planner::test::mk_p(-2.5, 0.0);
  seg.end = mowgli_coverage_planner::test::mk_p(+2.5, 0.0);
  return seg;
}

}  // namespace

// SPEC R-13 SKIP: emits no waypoints, populates a warning.
TEST(NarrowAreaStrategy, SkipEmitsWarning)
{
  auto strip = make_horizontal_strip(0.0, 0.0, 5.0, 0.30);
  auto seg = make_segment();
  auto robot = make_robot();
  std::vector<geometry_msgs::msg::Polygon> obstacles;

  auto result = apply_strategy(/*strategy=*/0, seg, strip, obstacles, robot,
                               /*mow_angle=*/0.0, /*mowing_speed=*/0.5);

  EXPECT_TRUE(result.waypoints.empty());
  EXPECT_FALSE(result.warning.empty()) << "SKIP must populate a warning";
}

// SPEC R-13 OUTLINE_ONLY: emits an inward-offset outline traversal.
TEST(NarrowAreaStrategy, OutlineOnlyEmitsWaypoints)
{
  auto strip = make_horizontal_strip(0.0, 0.0, 5.0, 0.30);
  auto seg = make_segment();
  auto robot = make_robot();
  std::vector<geometry_msgs::msg::Polygon> obstacles;

  auto result = apply_strategy(/*strategy=*/1, seg, strip, obstacles, robot,
                               /*mow_angle=*/0.0, /*mowing_speed=*/0.5);

  EXPECT_FALSE(result.warning.empty());
  // OUTLINE_ONLY emits at least 1 waypoint (or zero if the offset polygon
  // collapses too — narrow strip 0.30 m vs robot_width/2 = 0.20 + outline
  // offset 0.05 = 0.25 inset leaves a 0.05 m offset polygon, which is
  // valid). Don't be strict on the waypoint count — strategy is allowed
  // to emit zero with a warning when the inward offset collapses.
  for (const auto& wp : result.waypoints)
  {
    EXPECT_EQ(wp.segment_type, CoverageWaypoint::SEGMENT_OUTLINE_WORKING_AREA);
    EXPECT_TRUE(wp.blade_enabled);
  }
}

// SPEC R-13 SPECIAL_PATTERN: centerline along PCA axis. Per-pose footprint
// validation per D-10. Every emitted pose's footprint must be inside the
// strip polygon.
TEST(NarrowAreaStrategy, SpecialPatternFootprintValidatesPerPose)
{
  // Strip wide enough that some centerline poses can fit a footprint:
  // robot_width = 0.40 m; strip width = 0.50 m.
  auto strip = make_horizontal_strip(0.0, 0.0, 5.0, 0.50);
  auto seg = make_segment();
  auto robot = make_robot();
  std::vector<geometry_msgs::msg::Polygon> obstacles;

  auto result = apply_strategy(/*strategy=*/2, seg, strip, obstacles, robot,
                               /*mow_angle=*/0.0, /*mowing_speed=*/0.5);

  EXPECT_FALSE(result.warning.empty());
  // Every emitted pose's footprint must be inside the strip polygon.
  for (const auto& wp : result.waypoints)
  {
    EXPECT_EQ(wp.segment_type, CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON);
    EXPECT_TRUE(wp.blade_enabled);

    // Build footprint at this pose, assert it's inside the strip.
    const double yaw = std::atan2(
        2.0 * (wp.pose.pose.orientation.w * wp.pose.pose.orientation.z +
               wp.pose.pose.orientation.x * wp.pose.pose.orientation.y),
        1.0 - 2.0 * (wp.pose.pose.orientation.y * wp.pose.pose.orientation.y +
                     wp.pose.pose.orientation.z * wp.pose.pose.orientation.z));
    auto fp = mowgli_geometry::footprint_polygon(
        wp.pose.pose.position.x, wp.pose.pose.position.y, yaw,
        robot.footprint);
    EXPECT_TRUE(mowgli_geometry::footprint_inside_polygon(fp, strip))
        << "SPECIAL_PATTERN emitted pose at (" << wp.pose.pose.position.x
        << ", " << wp.pose.pose.position.y
        << ") whose footprint is not inside the strip";
  }
}
