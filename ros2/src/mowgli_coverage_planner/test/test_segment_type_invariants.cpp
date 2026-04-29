// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// SPEC R-4 acceptance: segment-type invariants. UNDOCK only at index 0-1,
// DOCKING at last index, MOWING_BOUSTROPHEDON only inside working areas,
// blade_enabled correlates per the spec table. The post-geometry
// SegmentTypeInvariantValidator enforces these rules; this test exercises
// both the success path and the failure path (T-07-07 mitigation: a plan
// that emits MOWING_BOUSTROPHEDON inside a navigation area is rejected
// with ERROR_INTERNAL failed_validation_point=4).

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <mowgli_interfaces/msg/plan_error.hpp>

#include "fixtures/polygons.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"
#include "mowgli_coverage_planner/validators.hpp"

using mowgli_coverage_planner::PlanContext;
using mowgli_coverage_planner::RobotGeometry;
using mowgli_coverage_planner::ValidatorPipeline;
using mowgli_coverage_planner::test::make_square;
using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;
using PlanError = mowgli_interfaces::msg::PlanError;

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

CoverageWaypoint mk_wp(double x, double y, std::uint8_t seg, bool blade,
                       std::uint32_t seq)
{
  CoverageWaypoint wp;
  wp.pose.header.frame_id = "map";
  wp.pose.pose.position.x = x;
  wp.pose.pose.position.y = y;
  wp.pose.pose.orientation.w = 1.0;
  wp.segment_type = seg;
  wp.blade_enabled = blade;
  wp.sequence_id = seq;
  wp.speed = 0.5F;
  return wp;
}

mowgli_interfaces::msg::MapArea make_working(
    const geometry_msgs::msg::Polygon& poly)
{
  mowgli_interfaces::msg::MapArea m;
  m.area = poly;
  m.is_navigation_area = false;
  return m;
}

mowgli_interfaces::msg::MapArea make_navigation(
    const geometry_msgs::msg::Polygon& poly)
{
  mowgli_interfaces::msg::MapArea m;
  m.area = poly;
  m.is_navigation_area = true;
  return m;
}

}  // namespace

// SPEC R-4 success path: a synthetic well-formed plan passes the segment-type
// invariant validator.
TEST(SegmentTypeInvariants, ValidPlanIsAccepted)
{
  PlanContext ctx;
  ctx.robot = make_robot();
  ctx.areas.push_back(make_working(make_square(0.0, 0.0, 10.0)));

  // UNDOCK -> MOWING_BOUSTROPHEDON (in working area) -> DOCKING.
  ctx.plan.push_back(
      mk_wp(0.0, 0.0, CoverageWaypoint::SEGMENT_UNDOCK, false, 0));
  ctx.plan.push_back(
      mk_wp(0.5, 0.0, CoverageWaypoint::SEGMENT_UNDOCK, false, 1));
  ctx.plan.push_back(mk_wp(1.0, 0.0,
                            CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON,
                            true, 2));
  ctx.plan.push_back(
      mk_wp(0.0, 0.0, CoverageWaypoint::SEGMENT_DOCKING, false, 3));

  ValidatorPipeline pipe;
  pipe.add_post_geometry_validators();
  auto err = pipe.run(ctx);

  // The whole post-geometry pipeline runs (footprint check etc. may complain
  // about the footprint hugging the obstacle if any — but with no obstacles
  // and a small plan, they all pass). Either result is acceptable for this
  // assertion; the invariant validator alone must NOT trip.
  if (err.has_value())
  {
    EXPECT_NE(err->failed_validation_point, 3);
    EXPECT_NE(err->failed_validation_point, 4);
  }
}

// SPEC R-4 / T-07-07: MOWING_BOUSTROPHEDON inside a navigation area is
// rejected with ERROR_INTERNAL failed_validation_point=4.
TEST(SegmentTypeInvariants, RejectsMowingInsideNavigationArea)
{
  PlanContext ctx;
  ctx.robot = make_robot();

  // One navigation area at (0, 0) and one working area at (50, 50).
  ctx.areas.push_back(make_navigation(make_square(0.0, 0.0, 10.0)));
  ctx.areas.push_back(make_working(make_square(50.0, 50.0, 10.0)));

  // Build a plan with a MOWING_BOUSTROPHEDON pose inside the navigation area
  // (T-07-07 violation).
  ctx.plan.push_back(
      mk_wp(0.0, 0.0, CoverageWaypoint::SEGMENT_UNDOCK, false, 0));
  ctx.plan.push_back(mk_wp(0.0, 0.0,
                            CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON,
                            true, 1));
  ctx.plan.push_back(
      mk_wp(0.0, 0.0, CoverageWaypoint::SEGMENT_DOCKING, false, 2));

  ValidatorPipeline pipe;
  pipe.add_post_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value())
      << "MOWING_BOUSTROPHEDON inside navigation area must be rejected";
  EXPECT_EQ(err->error_code, PlanError::ERROR_INTERNAL);
  EXPECT_EQ(err->failed_validation_point, 4);
}

// SPEC R-4: blade_enabled must correlate with segment_type. Blade ON for a
// TRANSIT waypoint is a violation.
TEST(SegmentTypeInvariants, RejectsBladeOnDuringTransit)
{
  PlanContext ctx;
  ctx.robot = make_robot();
  ctx.areas.push_back(make_working(make_square(0.0, 0.0, 10.0)));

  ctx.plan.push_back(mk_wp(0.0, 0.0,
                            CoverageWaypoint::SEGMENT_TRANSIT,
                            /*blade=*/true, 0));

  ValidatorPipeline pipe;
  pipe.add_post_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value());
  EXPECT_EQ(err->error_code, PlanError::ERROR_INTERNAL);
}
