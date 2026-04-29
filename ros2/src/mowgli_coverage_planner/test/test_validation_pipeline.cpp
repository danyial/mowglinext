// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// SPEC R-12 acceptance: each of the 8 PlanError.error_code values has a
// triggering test, and on the success path all SPEC validation points are
// evaluated (assert via the test-only run-log hook).
//
// SPEC R-6 regression guard: a working area where the tool fits but the
// chassis overhang clips an obstacle is rejected with FOOTPRINT_VIOLATION.

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <mowgli_interfaces/msg/plan_error.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "fixtures/polygons.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"
#include "mowgli_coverage_planner/validators.hpp"

using mowgli_coverage_planner::PlanContext;
using mowgli_coverage_planner::RobotGeometry;
using mowgli_coverage_planner::ValidatorPipeline;
using mowgli_coverage_planner::test::make_circle;
using mowgli_coverage_planner::test::make_horizontal_strip;
using mowgli_coverage_planner::test::make_square;
using PlanError = mowgli_interfaces::msg::PlanError;
namespace fs = std::filesystem;

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
  r.angle_increment_deg = 30.0;
  return r;
}

PlanContext make_minimal_ctx(const std::string& areas_dir = "/tmp")
{
  PlanContext ctx;
  ctx.robot = make_robot();
  ctx.areas_dir = areas_dir;
  ctx.goal.dock_pose.header.frame_id = "map";
  ctx.goal.dock_pose.pose.position.x = 0.0;
  ctx.goal.dock_pose.pose.position.y = 0.0;
  ctx.goal.dock_pose.pose.orientation.w = 1.0;
  ctx.goal.start_pose.header.frame_id = "map";
  ctx.goal.start_pose.pose.orientation.w = 1.0;
  ctx.goal.mow_angle_offset_deg = 0.0F;
  ctx.goal.resume_from_checkpoint = false;
  return ctx;
}

mowgli_interfaces::msg::MapArea make_working_area(
    const geometry_msgs::msg::Polygon& poly)
{
  mowgli_interfaces::msg::MapArea m;
  m.name = "test";
  m.area = poly;
  m.is_navigation_area = false;
  m.narrow_area_strategy = 0;  // SKIP; relevant for AreaWidthValidator.
  return m;
}

}  // namespace

// ----- Pre-geometry error codes --------------------------------------------

// ERROR_NO_AREAS: empty areas list.
TEST(ValidationPipeline, RejectsEmptyAreaList)
{
  PlanContext ctx = make_minimal_ctx();
  ctx.areas.clear();

  ValidatorPipeline pipe;
  pipe.add_pre_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value());
  EXPECT_EQ(err->error_code, PlanError::ERROR_NO_AREAS);
}

// ERROR_DOCK_OUTSIDE_AREAS: dock pose far away from any allowed area.
TEST(ValidationPipeline, RejectsDockOutsideAreas)
{
  PlanContext ctx = make_minimal_ctx();
  ctx.areas.push_back(make_working_area(make_square(0.0, 0.0, 10.0)));
  ctx.goal.dock_pose.pose.position.x = 1000.0;
  ctx.goal.dock_pose.pose.position.y = 1000.0;

  ValidatorPipeline pipe;
  pipe.add_pre_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value());
  EXPECT_EQ(err->error_code, PlanError::ERROR_DOCK_OUTSIDE_AREAS);
}

// ERROR_AREA_TOO_NARROW: SKIP-strategy area where footprint never fits.
// 0.1 m wide strip is narrower than robot_width (0.40 m), so no orientation
// admits a footprint.
TEST(ValidationPipeline, RejectsAreaTooNarrowWhenSkip)
{
  PlanContext ctx = make_minimal_ctx();
  // 5 m long, 0.1 m wide strip.
  auto strip = make_horizontal_strip(0.0, 0.0, 5.0, 0.1);
  auto area = make_working_area(strip);
  area.narrow_area_strategy = 0;  // SKIP triggers AreaWidthValidator.
  ctx.areas.push_back(area);
  // Dock inside the strip so the dock-in-area validator passes.
  ctx.goal.dock_pose.pose.position.x = 0.0;
  ctx.goal.dock_pose.pose.position.y = 0.0;

  ValidatorPipeline pipe;
  pipe.add_pre_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value());
  EXPECT_EQ(err->error_code, PlanError::ERROR_AREA_TOO_NARROW);
}

// ERROR_OBSTACLE_BLOCKS_AREA: working area whose obstacles cover the entire
// fittable interior. Obstacle is slightly larger than the area so every
// area vertex is strictly inside the obstacle polygon (point_in_polygon's
// ray-cast is non-deterministic on edges).
TEST(ValidationPipeline, RejectsObstacleBlockingArea)
{
  PlanContext ctx = make_minimal_ctx();
  auto area = make_working_area(make_square(0.0, 0.0, 5.0));
  area.obstacles.push_back(make_square(0.0, 0.0, 6.0));
  ctx.areas.push_back(area);

  ValidatorPipeline pipe;
  pipe.add_pre_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value());
  EXPECT_EQ(err->error_code, PlanError::ERROR_OBSTACLE_BLOCKS_AREA);
}

// ERROR_OBSTACLE_OFFSET_FAILED: degenerate obstacle whose outward expansion
// produces an empty polygon (sub-2-vertex).
TEST(ValidationPipeline, RejectsDegenerateObstacleOffset)
{
  PlanContext ctx = make_minimal_ctx();
  auto area = make_working_area(make_square(0.0, 0.0, 10.0));
  // Degenerate obstacle: only 2 distinct points (collapses on offset).
  geometry_msgs::msg::Polygon bad_obs;
  bad_obs.points.push_back(mowgli_coverage_planner::test::mk_p(1.0, 1.0));
  bad_obs.points.push_back(mowgli_coverage_planner::test::mk_p(1.0, 1.0));
  bad_obs.points.push_back(mowgli_coverage_planner::test::mk_p(1.0, 1.0));
  area.obstacles.push_back(bad_obs);
  ctx.areas.push_back(area);

  ValidatorPipeline pipe;
  pipe.add_pre_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value());
  EXPECT_EQ(err->error_code, PlanError::ERROR_OBSTACLE_OFFSET_FAILED);
}

// ERROR_RESUME_CHECKPOINT_INVALID: corrupt .kv on disk while
// resume_from_checkpoint=true.
TEST(ValidationPipeline, RejectsCorruptCheckpointOnResume)
{
  // Per-test temp dir.
  const auto tmp = fs::temp_directory_path() / "mowgli_validate_resume_test";
  std::error_code ec;
  fs::create_directories(tmp, ec);
  // Write a garbage .kv for area_index 0.
  std::ofstream f(tmp / "coverage_0.kv");
  f << "this is not a valid checkpoint\n";
  f.close();

  PlanContext ctx = make_minimal_ctx(tmp.string());
  ctx.areas.push_back(make_working_area(make_square(0.0, 0.0, 10.0)));
  ctx.goal.resume_from_checkpoint = true;

  ValidatorPipeline pipe;
  pipe.add_pre_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value());
  EXPECT_EQ(err->error_code, PlanError::ERROR_RESUME_CHECKPOINT_INVALID);

  fs::remove_all(tmp, ec);
}

// ERROR_INTERNAL: out-of-range mow_angle_offset_deg=999.
TEST(ValidationPipeline, RejectsOutOfRangeMowAngleAsInternal)
{
  PlanContext ctx = make_minimal_ctx();
  ctx.areas.push_back(make_working_area(make_square(0.0, 0.0, 10.0)));
  ctx.goal.mow_angle_offset_deg = 999.0F;  // out of [-1, 360]

  ValidatorPipeline pipe;
  pipe.add_pre_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value());
  EXPECT_EQ(err->error_code, PlanError::ERROR_INTERNAL);
}

// ----- Post-geometry error codes (R-6 regression guard) --------------------

// ERROR_FOOTPRINT_VIOLATION (failed_validation_point=2):
// Working area where the tool fits but chassis overhang would clip an
// obstacle. R-6 regression guard: legacy tool-centric check would accept,
// the new footprint-aware check rejects. We construct a plan manually with
// a waypoint whose footprint overlaps an obstacle.
TEST(ValidationPipeline, RejectsChassisOverhangIntoObstacle_R6Regression)
{
  PlanContext ctx = make_minimal_ctx();
  auto area = make_working_area(make_square(0.0, 0.0, 10.0));
  // Obstacle close enough to the planned waypoint that the chassis overhang
  // (drive_axis_x_offset = -0.20, robot_length = 0.60 -> rear extends to
  // x = -0.10 from axle, front extends to x = +0.50 from axle) will clip it
  // even though the blade (assumed centered on axle plus blade_x_offset = 0.25)
  // would not. Place a 0.5 x 0.5 obstacle just behind the axle.
  area.obstacles.push_back(make_square(-0.30, 0.0, 0.40));
  ctx.areas.push_back(area);

  // Plan with a single MOWING_BOUSTROPHEDON pose at origin pointing +X.
  // Body footprint stretches from x=-0.10 to x=+0.50 around axle at (0, 0)
  // when yaw=0. The obstacle at x=-0.30..+0.10 overlaps the rear of the
  // chassis (chassis overhang case for R-6 regression guard / tool fits but
  // footprint violation).
  mowgli_interfaces::msg::CoverageWaypoint wp;
  wp.pose.header.frame_id = "map";
  wp.pose.pose.position.x = 0.0;
  wp.pose.pose.position.y = 0.0;
  wp.pose.pose.orientation.w = 1.0;
  wp.segment_type = mowgli_interfaces::msg::CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON;
  wp.blade_enabled = true;
  wp.speed = 0.5F;
  wp.sequence_id = 0;
  ctx.plan.push_back(wp);

  ValidatorPipeline pipe;
  pipe.add_post_geometry_validators();
  auto err = pipe.run(ctx);

  ASSERT_TRUE(err.has_value())
      << "post-geometry pipeline did not flag chassis-overhang obstacle clip "
         "(R-6 regression guard / tool fits but footprint violation)";
  EXPECT_EQ(err->error_code, PlanError::ERROR_FOOTPRINT_VIOLATION);
}

// ----- Success path: ALL pre-geometry validators run -----------------------

// On the success path the pipeline must evaluate every registered validator.
// run_log size == pipeline size on success.
TEST(ValidationPipeline, AllValidatorsRunOnSuccess)
{
  PlanContext ctx = make_minimal_ctx();
  ctx.areas.push_back(make_working_area(make_square(0.0, 0.0, 10.0)));

  ValidatorPipeline pipe;
  pipe.add_pre_geometry_validators();
  auto err = pipe.run(ctx);

  EXPECT_FALSE(err.has_value()) << "expected pre-geometry validators to pass";
  EXPECT_EQ(pipe.run_log_for_test_only().size(), pipe.size());
  EXPECT_GE(pipe.size(), 6u)
      << "pre-geometry pipeline should register at least 6 validators";
}
