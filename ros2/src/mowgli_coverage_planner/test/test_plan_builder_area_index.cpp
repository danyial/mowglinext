// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// Plan 01-10 Task 2 (RED/GREEN): Verify that PlanBuilder stamps area_index on
// every emitted CoverageWaypoint, and that UNDOCK/RETURN_TO_DOCK/DOCK_APPROACH/
// DOCKING segments carry UINT32_MAX (kNoArea) while working-area outlines and
// sweep waypoints carry the loop index of the owning area. Cements the R-9/R-11
// per-segment-type stamping contract.

#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <mowgli_interfaces/msg/map_area.hpp>

#include "fixtures/polygons.hpp"
#include "mowgli_coverage_planner/plan_builder.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"

using mowgli_coverage_planner::PlanBuilder;
using mowgli_coverage_planner::PlanContext;
using mowgli_coverage_planner::RobotGeometry;
using mowgli_coverage_planner::test::make_square;
using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;
namespace fs = std::filesystem;

namespace
{

/// Sentinel: waypoint does not belong to any working/navigation area.
constexpr std::uint32_t kNoArea = std::numeric_limits<std::uint32_t>::max();

// ─── Test fixture ────────────────────────────────────────────────────────────

class PlanBuilderAreaIndexTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto base = fs::temp_directory_path();
    int counter = 0;
    while (true)
    {
      auto cand = base / ("mowgli_area_idx_" + std::to_string(::getpid()) +
                          "_" + std::to_string(counter));
      std::error_code ec;
      if (fs::create_directory(cand, ec))
      {
        tmpdir_ = cand.string();
        break;
      }
      ++counter;
      ASSERT_LT(counter, 1024) << "could not create unique tmpdir";
    }
  }

  void TearDown() override
  {
    if (!tmpdir_.empty())
    {
      std::error_code ec;
      fs::remove_all(tmpdir_, ec);
    }
  }

  std::string tmpdir_;
};

// ─── Fixture factory helpers ──────────────────────────────────────────────────

/// Two non-overlapping 5 m² working areas with a dock far outside both.
PlanContext make_two_area_ctx(const std::string& areas_dir)
{
  PlanContext ctx;
  ctx.areas_dir = areas_dir;

  // Robot: rectangular 0.6 x 0.4, drive_axis at -0.20, tool_width 0.18.
  ctx.robot.footprint.robot_length = 0.60;
  ctx.robot.footprint.robot_width = 0.40;
  ctx.robot.footprint.drive_axis_x_offset = -0.20;
  ctx.robot.footprint.drive_axis_y_offset = 0.0;
  ctx.robot.tool_width = 0.18;
  ctx.robot.outline_offset = 0.05;
  ctx.robot.strip_overlap = 0.02;
  ctx.robot.outline_passes = 1u;
  ctx.robot.angle_increment_deg = 30.0;

  // Area 0: 5 m square centred at (0, 0).
  mowgli_interfaces::msg::MapArea a0;
  a0.is_navigation_area = false;
  a0.narrow_area_strategy = mowgli_interfaces::msg::MapArea::NARROW_AREA_SKIP;
  a0.area = make_square(0.0, 0.0, 5.0);
  ctx.areas.push_back(a0);

  // Area 1: 5 m square centred at (20, 0) — no overlap with area 0.
  mowgli_interfaces::msg::MapArea a1;
  a1.is_navigation_area = false;
  a1.narrow_area_strategy = mowgli_interfaces::msg::MapArea::NARROW_AREA_SKIP;
  a1.area = make_square(20.0, 0.0, 5.0);
  ctx.areas.push_back(a1);

  // Dock outside both areas.
  ctx.goal.dock_pose.header.frame_id = "map";
  ctx.goal.dock_pose.pose.position.x = -10.0;
  ctx.goal.dock_pose.pose.position.y = 0.0;
  ctx.goal.dock_pose.pose.orientation.w = 1.0;
  ctx.goal.start_pose.header.frame_id = "";  // empty -> start_from_dock
  ctx.goal.mow_angle_offset_deg = 0.0F;       // explicit angle for determinism
  ctx.goal.resume_from_checkpoint = false;
  return ctx;
}

}  // namespace

// ─── Test 1: Area waypoints carry the owning area's index ────────────────────

TEST_F(PlanBuilderAreaIndexTest, StampsAreaIndexOnAreaWaypoints)
{
  PlanContext ctx = make_two_area_ctx(tmpdir_);
  PlanBuilder builder(ctx.robot, tmpdir_);
  ASSERT_TRUE(builder.build(ctx));

  std::size_t mow_for_a0 = 0, mow_for_a1 = 0;
  for (const auto& wp : ctx.plan)
  {
    if (wp.segment_type == CoverageWaypoint::SEGMENT_OUTLINE_WORKING_AREA ||
        wp.segment_type == CoverageWaypoint::SEGMENT_OUTLINE_OBSTACLE ||
        wp.segment_type == CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON)
    {
      EXPECT_NE(wp.area_index, kNoArea)
          << "area waypoint at sequence " << wp.sequence_id
          << " segment_type=" << static_cast<int>(wp.segment_type)
          << " has UINT32_MAX area_index";
      EXPECT_LT(wp.area_index, 2u)
          << "area waypoint at sequence " << wp.sequence_id
          << " has out-of-range area_index=" << wp.area_index;
      if (wp.segment_type == CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON)
      {
        if (wp.area_index == 0u) ++mow_for_a0;
        else if (wp.area_index == 1u) ++mow_for_a1;
      }
    }
  }
  EXPECT_GT(mow_for_a0, 0u) << "no MOWING waypoints stamped for area 0";
  EXPECT_GT(mow_for_a1, 0u) << "no MOWING waypoints stamped for area 1";
}

// ─── Test 2: Dock/undock segments carry UINT32_MAX sentinel ──────────────────

TEST_F(PlanBuilderAreaIndexTest, StampsSentinelOnDockAndUndockSegments)
{
  PlanContext ctx = make_two_area_ctx(tmpdir_);
  PlanBuilder builder(ctx.robot, tmpdir_);
  ASSERT_TRUE(builder.build(ctx));

  bool saw_undock = false, saw_return = false, saw_approach = false,
       saw_docking = false;
  for (const auto& wp : ctx.plan)
  {
    switch (wp.segment_type)
    {
      case CoverageWaypoint::SEGMENT_UNDOCK:
        saw_undock = true;
        EXPECT_EQ(wp.area_index, kNoArea)
            << "UNDOCK waypoint at sequence " << wp.sequence_id
            << " should carry kNoArea";
        break;
      case CoverageWaypoint::SEGMENT_RETURN_TO_DOCK:
        saw_return = true;
        EXPECT_EQ(wp.area_index, kNoArea)
            << "RETURN_TO_DOCK waypoint at sequence " << wp.sequence_id
            << " should carry kNoArea";
        break;
      case CoverageWaypoint::SEGMENT_DOCK_APPROACH:
        saw_approach = true;
        EXPECT_EQ(wp.area_index, kNoArea)
            << "DOCK_APPROACH waypoint at sequence " << wp.sequence_id
            << " should carry kNoArea";
        break;
      case CoverageWaypoint::SEGMENT_DOCKING:
        saw_docking = true;
        EXPECT_EQ(wp.area_index, kNoArea)
            << "DOCKING waypoint at sequence " << wp.sequence_id
            << " should carry kNoArea";
        break;
      default:
        break;
    }
  }
  EXPECT_TRUE(saw_undock) << "plan has no UNDOCK waypoint";
  EXPECT_TRUE(saw_return) << "plan has no RETURN_TO_DOCK waypoint";
  EXPECT_TRUE(saw_approach) << "plan has no DOCK_APPROACH waypoint";
  EXPECT_TRUE(saw_docking) << "plan has no DOCKING waypoint";
}

// ─── Test 3: Navigation-area centroid TRANSIT carries the nav-area's index ───

TEST_F(PlanBuilderAreaIndexTest, NavigationAreaTransitWaypointsCarryNavigationAreaIndex)
{
  PlanContext ctx = make_two_area_ctx(tmpdir_);
  // Flip area 0 to navigation area; area 1 remains a working area.
  ctx.areas[0].is_navigation_area = true;

  PlanBuilder builder(ctx.robot, tmpdir_);
  ASSERT_TRUE(builder.build(ctx));

  // The navigation-area centroid is emitted as a TRANSIT waypoint. Its
  // area_index must be 0 (the navigation area's loop index), not kNoArea
  // and not 1 (the working area).
  bool found_nav_transit = false;
  for (const auto& wp : ctx.plan)
  {
    if (wp.segment_type == CoverageWaypoint::SEGMENT_TRANSIT &&
        wp.area_index == 0u)
    {
      found_nav_transit = true;
      break;
    }
  }
  EXPECT_TRUE(found_nav_transit)
      << "no TRANSIT waypoint with area_index=0 (navigation area) found in plan";
}
