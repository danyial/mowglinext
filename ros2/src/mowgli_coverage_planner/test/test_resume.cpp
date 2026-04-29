// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// SPEC R-11 acceptance: when goal.resume_from_checkpoint = true, the
// plan's first MOWING_BOUSTROPHEDON pose must lie within 5 cm + 5° of the
// persisted last_swath_endpoint, and the plan must NOT contain
// MOWING_BOUSTROPHEDON pairs covering the already-mowed swaths.

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include <mowgli_interfaces/msg/checkpoint.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "fixtures/polygons.hpp"
#include "mowgli_coverage_planner/checkpoint_io.hpp"
#include "mowgli_coverage_planner/plan_builder.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"

using mowgli_coverage_planner::PlanBuilder;
using mowgli_coverage_planner::PlanContext;
using mowgli_coverage_planner::RobotGeometry;
using mowgli_coverage_planner::test::make_square;
using mowgli_coverage_planner::write_checkpoint_file;
using Checkpoint = mowgli_interfaces::msg::Checkpoint;
using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;
namespace fs = std::filesystem;

namespace
{

constexpr double deg2rad(double d) { return d * M_PI / 180.0; }

RobotGeometry make_robot()
{
  RobotGeometry r;
  r.footprint.robot_length = 0.40;  // smaller so a 5x5 area fits >2 swaths
  r.footprint.robot_width = 0.40;
  r.footprint.drive_axis_x_offset = -0.10;
  r.footprint.drive_axis_y_offset = 0.0;
  r.tool_width = 0.50;
  r.outline_offset = 0.05;
  r.strip_overlap = 0.02;
  r.outline_passes = 1;
  r.angle_increment_deg = 30.0;
  return r;
}

PlanContext make_ctx(const std::string& areas_dir)
{
  PlanContext ctx;
  ctx.robot = make_robot();
  ctx.areas_dir = areas_dir;
  ctx.areas.push_back([] {
    mowgli_interfaces::msg::MapArea m;
    m.area = make_square(0.0, 0.0, 5.0);
    m.is_navigation_area = false;
    return m;
  }());
  ctx.goal.dock_pose.header.frame_id = "map";
  ctx.goal.dock_pose.pose.position.x = 0.0;
  ctx.goal.dock_pose.pose.position.y = 0.0;
  ctx.goal.dock_pose.pose.orientation.w = 1.0;
  ctx.goal.start_pose.pose.orientation.w = 1.0;
  ctx.goal.mow_angle_offset_deg = 0.0F;  // explicit angle for determinism
  return ctx;
}

class ResumeFsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto base = fs::temp_directory_path();
    int counter = 0;
    while (true)
    {
      auto cand = base / ("mowgli_resume_" + std::to_string(::getpid()) +
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

}  // namespace

// SPEC R-11: persist a checkpoint mid-swath, re-plan with
// resume_from_checkpoint=true, assert (a) first MOWING_BOUSTROPHEDON pose
// distance to persisted endpoint <= 0.05 m and yaw delta <= deg2rad(5),
// (b) ctx.mow_angle_used_deg matches persisted angle.
TEST_F(ResumeFsTest, ResumeWithinFiveCentimetresAndFiveDegrees)
{
  // Persist a checkpoint at (1.234, 0.567, yaw=pi/2) for area 0.
  Checkpoint ck;
  ck.area_index = 0u;
  ck.current_outline_index = 1u;
  ck.current_swath_index = 2u;
  ck.swath_direction = Checkpoint::SWATH_DIRECTION_FORWARD;
  ck.last_completed_swath_index = 1u;
  ck.next_open_swath_index = 2u;
  const double persisted_angle_deg = 0.0;
  ck.last_mow_angle_deg = persisted_angle_deg;
  ck.last_swath_endpoint.position.x = 1.234;
  ck.last_swath_endpoint.position.y = 0.567;
  ck.last_swath_endpoint.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI / 2.0);
  ck.last_swath_endpoint.orientation.x = q.x();
  ck.last_swath_endpoint.orientation.y = q.y();
  ck.last_swath_endpoint.orientation.z = q.z();
  ck.last_swath_endpoint.orientation.w = q.w();

  std::string werr;
  ASSERT_TRUE(write_checkpoint_file(tmpdir_, ck, &werr)) << werr;

  PlanContext ctx = make_ctx(tmpdir_);
  ctx.goal.resume_from_checkpoint = true;

  PlanBuilder builder(ctx.robot, tmpdir_);
  const bool ok = builder.build(ctx);
  ASSERT_TRUE(ok) << "PlanBuilder::build returned false on a valid resume input";

  // Find the first MOWING_BOUSTROPHEDON waypoint in the plan.
  const CoverageWaypoint* first_mow = nullptr;
  for (const auto& wp : ctx.plan)
  {
    if (wp.segment_type ==
        CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON)
    {
      first_mow = &wp;
      break;
    }
  }
  ASSERT_NE(first_mow, nullptr) << "plan contained no MOWING_BOUSTROPHEDON pose";

  // (a) Position tolerance <= 0.05 m.
  const double dx = first_mow->pose.pose.position.x - ck.last_swath_endpoint.position.x;
  const double dy = first_mow->pose.pose.position.y - ck.last_swath_endpoint.position.y;
  const double dist = std::hypot(dx, dy);
  EXPECT_LE(dist, 0.05)
      << "first MOWING_BOUSTROPHEDON pose distance to persisted endpoint exceeds 5 cm";

  // (b) Yaw tolerance <= deg2rad(5).
  const auto& wo = first_mow->pose.pose.orientation;
  const auto& po = ck.last_swath_endpoint.orientation;
  const double wo_yaw = std::atan2(2.0 * (wo.w * wo.z + wo.x * wo.y),
                                    1.0 - 2.0 * (wo.y * wo.y + wo.z * wo.z));
  const double po_yaw = std::atan2(2.0 * (po.w * po.z + po.x * po.y),
                                    1.0 - 2.0 * (po.y * po.y + po.z * po.z));
  double yaw_diff = std::fmod(std::abs(wo_yaw - po_yaw), 2.0 * M_PI);
  if (yaw_diff > M_PI) yaw_diff = 2.0 * M_PI - yaw_diff;
  EXPECT_LE(yaw_diff, deg2rad(5.0))
      << "first MOWING_BOUSTROPHEDON yaw delta exceeds 5°";

  // (c) ctx.mow_angle_used_deg matches the persisted angle.
  EXPECT_NEAR(ctx.mow_angle_used_deg, persisted_angle_deg, 1e-3)
      << "ctx.mow_angle_used_deg must equal persisted last_mow_angle_deg";
}
