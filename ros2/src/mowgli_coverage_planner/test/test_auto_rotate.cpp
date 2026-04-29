// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// SPEC R-9 acceptance: when goal.mow_angle_offset_deg = -1, the planner
// derives new_angle = (last_completed_angle + angle_increment) mod 180°.
// Three sequential plans should produce angles that differ by exactly
// angle_increment_deg mod 180°. First run with no checkpoint history uses
// the MBR seed (mowgli_geometry::compute_optimal_mow_angle).

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include <mowgli_interfaces/msg/checkpoint.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "fixtures/polygons.hpp"
#include "mowgli_coverage_planner/checkpoint_io.hpp"
#include "mowgli_coverage_planner/plan_builder.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"

using mowgli_coverage_planner::derive_mow_angle;
using mowgli_coverage_planner::PlanContext;
using mowgli_coverage_planner::RobotGeometry;
using mowgli_coverage_planner::test::make_square;
using mowgli_coverage_planner::write_checkpoint_file;
using Checkpoint = mowgli_interfaces::msg::Checkpoint;
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

PlanContext make_ctx(const std::string& areas_dir)
{
  PlanContext ctx;
  ctx.robot = make_robot();
  ctx.areas_dir = areas_dir;
  ctx.areas.push_back([] {
    mowgli_interfaces::msg::MapArea m;
    m.area = make_square(0.0, 0.0, 10.0);
    m.is_navigation_area = false;
    return m;
  }());
  ctx.goal.mow_angle_offset_deg = -1.0F;  // auto-rotate
  ctx.goal.resume_from_checkpoint = false;
  return ctx;
}

class AutoRotateFsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto base = fs::temp_directory_path();
    int counter = 0;
    while (true)
    {
      auto cand = base / ("mowgli_auto_rotate_" + std::to_string(::getpid()) +
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

void persist_angle(const std::string& areas_dir, std::uint32_t area_index,
                   double angle_deg)
{
  Checkpoint ck;
  ck.area_index = area_index;
  ck.current_outline_index = 0;
  ck.current_swath_index = 0;
  ck.swath_direction = Checkpoint::SWATH_DIRECTION_FORWARD;
  ck.last_completed_swath_index = 0;
  ck.next_open_swath_index = 0;
  ck.last_mow_angle_deg = angle_deg;
  ck.last_swath_endpoint.position.x = 0.0;
  ck.last_swath_endpoint.position.y = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  ck.last_swath_endpoint.orientation.x = q.x();
  ck.last_swath_endpoint.orientation.y = q.y();
  ck.last_swath_endpoint.orientation.z = q.z();
  ck.last_swath_endpoint.orientation.w = q.w();
  std::string err;
  ASSERT_TRUE(write_checkpoint_file(areas_dir, ck, &err)) << err;
}

double normalize_180(double a)
{
  // Map any angle in degrees to [0, 180).
  a = std::fmod(a, 180.0);
  if (a < 0.0) a += 180.0;
  return a;
}

}  // namespace

// SPEC R-9: three sequential plans with mow_angle_offset_deg=-1. Each plan
// reads the persisted last_mow_angle_deg, rotates by angle_increment_deg,
// and stores the new angle. Differences between consecutive runs must
// equal angle_increment_deg mod 180°.
TEST_F(AutoRotateFsTest, ThreeSequentialPlansRotateByIncrement)
{
  const double increment = 30.0;
  PlanContext ctx = make_ctx(tmpdir_);
  ctx.goal.resume_from_checkpoint = true;

  // Plan 1: no checkpoint exists -> MBR seed.
  double a1 = derive_mow_angle(ctx, /*area_index=*/0u);
  // Persist a checkpoint for the next call.
  persist_angle(tmpdir_, 0u, a1);

  // Plan 2: previous checkpoint exists -> a1 + increment.
  double a2 = derive_mow_angle(ctx, /*area_index=*/0u);
  EXPECT_NEAR(normalize_180(a2 - a1), increment, 1e-3)
      << "plan 2 should differ from plan 1 by exactly the increment";
  persist_angle(tmpdir_, 0u, a2);

  // Plan 3: another step.
  double a3 = derive_mow_angle(ctx, /*area_index=*/0u);
  EXPECT_NEAR(normalize_180(a3 - a2), increment, 1e-3)
      << "plan 3 should differ from plan 2 by exactly the increment";
  EXPECT_NEAR(normalize_180(a3 - a1), 2.0 * increment, 1e-3)
      << "plan 3 should differ from plan 1 by exactly 2 * increment";
}
