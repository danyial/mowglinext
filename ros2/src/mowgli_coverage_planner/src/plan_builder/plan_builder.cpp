// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/plan_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <mowgli_geometry/geometry.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <mowgli_interfaces/msg/plan_error.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "mowgli_coverage_planner/boustrophedon_sweeper.hpp"
#include "mowgli_coverage_planner/checkpoint_io.hpp"
#include "mowgli_coverage_planner/narrow_area_strategy.hpp"
#include "mowgli_coverage_planner/outline_generator.hpp"

namespace mowgli_coverage_planner
{

namespace
{

using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;
using PlanError = mowgli_interfaces::msg::PlanError;

constexpr double kMowingSpeed = 0.5;
constexpr double kTransitSpeed = 0.5;
constexpr double kUndockDistance = 1.5;
constexpr double kDockApproachDistance = 1.0;
constexpr double kUndockSpeed = 0.15;

double normalize_angle_180_deg(double a)
{
  a = std::fmod(a, 180.0);
  if (a < 0.0) a += 180.0;
  return a;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

CoverageWaypoint mk_waypoint(double x, double y, double yaw,
                             std::uint8_t seg_type, bool blade, double speed,
                             std::uint32_t seq)
{
  CoverageWaypoint wp;
  wp.pose.header.frame_id = "map";
  wp.pose.pose.position.x = x;
  wp.pose.pose.position.y = y;
  wp.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  wp.pose.pose.orientation.x = q.x();
  wp.pose.pose.orientation.y = q.y();
  wp.pose.pose.orientation.z = q.z();
  wp.pose.pose.orientation.w = q.w();

  wp.sequence_id = seq;
  wp.speed = static_cast<float>(speed);
  wp.blade_enabled = blade;
  wp.segment_type = seg_type;
  return wp;
}

}  // namespace

// ---------------------------------------------------------------------------
// derive_mow_angle (free function — testable directly)
// ---------------------------------------------------------------------------

double derive_mow_angle(const PlanContext& ctx, std::uint32_t area_index)
{
  // 1. If goal supplies a non-sentinel angle, use it directly.
  const double goal_angle = static_cast<double>(ctx.goal.mow_angle_offset_deg);
  if (goal_angle >= 0.0)
  {
    return normalize_angle_180_deg(goal_angle);
  }

  // 2. Sentinel = -1 (auto-rotate). Always look up the persisted
  //    last_mow_angle_deg — the auto-rotate semantics are independent of
  //    the resume_from_checkpoint flag (resume = continue at swath endpoint,
  //    auto-rotate = next angle for the full new plan).
  auto loaded = read_checkpoint_file(ctx.areas_dir, area_index);
  if (loaded.has_value() && std::isfinite(loaded->last_mow_angle_deg))
  {
    const double next =
        loaded->last_mow_angle_deg + ctx.robot.angle_increment_deg;
    return normalize_angle_180_deg(next);
  }

  // 3. First run: MBR seed via mowgli_geometry::compute_optimal_mow_angle.
  if (area_index < ctx.areas.size())
  {
    const double mbr_rad =
        mowgli_geometry::compute_optimal_mow_angle(ctx.areas[area_index].area);
    return normalize_angle_180_deg(mbr_rad * 180.0 / M_PI);
  }

  return 0.0;
}

// ---------------------------------------------------------------------------
// PlanBuilder
// ---------------------------------------------------------------------------

PlanBuilder::PlanBuilder(const RobotGeometry& robot, std::string areas_dir)
    : robot_(robot), areas_dir_(std::move(areas_dir))
{
}

bool PlanBuilder::build(PlanContext& ctx)
{
  std::uint32_t seq = 0;

  // Whether the start_pose is implicit (start from dock) or explicit.
  const bool start_from_dock = ctx.goal.start_pose.header.frame_id.empty();

  const double dock_x = ctx.goal.dock_pose.pose.position.x;
  const double dock_y = ctx.goal.dock_pose.pose.position.y;
  const double dock_yaw = yaw_from_quaternion(ctx.goal.dock_pose.pose.orientation);

  // 1. UNDOCK: 2 waypoints (at dock + dock-frame-forward by undock_distance).
  if (start_from_dock)
  {
    ctx.plan.push_back(
        mk_waypoint(dock_x, dock_y, dock_yaw,
                    CoverageWaypoint::SEGMENT_UNDOCK,
                    /*blade=*/false, kUndockSpeed, seq++));
    // Forward in the dock's body frame is +X, so subtract for "back away".
    const double back_x = dock_x - kUndockDistance * std::cos(dock_yaw);
    const double back_y = dock_y - kUndockDistance * std::sin(dock_yaw);
    ctx.plan.push_back(
        mk_waypoint(back_x, back_y, dock_yaw,
                    CoverageWaypoint::SEGMENT_UNDOCK,
                    /*blade=*/false, kUndockSpeed, seq++));
  }
  else
  {
    // Start at the supplied pose. Emit a TRANSIT placeholder so the BT has a
    // start anchor.
    const double sx = ctx.goal.start_pose.pose.position.x;
    const double sy = ctx.goal.start_pose.pose.position.y;
    const double syaw =
        yaw_from_quaternion(ctx.goal.start_pose.pose.orientation);
    ctx.plan.push_back(
        mk_waypoint(sx, sy, syaw, CoverageWaypoint::SEGMENT_TRANSIT,
                    /*blade=*/false, kTransitSpeed, seq++));
  }

  // 2. Iterate areas. Single-angle-per-plan policy: the first working area
  //    determines ctx.mow_angle_used_deg for the whole plan.
  bool first_working_area = true;
  for (std::size_t i = 0; i < ctx.areas.size(); ++i)
  {
    const auto& area = ctx.areas[i];
    const auto idx = static_cast<std::uint32_t>(i);

    if (area.is_navigation_area)
    {
      // Compute centroid as a waypoint to anchor the navigation area on
      // the BT side. Single TRANSIT waypoint, blade off.
      double cx = 0.0, cy = 0.0;
      for (const auto& p : area.area.points)
      {
        cx += static_cast<double>(p.x);
        cy += static_cast<double>(p.y);
      }
      if (!area.area.points.empty())
      {
        cx /= static_cast<double>(area.area.points.size());
        cy /= static_cast<double>(area.area.points.size());
      }
      // Use yaw from previous segment if any, else 0.
      const double yaw =
          ctx.plan.empty() ? 0.0
                           : yaw_from_quaternion(
                                 ctx.plan.back().pose.pose.orientation);
      ctx.plan.push_back(
          mk_waypoint(cx, cy, yaw, CoverageWaypoint::SEGMENT_TRANSIT,
                      /*blade=*/false, kTransitSpeed, seq++));
      ctx.processed_area_indices.push_back(idx);
      continue;
    }

    // Working area.
    double area_angle_deg = 0.0;
    if (first_working_area)
    {
      area_angle_deg = derive_mow_angle(ctx, idx);
      ctx.mow_angle_used_deg = area_angle_deg;
      first_working_area = false;
    }
    else
    {
      area_angle_deg = ctx.mow_angle_used_deg;
    }
    const double area_angle_rad = area_angle_deg * M_PI / 180.0;

    // Resume: load checkpoint to know which swath / outline to start from.
    std::optional<mowgli_interfaces::msg::Checkpoint> resume_ck;
    if (ctx.goal.resume_from_checkpoint)
    {
      resume_ck = read_checkpoint_file(areas_dir_, idx);
    }

    // 3. Outlines (working area outside-in).
    if (!resume_ck.has_value() ||
        resume_ck->current_outline_index < ctx.robot.outline_passes)
    {
      auto outlines =
          generate_working_area_outlines(area.area, ctx.robot, kMowingSpeed);
      if (!outlines.warning.empty()) ctx.warnings.push_back(outlines.warning);
      // On resume, skip outlines already done (current_outline_index counts
      // completed outline passes). The OutlineGenerator emits one waypoint
      // per polygon vertex per pass; we trim the front N passes by
      // approximating "passes = total_waypoints / outline_passes".
      std::size_t skip_n = 0;
      if (resume_ck.has_value() && ctx.robot.outline_passes > 0u)
      {
        const std::size_t per_pass =
            outlines.waypoints.size() / ctx.robot.outline_passes;
        skip_n = std::min<std::size_t>(
            outlines.waypoints.size(),
            per_pass * resume_ck->current_outline_index);
      }
      for (std::size_t k = skip_n; k < outlines.waypoints.size(); ++k)
      {
        outlines.waypoints[k].sequence_id = seq++;
        ctx.plan.push_back(std::move(outlines.waypoints[k]));
      }
    }

    // 4. Obstacle outlines (inside-out).
    for (const auto& obs : area.obstacles)
    {
      auto obs_outlines =
          generate_obstacle_outlines(obs, ctx.robot, kMowingSpeed);
      if (obs_outlines.failed_offset)
      {
        PlanError err;
        err.error_code = PlanError::ERROR_OBSTACLE_OFFSET_FAILED;
        err.failed_validation_point = 5;
        err.human_readable = obs_outlines.warning;
        err.affected_area_indices.push_back(idx);
        err.affected_polygons.push_back(obs);
        ctx.error = err;
        return false;
      }
      if (!obs_outlines.warning.empty())
        ctx.warnings.push_back(obs_outlines.warning);
      for (auto& wp : obs_outlines.waypoints)
      {
        wp.sequence_id = seq++;
        ctx.plan.push_back(std::move(wp));
      }
    }

    // 5. Boustrophedon sweep + narrow-area dispatch.
    auto narrow_cb = [&area, &ctx, idx](
                          const ScanSegment& seg,
                          const geometry_msgs::msg::Polygon& a,
                          const std::vector<geometry_msgs::msg::Polygon>& obs,
                          const RobotGeometry& r,
                          double angle) -> NarrowAreaResult {
      auto strat_result = apply_strategy(area.narrow_area_strategy, seg, a,
                                          obs, r, angle, kMowingSpeed);
      if (!strat_result.warning.empty())
      {
        // Tag with area index for operator-facing logs.
        strat_result.warning =
            "area " + std::to_string(idx) + ": " + strat_result.warning;
      }
      return strat_result;
    };

    auto sweep_result = sweep(area.area, area.obstacles, area_angle_rad,
                               ctx.robot, kMowingSpeed, narrow_cb);
    for (const auto& warn : sweep_result.warnings) ctx.warnings.push_back(warn);

    // Resume: skip swaths already mowed (current_swath_index counts the
    // next-open swath in plan-emission order). Each full swath emits 2
    // waypoints (start + end); narrow-strategy results may emit any number.
    // Approximation: skip the first 2 * current_swath_index MOWING_BOUSTROPHEDON
    // waypoints. Then snap the very first remaining MOWING_BOUSTROPHEDON
    // pose to the persisted last_swath_endpoint within the SPEC R-11 5cm/5°
    // tolerance.
    std::size_t mow_count = 0;
    bool first_mow_emitted = false;
    for (auto& wp : sweep_result.waypoints)
    {
      const bool is_mow =
          wp.segment_type == CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON;
      if (is_mow && resume_ck.has_value() &&
          mow_count < 2u * resume_ck->current_swath_index)
      {
        ++mow_count;
        continue;  // skip already-mowed swath waypoints
      }
      if (is_mow) ++mow_count;

      // Resume snap: rewrite the first emitted MOWING_BOUSTROPHEDON pose to
      // the persisted last_swath_endpoint so SPEC R-11's 5cm/5° tolerance
      // is satisfied by construction. This is the "continue at the open
      // swath endpoint" guarantee.
      if (is_mow && resume_ck.has_value() && !first_mow_emitted)
      {
        wp.pose.pose.position.x = resume_ck->last_swath_endpoint.position.x;
        wp.pose.pose.position.y = resume_ck->last_swath_endpoint.position.y;
        wp.pose.pose.position.z = resume_ck->last_swath_endpoint.position.z;
        wp.pose.pose.orientation = resume_ck->last_swath_endpoint.orientation;
        first_mow_emitted = true;
      }

      wp.sequence_id = seq++;
      ctx.plan.push_back(std::move(wp));
    }

    ctx.processed_area_indices.push_back(idx);
  }

  // 6. RETURN_TO_DOCK: TRANSIT to the dock_approach_distance staging point.
  const double approach_x = dock_x - kDockApproachDistance * std::cos(dock_yaw);
  const double approach_y = dock_y - kDockApproachDistance * std::sin(dock_yaw);
  ctx.plan.push_back(
      mk_waypoint(approach_x, approach_y, dock_yaw,
                  CoverageWaypoint::SEGMENT_RETURN_TO_DOCK,
                  /*blade=*/false, kTransitSpeed, seq++));

  // 7. DOCK_APPROACH (slow approach to the dock).
  ctx.plan.push_back(
      mk_waypoint(approach_x, approach_y, dock_yaw,
                  CoverageWaypoint::SEGMENT_DOCK_APPROACH,
                  /*blade=*/false, kUndockSpeed, seq++));

  // 8. DOCKING (final pose at the dock).
  ctx.plan.push_back(
      mk_waypoint(dock_x, dock_y, dock_yaw,
                  CoverageWaypoint::SEGMENT_DOCKING,
                  /*blade=*/false, /*speed=*/0.0, seq++));

  return true;
}

}  // namespace mowgli_coverage_planner
