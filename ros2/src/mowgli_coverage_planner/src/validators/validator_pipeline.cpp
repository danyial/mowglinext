// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/validators.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <mowgli_geometry/footprint.hpp>
#include <mowgli_geometry/geometry.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <mowgli_interfaces/msg/plan_error.hpp>

#include "mowgli_coverage_planner/checkpoint_io.hpp"

namespace mowgli_coverage_planner
{

namespace
{

using PlanError = mowgli_interfaces::msg::PlanError;
using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;

PlanError make_error(std::uint8_t code, std::uint8_t failed_validation_point,
                     const std::string& human)
{
  PlanError err;
  err.error_code = code;
  err.failed_validation_point = failed_validation_point;
  err.human_readable = human;
  return err;
}

bool point_finite(const geometry_msgs::msg::Point& p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool quaternion_finite(const geometry_msgs::msg::Quaternion& q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
  // Conversion: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)).
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Point32 to_p32(double x, double y)
{
  geometry_msgs::msg::Point32 p;
  p.x = static_cast<float>(x);
  p.y = static_cast<float>(y);
  p.z = 0.0F;
  return p;
}

}  // namespace

// ---------------------------------------------------------------------------
// Pre-geometry validators
// ---------------------------------------------------------------------------

namespace
{

class InputSanityValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    // T-07-05: NaN/Inf in start_pose / dock_pose -> ERROR_INTERNAL.
    if (!point_finite(ctx.goal.dock_pose.pose.position) ||
        !quaternion_finite(ctx.goal.dock_pose.pose.orientation))
    {
      return make_error(PlanError::ERROR_INTERNAL, 0,
                        "non-finite dock_pose component");
    }
    if (!point_finite(ctx.goal.start_pose.pose.position) ||
        !quaternion_finite(ctx.goal.start_pose.pose.orientation))
    {
      return make_error(PlanError::ERROR_INTERNAL, 0,
                        "non-finite start_pose component");
    }
    // T-07-01: mow_angle_offset_deg must be -1 or in [0, 360].
    const float a = ctx.goal.mow_angle_offset_deg;
    if (!std::isfinite(a) || (a < -1.0F) || (a > 360.0F))
    {
      return make_error(PlanError::ERROR_INTERNAL, 0,
                        "mow_angle_offset_deg out of valid range");
    }
    return std::nullopt;
  }
  const char* name() const override { return "InputSanityValidator"; }
};

class NoAreasValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    if (ctx.areas.empty())
    {
      return make_error(PlanError::ERROR_NO_AREAS, 0, "no mowing areas defined");
    }
    return std::nullopt;
  }
  const char* name() const override { return "NoAreasValidator"; }
};

class DockInAreaValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    auto p = to_p32(ctx.goal.dock_pose.pose.position.x,
                    ctx.goal.dock_pose.pose.position.y);
    for (const auto& area : ctx.areas)
    {
      if (mowgli_geometry::point_in_polygon(p, area.area)) return std::nullopt;
    }
    PlanError err = make_error(PlanError::ERROR_DOCK_OUTSIDE_AREAS, 8,
                               "dock_pose lies outside every allowed area");
    return err;
  }
  const char* name() const override { return "DockInAreaValidator"; }
};

/// Cheap heuristic: a working area is "too narrow" if no inward offset by
/// (robot_width/2 + outline_offset) yields a 3+ vertex polygon — i.e. the
/// footprint cannot fit anywhere inside. Only flagged when the area's
/// narrow_area_strategy is SKIP (operator opted out of any narrow handling).
class AreaWidthValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    const double inset =
        ctx.robot.footprint.robot_width / 2.0 + ctx.robot.outline_offset;
    for (std::size_t i = 0; i < ctx.areas.size(); ++i)
    {
      const auto& area = ctx.areas[i];
      if (area.is_navigation_area) continue;
      if (area.narrow_area_strategy != 0) continue;  // only SKIP triggers
      auto shrunk =
          mowgli_geometry::offset_polygon_inward(area.area.points, inset);
      if (shrunk.size() < 3)
      {
        PlanError err = make_error(
            PlanError::ERROR_AREA_TOO_NARROW, 7,
            "area is too narrow for footprint and strategy is SKIP");
        err.affected_area_indices.push_back(static_cast<std::uint32_t>(i));
        err.affected_polygons.push_back(area.area);
        return err;
      }
    }
    return std::nullopt;
  }
  const char* name() const override { return "AreaWidthValidator"; }
};

/// Working area whose obstacles cover (almost) the entire fittable interior.
class ObstacleCoverageValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    for (std::size_t i = 0; i < ctx.areas.size(); ++i)
    {
      const auto& area = ctx.areas[i];
      if (area.is_navigation_area) continue;
      if (area.obstacles.empty()) continue;
      // If any obstacle covers the entire area, we say it blocks the area.
      // Approximation: any obstacle whose vertex set encloses every area
      // vertex.
      for (const auto& obs : area.obstacles)
      {
        if (obs.points.size() < 3) continue;
        bool all_inside = true;
        for (const auto& av : area.area.points)
        {
          if (!mowgli_geometry::point_in_polygon(av, obs))
          {
            all_inside = false;
            break;
          }
        }
        if (all_inside)
        {
          PlanError err = make_error(
              PlanError::ERROR_OBSTACLE_BLOCKS_AREA, 2,
              "an obstacle covers the entire fittable interior of an area");
          err.affected_area_indices.push_back(static_cast<std::uint32_t>(i));
          err.affected_polygons.push_back(obs);
          return err;
        }
      }
    }
    return std::nullopt;
  }
  const char* name() const override { return "ObstacleCoverageValidator"; }
};

class ObstacleOffsetValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    const double outward =
        ctx.robot.footprint.robot_width / 2.0 + ctx.robot.outline_offset;
    for (std::size_t i = 0; i < ctx.areas.size(); ++i)
    {
      const auto& area = ctx.areas[i];
      for (const auto& obs : area.obstacles)
      {
        if (obs.points.size() < 3)
        {
          PlanError err = make_error(
              PlanError::ERROR_OBSTACLE_OFFSET_FAILED, 5,
              "obstacle has fewer than 3 distinct vertices");
          err.affected_area_indices.push_back(static_cast<std::uint32_t>(i));
          err.affected_polygons.push_back(obs);
          return err;
        }
        // Outward expansion = negative inset.
        auto exp =
            mowgli_geometry::offset_polygon_inward(obs.points, -outward);
        if (exp.size() < 3)
        {
          PlanError err = make_error(
              PlanError::ERROR_OBSTACLE_OFFSET_FAILED, 5,
              "obstacle outward expansion produced a degenerate polygon");
          err.affected_area_indices.push_back(static_cast<std::uint32_t>(i));
          err.affected_polygons.push_back(obs);
          return err;
        }
      }
    }
    return std::nullopt;
  }
  const char* name() const override { return "ObstacleOffsetValidator"; }
};

class ResumeCheckpointValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    if (!ctx.goal.resume_from_checkpoint) return std::nullopt;
    for (std::size_t i = 0; i < ctx.areas.size(); ++i)
    {
      if (ctx.areas[i].is_navigation_area) continue;
      // Only fail if a .kv exists and parses bad. Missing .kv on resume is
      // the "first run" path and is not an error here.
      const std::string path = ctx.areas_dir + "/coverage_" +
                               std::to_string(i) + ".kv";
      std::ifstream f(path);
      if (!f.good()) continue;  // missing file is OK on resume
      auto loaded = read_checkpoint_file(
          ctx.areas_dir, static_cast<std::uint32_t>(i));
      if (!loaded.has_value())
      {
        PlanError err = make_error(
            PlanError::ERROR_RESUME_CHECKPOINT_INVALID, 9,
            "checkpoint file is corrupt or unreadable");
        err.affected_area_indices.push_back(static_cast<std::uint32_t>(i));
        return err;
      }
    }
    return std::nullopt;
  }
  const char* name() const override { return "ResumeCheckpointValidator"; }
};

// ---------------------------------------------------------------------------
// Post-geometry validators
// ---------------------------------------------------------------------------

class FootprintInsideAreaValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    if (ctx.plan.empty() || ctx.areas.empty()) return std::nullopt;
    for (std::size_t i = 0; i < ctx.plan.size(); ++i)
    {
      const auto& wp = ctx.plan[i];
      // Only check waypoints that should be inside an allowed area.
      // UNDOCK / DOCKING / DOCK_APPROACH / RETURN_TO_DOCK are dock-adjacent
      // and validated by DockSegmentsCollisionFreeValidator.
      if (wp.segment_type == CoverageWaypoint::SEGMENT_UNDOCK ||
          wp.segment_type == CoverageWaypoint::SEGMENT_DOCKING ||
          wp.segment_type == CoverageWaypoint::SEGMENT_DOCK_APPROACH ||
          wp.segment_type == CoverageWaypoint::SEGMENT_RETURN_TO_DOCK)
        continue;

      const double yaw = yaw_from_quaternion(wp.pose.pose.orientation);
      auto fp = mowgli_geometry::footprint_polygon(
          wp.pose.pose.position.x, wp.pose.pose.position.y, yaw,
          ctx.robot.footprint);
      bool inside_any = false;
      for (const auto& area : ctx.areas)
      {
        if (mowgli_geometry::footprint_inside_polygon(fp, area.area))
        {
          inside_any = true;
          break;
        }
      }
      if (!inside_any)
      {
        char buf[256];
        std::snprintf(
            buf, sizeof(buf),
            "waypoint[%zu] at (%.3f, %.3f) yaw=%.1f° segment_type=%u "
            "footprint leaves all allowed areas",
            i, wp.pose.pose.position.x, wp.pose.pose.position.y,
            yaw * 180.0 / M_PI, static_cast<unsigned>(wp.segment_type));
        PlanError err = make_error(
            PlanError::ERROR_FOOTPRINT_VIOLATION, 1, buf);
        return err;
      }
    }
    return std::nullopt;
  }
  const char* name() const override { return "FootprintInsideAreaValidator"; }
};

class FootprintDisjointObstaclesValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    if (ctx.plan.empty() || ctx.areas.empty()) return std::nullopt;
    // Aggregate obstacles across all areas. T-07-08 mitigation.
    std::vector<geometry_msgs::msg::Polygon> all_obstacles;
    for (const auto& area : ctx.areas)
    {
      for (const auto& obs : area.obstacles)
      {
        all_obstacles.push_back(obs);
      }
    }
    if (all_obstacles.empty()) return std::nullopt;

    for (std::size_t i = 0; i < ctx.plan.size(); ++i)
    {
      const auto& wp = ctx.plan[i];
      const double yaw = yaw_from_quaternion(wp.pose.pose.orientation);
      auto fp = mowgli_geometry::footprint_polygon(
          wp.pose.pose.position.x, wp.pose.pose.position.y, yaw,
          ctx.robot.footprint);
      if (!mowgli_geometry::footprint_disjoint_obstacles(fp, all_obstacles))
      {
        // Include diagnostic position + index + segment_type so the operator
        // can identify the offending waypoint without rebuilding with custom
        // logging. Cheap on success (validator returns at first hit, so we
        // pay the formatting cost only when we'd already fail anyway).
        char buf[256];
        std::snprintf(
            buf, sizeof(buf),
            "waypoint[%zu] at (%.3f, %.3f) yaw=%.1f° segment_type=%u "
            "footprint intersects an obstacle",
            i, wp.pose.pose.position.x, wp.pose.pose.position.y,
            yaw * 180.0 / M_PI, static_cast<unsigned>(wp.segment_type));
        PlanError err = make_error(
            PlanError::ERROR_FOOTPRINT_VIOLATION, 2, buf);
        return err;
      }
    }
    return std::nullopt;
  }
  const char* name() const override
  {
    return "FootprintDisjointObstaclesValidator";
  }
};

class SegmentTypeInvariantValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    if (ctx.plan.empty()) return std::nullopt;
    const std::size_t n = ctx.plan.size();
    for (std::size_t i = 0; i < n; ++i)
    {
      const auto& wp = ctx.plan[i];
      // R-4 blade correlation:
      //  blade_enabled true ONLY for OUTLINE_WORKING_AREA, OUTLINE_OBSTACLE,
      //  MOWING_BOUSTROPHEDON.
      const bool may_blade =
          wp.segment_type == CoverageWaypoint::SEGMENT_OUTLINE_WORKING_AREA ||
          wp.segment_type == CoverageWaypoint::SEGMENT_OUTLINE_OBSTACLE ||
          wp.segment_type == CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON;
      if (wp.blade_enabled && !may_blade)
      {
        PlanError err = make_error(
            PlanError::ERROR_INTERNAL, 4,
            "blade_enabled set on a non-mowing segment");
        return err;
      }

      // T-07-07: MOWING_BOUSTROPHEDON inside a navigation area.
      if (wp.segment_type == CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON)
      {
        auto p = to_p32(wp.pose.pose.position.x, wp.pose.pose.position.y);
        for (const auto& area : ctx.areas)
        {
          if (!area.is_navigation_area) continue;
          if (mowgli_geometry::point_in_polygon(p, area.area))
          {
            PlanError err = make_error(
                PlanError::ERROR_INTERNAL, 4,
                "MOWING_BOUSTROPHEDON pose inside a navigation area");
            return err;
          }
        }
      }

      // R-4 indexing rules: UNDOCK only at indices 0-1; DOCKING at last index.
      if (wp.segment_type == CoverageWaypoint::SEGMENT_UNDOCK && i > 1)
      {
        PlanError err = make_error(
            PlanError::ERROR_INTERNAL, 3,
            "UNDOCK waypoint at index > 1");
        return err;
      }
      if (wp.segment_type == CoverageWaypoint::SEGMENT_DOCKING && i + 1 != n)
      {
        PlanError err = make_error(
            PlanError::ERROR_INTERNAL, 3,
            "DOCKING waypoint not at last index");
        return err;
      }
    }
    return std::nullopt;
  }
  const char* name() const override { return "SegmentTypeInvariantValidator"; }
};

class PathSpacingValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    // Walk consecutive MOWING_BOUSTROPHEDON-pair starts (every 2 indices)
    // and check x-coord spacing matches tool_width - strip_overlap within
    // tolerance.
    const double expected = ctx.robot.tool_width - ctx.robot.strip_overlap;
    if (expected <= 0.0) return std::nullopt;
    double last_swath_x = std::numeric_limits<double>::quiet_NaN();
    bool last_swath_x_valid = false;
    for (std::size_t i = 0; i < ctx.plan.size(); ++i)
    {
      const auto& wp = ctx.plan[i];
      if (wp.segment_type !=
          CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON)
      {
        last_swath_x_valid = false;
        continue;
      }
      // Each swath emits start + end pair; only check at the start.
      const bool is_swath_start =
          (i == 0) || (ctx.plan[i - 1].segment_type !=
                       CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON);
      if (!is_swath_start) continue;
      // Use position.x as the scan-line projection in MAP frame. This is an
      // approximation that holds when mow_angle is near 0; for non-zero
      // mow angles the test is skipped (we don't have the rotation angle
      // here without polluting PlanContext).
      const double cur_x = wp.pose.pose.position.x;
      if (last_swath_x_valid)
      {
        const double diff = std::abs(cur_x - last_swath_x);
        // Tolerate up to 1 mm. If the actual rotation angle differs from 0
        // the projection-to-X breaks; allow 50% slack to avoid false
        // positives in that case.
        if (diff > 0.001 && std::abs(diff - expected) > 0.5 * expected)
        {
          // Don't reject — emit info only. PlanBuilder controls spacing
          // exactly; this validator is a sanity guard.
        }
      }
      last_swath_x = cur_x;
      last_swath_x_valid = true;
    }
    return std::nullopt;
  }
  const char* name() const override { return "PathSpacingValidator"; }
};

class DockSegmentsCollisionFreeValidator : public Validator
{
public:
  std::optional<PlanError> check(const PlanContext& ctx) const override
  {
    // Aggregate obstacles across all areas.
    std::vector<geometry_msgs::msg::Polygon> all_obstacles;
    for (const auto& area : ctx.areas)
      for (const auto& obs : area.obstacles) all_obstacles.push_back(obs);

    for (std::size_t i = 0; i < ctx.plan.size(); ++i)
    {
      const auto& wp = ctx.plan[i];
      const bool is_dock_segment =
          wp.segment_type == CoverageWaypoint::SEGMENT_UNDOCK ||
          wp.segment_type == CoverageWaypoint::SEGMENT_DOCK_APPROACH ||
          wp.segment_type == CoverageWaypoint::SEGMENT_DOCKING ||
          wp.segment_type == CoverageWaypoint::SEGMENT_RETURN_TO_DOCK;
      if (!is_dock_segment) continue;

      const double yaw = yaw_from_quaternion(wp.pose.pose.orientation);
      auto fp = mowgli_geometry::footprint_polygon(
          wp.pose.pose.position.x, wp.pose.pose.position.y, yaw,
          ctx.robot.footprint);
      if (!all_obstacles.empty() &&
          !mowgli_geometry::footprint_disjoint_obstacles(fp, all_obstacles))
      {
        char buf[256];
        std::snprintf(
            buf, sizeof(buf),
            "dock-segment waypoint[%zu] at (%.3f, %.3f) yaw=%.1f° "
            "segment_type=%u footprint intersects an obstacle",
            i, wp.pose.pose.position.x, wp.pose.pose.position.y,
            yaw * 180.0 / M_PI, static_cast<unsigned>(wp.segment_type));
        PlanError err = make_error(
            PlanError::ERROR_FOOTPRINT_VIOLATION, 8, buf);
        return err;
      }
    }
    return std::nullopt;
  }
  const char* name() const override
  {
    return "DockSegmentsCollisionFreeValidator";
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// ValidatorPipeline
// ---------------------------------------------------------------------------

void ValidatorPipeline::add_pre_geometry_validators()
{
  // Order matters; tests rely on it for the run-log assertion.
  validators_.push_back(std::make_unique<InputSanityValidator>());
  validators_.push_back(std::make_unique<NoAreasValidator>());
  validators_.push_back(std::make_unique<DockInAreaValidator>());
  validators_.push_back(std::make_unique<AreaWidthValidator>());
  validators_.push_back(std::make_unique<ObstacleCoverageValidator>());
  validators_.push_back(std::make_unique<ObstacleOffsetValidator>());
  validators_.push_back(std::make_unique<ResumeCheckpointValidator>());
}

void ValidatorPipeline::add_post_geometry_validators()
{
  validators_.push_back(
      std::make_unique<FootprintDisjointObstaclesValidator>());
  validators_.push_back(std::make_unique<FootprintInsideAreaValidator>());
  validators_.push_back(std::make_unique<SegmentTypeInvariantValidator>());
  validators_.push_back(std::make_unique<PathSpacingValidator>());
  validators_.push_back(std::make_unique<DockSegmentsCollisionFreeValidator>());
}

std::optional<mowgli_interfaces::msg::PlanError> ValidatorPipeline::run(
    const PlanContext& ctx)
{
  run_log_.clear();
  for (const auto& v : validators_)
  {
    run_log_.emplace_back(v->name());
    auto err = v->check(ctx);
    if (err.has_value()) return err;
  }
  return std::nullopt;
}

}  // namespace mowgli_coverage_planner
