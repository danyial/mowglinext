// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/narrow_area_strategy.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <mowgli_geometry/footprint.hpp>
#include <mowgli_geometry/geometry.hpp>
#include <mowgli_geometry/pca.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace mowgli_coverage_planner
{

namespace
{

using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;

CoverageWaypoint mk_wp(double x, double y, double yaw, std::uint8_t seg_type,
                       bool blade, double mowing_speed, std::uint32_t seq)
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
  wp.speed = static_cast<float>(mowing_speed);
  wp.blade_enabled = blade;
  wp.segment_type = seg_type;
  return wp;
}

}  // namespace

NarrowAreaResult apply_strategy(
    std::uint8_t strategy_value, const ScanSegment& /*segment*/,
    const geometry_msgs::msg::Polygon& area,
    const std::vector<geometry_msgs::msg::Polygon>& obstacles,
    const RobotGeometry& robot, double /*mow_angle_rad*/, double mowing_speed)
{
  NarrowAreaResult result;

  switch (strategy_value)
  {
    case 0:  // SKIP
    {
      result.warning = "narrow strip skipped (SKIP strategy)";
      return result;
    }

    case 1:  // OUTLINE_ONLY
    {
      // Emit one inward-offset traversal of the area polygon at robot_width/2.
      // This fills the strip with a single outline pass.
      const double inset = robot.footprint.robot_width / 2.0;
      auto offset =
          mowgli_geometry::offset_polygon_inward(area.points, inset);
      if (offset.size() >= 3)
      {
        std::uint32_t seq = 0;
        for (std::size_t i = 0; i < offset.size(); ++i)
        {
          const std::size_t next = (i + 1) % offset.size();
          const double dx = static_cast<double>(offset[next].x) -
                            static_cast<double>(offset[i].x);
          const double dy = static_cast<double>(offset[next].y) -
                            static_cast<double>(offset[i].y);
          const double yaw =
              (std::abs(dx) > 1e-9 || std::abs(dy) > 1e-9)
                  ? std::atan2(dy, dx)
                  : 0.0;
          result.waypoints.push_back(mk_wp(
              offset[i].x, offset[i].y, yaw,
              CoverageWaypoint::SEGMENT_OUTLINE_WORKING_AREA,
              /*blade=*/true, mowing_speed, seq++));
        }
      }
      result.warning = "narrow strip handled via OUTLINE_ONLY strategy";
      return result;
    }

    case 2:  // SPECIAL_PATTERN
    {
      // PCA principal axis on the area polygon's vertices (D-10).
      // Centerline poses spaced at robot.footprint.robot_length intervals;
      // per-pose footprint validation against area minus obstacles.
      const double axis_angle = mowgli_geometry::pca_principal_axis(area.points);

      // Compute centroid of the area polygon.
      double cx = 0.0, cy = 0.0;
      const std::size_t n = area.points.size();
      if (n == 0)
      {
        result.warning =
            "narrow strip handled via SPECIAL_PATTERN (degenerate polygon)";
        return result;
      }
      for (const auto& p : area.points)
      {
        cx += static_cast<double>(p.x);
        cy += static_cast<double>(p.y);
      }
      cx /= static_cast<double>(n);
      cy /= static_cast<double>(n);

      // Project all vertices onto the axis through the centroid to find
      // the extent along the principal direction.
      const double ax = std::cos(axis_angle);
      const double ay = std::sin(axis_angle);
      double t_min = std::numeric_limits<double>::infinity();
      double t_max = -std::numeric_limits<double>::infinity();
      for (const auto& p : area.points)
      {
        const double rx = static_cast<double>(p.x) - cx;
        const double ry = static_cast<double>(p.y) - cy;
        const double t = rx * ax + ry * ay;
        t_min = std::min(t_min, t);
        t_max = std::max(t_max, t);
      }

      // Step poses along the axis at robot_length intervals. Skip poses
      // whose footprint does not fit inside area minus obstacles.
      const double step = robot.footprint.robot_length;
      if (step <= 0.0)
      {
        result.warning =
            "narrow strip handled via SPECIAL_PATTERN (invalid robot_length)";
        return result;
      }
      // Inset endpoints so the footprint clears the polygon end edges.
      const double t_first = t_min + step / 2.0;
      const double t_last = t_max - step / 2.0;

      std::uint32_t seq = 0;
      for (double t = t_first; t <= t_last + 1e-9; t += step)
      {
        const double px = cx + t * ax;
        const double py = cy + t * ay;
        auto fp = mowgli_geometry::footprint_polygon(px, py, axis_angle,
                                                      robot.footprint);
        if (!mowgli_geometry::footprint_inside_polygon(fp, area)) continue;
        if (!mowgli_geometry::footprint_disjoint_obstacles(fp, obstacles))
          continue;

        result.waypoints.push_back(
            mk_wp(px, py, axis_angle,
                  CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON,
                  /*blade=*/true, mowing_speed, seq++));
      }
      result.warning = "narrow strip handled via SPECIAL_PATTERN strategy";
      return result;
    }

    default:
    {
      result.warning = "unknown narrow_area_strategy value; defaulting to SKIP";
      return result;
    }
  }
}

}  // namespace mowgli_coverage_planner
