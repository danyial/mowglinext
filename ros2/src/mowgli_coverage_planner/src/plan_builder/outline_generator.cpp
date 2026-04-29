// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/outline_generator.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <mowgli_geometry/geometry.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mowgli_coverage_planner
{

namespace
{

using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;

/// Signed-area shoelace sign for a polygon (>0: CW, <0: CCW, 0: degenerate).
double signed_area(const std::vector<geometry_msgs::msg::Point32>& pts)
{
  double a = 0.0;
  const std::size_t n = pts.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const std::size_t j = (i + 1) % n;
    a += static_cast<double>(pts[j].x - pts[i].x) *
         static_cast<double>(pts[j].y + pts[i].y);
  }
  return a;
}

/// Build one CoverageWaypoint at (x, y) with yaw, segment type, blade flag.
CoverageWaypoint mk_waypoint(double x, double y, double yaw,
                             std::uint8_t seg_type, bool blade,
                             double mowing_speed, std::uint32_t seq)
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

/// Convert a polygon traversal into per-vertex waypoints with direction-of-
/// travel yaw at each vertex. The polygon is treated as closed — yaw at the
/// last vertex points back to the first vertex.
void emit_outline_pass(
    const std::vector<geometry_msgs::msg::Point32>& vertices,
    std::uint8_t seg_type, double mowing_speed, std::uint32_t& seq,
    std::vector<CoverageWaypoint>& out)
{
  const std::size_t n = vertices.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const std::size_t next = (i + 1) % n;
    const double dx = static_cast<double>(vertices[next].x) -
                      static_cast<double>(vertices[i].x);
    const double dy = static_cast<double>(vertices[next].y) -
                      static_cast<double>(vertices[i].y);
    const double yaw =
        (std::abs(dx) > 1e-9 || std::abs(dy) > 1e-9) ? std::atan2(dy, dx) : 0.0;
    out.push_back(mk_waypoint(vertices[i].x, vertices[i].y, yaw, seg_type,
                              /*blade=*/true, mowing_speed, seq++));
  }
}

OutlineResult generate_outlines_impl(const geometry_msgs::msg::Polygon& poly,
                                     const RobotGeometry& robot,
                                     double mowing_speed,
                                     std::uint8_t seg_type,
                                     bool obstacle_variant)
{
  OutlineResult result;
  if (poly.points.size() < 3)
  {
    result.warning = "polygon has fewer than 3 vertices";
    result.failed_offset = obstacle_variant;
    return result;
  }

  const double base_inset = robot.footprint.robot_width / 2.0 + robot.outline_offset;
  const double step = robot.tool_width - robot.strip_overlap;

  std::uint32_t seq = 0;
  for (std::uint32_t p = 0; p < robot.outline_passes; ++p)
  {
    // For working areas: positive inset (inward).
    // For obstacles: negative inset (outward expansion).
    double mag = base_inset + static_cast<double>(p) * step;
    double inset = obstacle_variant ? -mag : +mag;

    auto offset_poly =
        mowgli_geometry::offset_polygon_inward(poly.points, inset);

    // Detect degenerate / flipped offsets: signed-area magnitude must remain
    // > some small fraction of the original (a flipped offset has the
    // opposite sign and indicates the inset overshoots the polygon).
    bool flipped = false;
    if (offset_poly.size() >= 3)
    {
      const double sa_in = signed_area(poly.points);
      const double sa_out = signed_area(offset_poly);
      if (sa_in != 0.0 && sa_out != 0.0)
      {
        // Sign flip = polygon turned inside-out.
        flipped = (sa_in > 0.0) != (sa_out > 0.0);
      }
    }

    if (offset_poly.size() < 3 || flipped)
    {
      // Empty / degenerate offset polygon. For the working-area variant
      // this is a warning ("outline pass collapsed"); for the obstacle
      // variant it is a hard failure (caller maps to ERROR_OBSTACLE_OFFSET_FAILED).
      if (obstacle_variant)
      {
        result.failed_offset = true;
        result.warning = "obstacle outline pass " + std::to_string(p) +
                         " produced a degenerate polygon";
        return result;
      }
      else
      {
        // Note: a single failed pass aborts subsequent passes (they would
        // be even more inset and equally degenerate).
        result.warning = "working-area outline pass " + std::to_string(p) +
                         " collapsed; halting outlines for this area";
        return result;
      }
    }

    emit_outline_pass(offset_poly, seg_type, mowing_speed, seq,
                      result.waypoints);
  }

  return result;
}

}  // namespace

OutlineResult generate_working_area_outlines(
    const geometry_msgs::msg::Polygon& area, const RobotGeometry& robot,
    double mowing_speed)
{
  return generate_outlines_impl(
      area, robot, mowing_speed,
      mowgli_interfaces::msg::CoverageWaypoint::SEGMENT_OUTLINE_WORKING_AREA,
      /*obstacle_variant=*/false);
}

OutlineResult generate_obstacle_outlines(
    const geometry_msgs::msg::Polygon& obstacle, const RobotGeometry& robot,
    double mowing_speed)
{
  return generate_outlines_impl(
      obstacle, robot, mowing_speed,
      mowgli_interfaces::msg::CoverageWaypoint::SEGMENT_OUTLINE_OBSTACLE,
      /*obstacle_variant=*/true);
}

}  // namespace mowgli_coverage_planner
