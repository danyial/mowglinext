// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// Robot-footprint helpers used by the coverage planner. Footprint is a
// 4-corner CCW rectangle in the map frame, built from the body-frame
// dimensions in mowgli_robot.yaml `robot_geometry:` (D-07). The drive
// axle is the rotation centre, so an in-place yaw rotation sweeps a
// disc of radius `rotation_sweep_radius()` around the pose's (x, y).
//
// STUB MARKER (RED gate, plan 01-02 task 2): the implementations below
// are intentionally incorrect placeholders. They are filled in during
// the GREEN gate of this same task.

#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace mowgli_geometry
{

struct FootprintParams
{
  double robot_length;          ///< L: total chassis length front-to-back (m).
  double robot_width;           ///< W: total chassis width (m).
  double drive_axis_x_offset;   ///< dx: axle offset from chassis-centre, X (m).
                                ///< Negative = axle behind centre (YardForce 500).
  double drive_axis_y_offset;   ///< dy: lateral axle offset (m).
};

/// 4-corner CCW rectangular footprint at pose (px, py, yaw) in map frame.
/// Corner order: front-left, front-right, rear-right, rear-left.
/// Drive axle is the rotation centre; chassis centre offset by (dx, dy).
inline std::array<std::pair<double, double>, 4> footprint_polygon(
    double /*px*/, double /*py*/, double /*yaw*/, const FootprintParams& /*p*/)
{
  // STUB: returns 4 zero corners — fails all tests.
  return {{ {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0} }};
}

/// Bounding-disc radius for in-place yaw rotation sweep (max corner distance
/// from the drive axle).
inline double rotation_sweep_radius(const FootprintParams& /*p*/)
{
  return 0.0;  // STUB
}

/// Disc of (cx, cy, radius) is fully inside polygon iff:
///   - the centre is inside the polygon AND
///   - the minimum distance from the centre to any polygon edge >= radius.
/// Returns false for degenerate polygons (n < 3).
inline bool disc_inside_polygon(double /*cx*/, double /*cy*/, double /*radius*/,
                                const geometry_msgs::msg::Polygon& /*polygon*/)
{
  return true;  // STUB — incorrect default value
}

/// Footprint (4 CCW corners) is fully inside polygon (Boost.Geometry within).
inline bool footprint_inside_polygon(
    const std::array<std::pair<double, double>, 4>& /*footprint*/,
    const geometry_msgs::msg::Polygon& /*polygon*/)
{
  return true;  // STUB
}

/// Footprint is disjoint from EVERY obstacle in the list (any intersection -> false).
inline bool footprint_disjoint_obstacles(
    const std::array<std::pair<double, double>, 4>& /*footprint*/,
    const std::vector<geometry_msgs::msg::Polygon>& /*obstacles*/)
{
  return true;  // STUB
}

}  // namespace mowgli_geometry
