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
// Polygon containment / disjointness checks use Boost.Geometry (header-
// only) — see RESEARCH §5.2 - §5.3 for the to_bg adapter pattern.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

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

namespace detail
{

namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint>;

/// Convert a geometry_msgs Polygon to a Boost.Geometry polygon. Calls
/// bg::correct() to enforce the closing point + winding. Empty / degenerate
/// inputs produce an empty BgPolygon (caller must guard).
inline BgPolygon to_bg(const geometry_msgs::msg::Polygon& polygon)
{
  BgPolygon out;
  for (const auto& p : polygon.points)
  {
    bg::append(out.outer(),
               BgPoint(static_cast<double>(p.x), static_cast<double>(p.y)));
  }
  bg::correct(out);
  return out;
}

inline BgPolygon to_bg(const std::array<std::pair<double, double>, 4>& corners)
{
  BgPolygon out;
  for (const auto& [x, y] : corners)
  {
    bg::append(out.outer(), BgPoint(x, y));
  }
  bg::correct(out);
  return out;
}

}  // namespace detail

/// 4-corner CCW rectangular footprint at pose (px, py, yaw) in map frame.
/// Corner order: front-left, front-right, rear-right, rear-left.
/// Drive axle is the rotation centre; chassis centre offset by (dx, dy).
inline std::array<std::pair<double, double>, 4> footprint_polygon(
    double px, double py, double yaw, const FootprintParams& p)
{
  // Body-frame corners with the drive axle at the origin (RESEARCH §3.1).
  // The axle is offset from the chassis centre by (dx, dy), so:
  //   front edge X = +L/2 - dx   (axle behind centre → larger positive)
  //   rear  edge X = -L/2 - dx
  //   left  edge Y = +W/2 - dy
  //   right edge Y = -W/2 - dy
  const double half_l = p.robot_length / 2.0;
  const double half_w = p.robot_width / 2.0;
  const double front_x = +half_l - p.drive_axis_x_offset;
  const double rear_x = -half_l - p.drive_axis_x_offset;
  const double left_y = +half_w - p.drive_axis_y_offset;
  const double right_y = -half_w - p.drive_axis_y_offset;

  const std::array<std::pair<double, double>, 4> body = {{
      { front_x, left_y },   // front-left
      { front_x, right_y },  // front-right
      { rear_x,  right_y },  // rear-right
      { rear_x,  left_y },   // rear-left
  }};

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::array<std::pair<double, double>, 4> out;
  for (size_t i = 0; i < 4; ++i)
  {
    out[i].first  = px + body[i].first * c - body[i].second * s;
    out[i].second = py + body[i].first * s + body[i].second * c;
  }
  return out;
}

/// Bounding-disc radius for the in-place yaw rotation safety check
/// (max corner distance from the drive axle). Conservative: a disc of
/// this radius around (px, py) contains every footprint corner across
/// all yaw values, so disc-inside-polygon is sufficient (and stricter
/// than necessary) for safe rotation clearance.
inline double rotation_sweep_radius(const FootprintParams& p)
{
  const double half_l = p.robot_length / 2.0;
  const double half_w = p.robot_width / 2.0;
  const double dx = p.drive_axis_x_offset;
  const double dy = p.drive_axis_y_offset;
  const double r1 = std::hypot(+half_l - dx, +half_w - dy);
  const double r2 = std::hypot(+half_l - dx, -half_w - dy);
  const double r3 = std::hypot(-half_l - dx, -half_w - dy);
  const double r4 = std::hypot(-half_l - dx, +half_w - dy);
  return std::max({r1, r2, r3, r4});
}

/// Disc of (cx, cy, radius) is fully inside polygon iff:
///   - the centre is inside the polygon AND
///   - the minimum distance from the centre to any polygon edge >= radius.
/// Returns false for degenerate polygons (n < 3).
inline bool disc_inside_polygon(double cx, double cy, double radius,
                                const geometry_msgs::msg::Polygon& polygon)
{
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 3)
  {
    return false;
  }

  // 1. Centre inside polygon (Boost.Geometry within).
  detail::BgPolygon poly_bg = detail::to_bg(polygon);
  detail::BgPoint centre(cx, cy);
  if (!boost::geometry::within(centre, poly_bg))
  {
    return false;
  }

  // 2. Minimum distance from centre to any polygon edge >= radius.
  double min_dist_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double ax = static_cast<double>(pts[j].x);
    const double ay = static_cast<double>(pts[j].y);
    const double bx = static_cast<double>(pts[i].x);
    const double by = static_cast<double>(pts[i].y);

    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-12)
    {
      t = std::clamp(((cx - ax) * dx + (cy - ay) * dy) / len2, 0.0, 1.0);
    }
    const double qx = ax + t * dx;
    const double qy = ay + t * dy;
    const double d2 = (cx - qx) * (cx - qx) + (cy - qy) * (cy - qy);
    if (d2 < min_dist_sq)
    {
      min_dist_sq = d2;
    }
  }

  return std::sqrt(min_dist_sq) >= radius;
}

/// Footprint (4 CCW corners) is fully inside polygon (Boost.Geometry within).
inline bool footprint_inside_polygon(
    const std::array<std::pair<double, double>, 4>& footprint,
    const geometry_msgs::msg::Polygon& polygon)
{
  if (polygon.points.size() < 3)
  {
    return false;
  }
  detail::BgPolygon area_bg = detail::to_bg(polygon);
  detail::BgPolygon fp_bg = detail::to_bg(footprint);
  // covered_by allows boundary touching; within requires strict interior.
  // For coverage planning we accept boundary contact, so use covered_by.
  return boost::geometry::covered_by(fp_bg, area_bg);
}

/// Footprint is disjoint from EVERY obstacle in the list (any intersection -> false).
inline bool footprint_disjoint_obstacles(
    const std::array<std::pair<double, double>, 4>& footprint,
    const std::vector<geometry_msgs::msg::Polygon>& obstacles)
{
  detail::BgPolygon fp_bg = detail::to_bg(footprint);
  for (const auto& obs : obstacles)
  {
    if (obs.points.size() < 3) continue;  // skip degenerate obstacles
    detail::BgPolygon obs_bg = detail::to_bg(obs);
    if (!boost::geometry::disjoint(fp_bg, obs_bg))
    {
      return false;
    }
  }
  return true;
}

}  // namespace mowgli_geometry
