// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/boustrophedon_sweeper.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <mowgli_geometry/geometry.hpp>
#include <mowgli_interfaces/msg/coverage_waypoint.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace mowgli_coverage_planner
{

namespace
{

using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;

/// 2D rotation. (x, y) -> (x*cos - y*sin, x*sin + y*cos).
inline std::pair<double, double> rotate_xy(double x, double y, double sin_t,
                                           double cos_t)
{
  return { x * cos_t - y * sin_t, x * sin_t + y * cos_t };
}

geometry_msgs::msg::Polygon rotate_polygon(
    const geometry_msgs::msg::Polygon& poly, double sin_t, double cos_t)
{
  geometry_msgs::msg::Polygon out;
  out.points.reserve(poly.points.size());
  for (const auto& p : poly.points)
  {
    auto [rx, ry] = rotate_xy(static_cast<double>(p.x),
                              static_cast<double>(p.y), sin_t, cos_t);
    geometry_msgs::msg::Point32 q;
    q.x = static_cast<float>(rx);
    q.y = static_cast<float>(ry);
    q.z = 0.0F;
    out.points.push_back(q);
  }
  return out;
}

/// Compute Y intersections of vertical scan-line @ x with polygon edges.
/// Skips vertical edges (a.x == b.x).
std::vector<double> scan_line_intersections(
    const geometry_msgs::msg::Polygon& poly, double x)
{
  std::vector<double> ys;
  const auto& pts = poly.points;
  const std::size_t n = pts.size();
  if (n < 3) return ys;

  // Treat the polygon as closed; iterate edges (i, i+1).
  for (std::size_t i = 0; i < n; ++i)
  {
    const std::size_t j = (i + 1) % n;
    const double ax = static_cast<double>(pts[i].x);
    const double ay = static_cast<double>(pts[i].y);
    const double bx = static_cast<double>(pts[j].x);
    const double by = static_cast<double>(pts[j].y);
    if (std::abs(bx - ax) < 1e-12) continue;  // vertical edge
    if ((ax - x) * (bx - x) > 0.0) continue;  // edge on one side of x
    const double t = (x - ax) / (bx - ax);
    if (t < 0.0 || t > 1.0) continue;
    ys.push_back(ay + t * (by - ay));
  }
  std::sort(ys.begin(), ys.end());
  return ys;
}

/// Build a CoverageWaypoint at (x, y) with the given yaw + sequence id.
CoverageWaypoint mk_swath_waypoint(double x, double y, double yaw,
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
  wp.blade_enabled = true;
  wp.segment_type = CoverageWaypoint::SEGMENT_MOWING_BOUSTROPHEDON;
  return wp;
}

}  // namespace

SweepResult sweep(const geometry_msgs::msg::Polygon& area,
                  const std::vector<geometry_msgs::msg::Polygon>& obstacles,
                  double mow_angle_rad, const RobotGeometry& robot,
                  double mowing_speed,
                  const NarrowStrategyCallback& narrow_cb)
{
  SweepResult result;
  if (area.points.size() < 3 || robot.tool_width <= 0.0 ||
      robot.footprint.robot_width <= 0.0)
  {
    return result;
  }

  // 1. Rotation that aligns scan lines with the X axis.
  //    rot_angle = pi/2 - mow_angle_rad. We rotate everything by -rot_angle
  //    (i.e. rotate by mow_angle_rad - pi/2) so that scan lines are vertical
  //    in the rotated frame and we step along X.
  const double rot = M_PI / 2.0 - mow_angle_rad;
  const double sin_rot = std::sin(rot);
  const double cos_rot = std::cos(rot);
  const double sin_inv = std::sin(-rot);
  const double cos_inv = std::cos(-rot);

  // 2. Compute the worst-case footprint corner distance from the pose centre
  //    (= base_link). The robot can be at ANY yaw on a strip endpoint, so we
  //    use the MAX corner distance to guarantee that any pose-centre inside
  //    an inset-by-this-distance polygon places ALL footprint corners safely
  //    within the original polygon — regardless of yaw and regardless of
  //    polygon shape (convex / concave / slanted edges).
  //
  //    This replaces the old per-axis approach (x_inset_innermost using
  //    robot_width/2, y_inset using robot_length/2) which assumed the
  //    polygon's edges were perpendicular to the strip direction at every
  //    scanline. That assumption breaks for any non-axis-aligned polygon
  //    edge — a slanted hexagon edge or a concave dent — when the footprint
  //    extends laterally or forward into a region where the polygon
  //    boundary has moved closer than the per-axis inset accounted for.
  //    See GH issue #68 for the failure cases.
  const double front_x = robot.footprint.robot_length / 2.0
                         - robot.footprint.drive_axis_x_offset;
  const double rear_x = -robot.footprint.robot_length / 2.0
                        - robot.footprint.drive_axis_x_offset;
  const double half_w = robot.footprint.robot_width / 2.0;
  const double max_corner_dist = std::max(
      std::hypot(front_x, half_w),
      std::hypot(std::abs(rear_x), half_w));

  // 3. Step. Same as before.
  const double step = robot.tool_width - robot.strip_overlap;
  if (step <= 0.0)
  {
    return result;
  }

  // 4. Inset the working area inward by max_corner_dist + outline_offset
  //    + (passes-1)*step. This shrinks the area to a "pose-safe" sub-polygon
  //    where any pose centre inside it is automatically clear of footprint
  //    violations against any edge of the original polygon. The strip
  //    layout sweeps inside this inset polygon.
  const double area_inset_dist =
      max_corner_dist + robot.outline_offset
      + static_cast<double>(robot.outline_passes > 0u
                                ? robot.outline_passes - 1u
                                : 0u) * step;
  auto inset_pts =
      mowgli_geometry::offset_polygon_inward(area.points, area_inset_dist);
  if (inset_pts.size() < 3)
  {
    // Area collapsed entirely under the inset — too small / narrow to mow.
    // Caller's narrow_area_strategy can take over via a separate path.
    return result;
  }
  geometry_msgs::msg::Polygon inset_area;
  inset_area.points = inset_pts;

  // 5. Rotate the INSET area into scan-frame.
  auto rot_area = rotate_polygon(inset_area, sin_rot, cos_rot);

  // 6. Rotate obstacles + expand outward by max_corner_dist + outline_offset.
  //    Same conservative reasoning as the area inset: any pose-centre
  //    outside the expanded obstacle keeps all footprint corners clear of
  //    the original obstacle, regardless of yaw.
  const double obs_outward = max_corner_dist + robot.outline_offset;
  std::vector<geometry_msgs::msg::Polygon> expanded_obstacles;
  expanded_obstacles.reserve(obstacles.size());
  for (const auto& obs : obstacles)
  {
    if (obs.points.size() < 3) continue;
    auto rot_obs = rotate_polygon(obs, sin_rot, cos_rot);
    // Negative inset = outward expansion.
    auto exp_pts =
        mowgli_geometry::offset_polygon_inward(rot_obs.points, -obs_outward);
    if (exp_pts.size() < 3) continue;  // degenerate; skip
    geometry_msgs::msg::Polygon ep;
    ep.points = exp_pts;
    expanded_obstacles.push_back(ep);
  }

  // 7. AABB of rotated INSET area.
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto& p : rot_area.points)
  {
    min_x = std::min(min_x, static_cast<double>(p.x));
    max_x = std::max(max_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }

  // 8. Strip layout. With the inset polygon already accounting for footprint
  //    corners + outline_offset + (passes-1)*step, the scanline x range is
  //    just the inset AABB plus a half-step centring offset so successive
  //    strips are step-spaced.
  const double x_first = min_x + step / 2.0;
  const double x_last = max_x;

  // 9. y_inset is no longer needed at swath endpoints — the strip's south
  //    and north endpoints land directly at the INSET polygon's scanline
  //    intersections, which are already at distance ≥ max_corner_dist
  //    + outline_offset from the original polygon edges.
  const double y_inset = 0.0;

  std::uint32_t seq = 0;
  std::uint32_t col = 0;
  for (double x = x_first; x <= x_last + 1e-9; x += step, ++col)
  {
    // Compute intersections with the rotated area + each expanded obstacle.
    auto area_ys = scan_line_intersections(rot_area, x);
    if (area_ys.size() < 2) continue;

    // For each (y_lo, y_hi) area pair, subtract the obstacle bands.
    // Even-odd fill.
    for (std::size_t k = 0; k + 1 < area_ys.size(); k += 2)
    {
      double y_lo_outer = area_ys[k];
      double y_hi_outer = area_ys[k + 1];
      // Apply y_inset at both ends.
      double y_lo = y_lo_outer + y_inset;
      double y_hi = y_hi_outer - y_inset;
      if (y_hi <= y_lo) continue;  // pair collapses entirely under inset

      // Collect obstacle bands within (y_lo, y_hi). Each band is a (lo, hi)
      // pair from the expanded obstacle's intersections at this scan line.
      std::vector<std::pair<double, double>> obs_bands;
      for (const auto& obs : expanded_obstacles)
      {
        auto obs_ys = scan_line_intersections(obs, x);
        for (std::size_t j = 0; j + 1 < obs_ys.size(); j += 2)
        {
          double a = obs_ys[j];
          double b = obs_ys[j + 1];
          if (b <= y_lo || a >= y_hi) continue;
          obs_bands.emplace_back(std::max(a, y_lo), std::min(b, y_hi));
        }
      }
      // Merge overlapping bands.
      std::sort(obs_bands.begin(), obs_bands.end());
      std::vector<std::pair<double, double>> merged;
      for (const auto& b : obs_bands)
      {
        if (!merged.empty() && b.first <= merged.back().second)
        {
          merged.back().second = std::max(merged.back().second, b.second);
        }
        else
        {
          merged.push_back(b);
        }
      }

      // Build the list of free sub-segments: [y_lo, y_hi] minus merged bands.
      std::vector<std::pair<double, double>> free_segs;
      double cursor = y_lo;
      for (const auto& [a, b] : merged)
      {
        if (a > cursor) free_segs.emplace_back(cursor, a);
        cursor = std::max(cursor, b);
      }
      if (cursor < y_hi) free_segs.emplace_back(cursor, y_hi);

      // Direction alternation per column index. Even cols: low->high.
      const bool low_to_high = (col % 2u) == 0u;

      // Iterate free segments in the swath traversal direction so the
      // boustrophedon pattern is preserved.
      if (!low_to_high)
      {
        std::reverse(free_segs.begin(), free_segs.end());
      }

      for (const auto& seg : free_segs)
      {
        double y0 = low_to_high ? seg.first : seg.second;
        double y1 = low_to_high ? seg.second : seg.first;
        const double seg_len = std::abs(seg.second - seg.first);

        // Rotate (x, y0) and (x, y1) back to MAP frame.
        auto [mx0, my0] = rotate_xy(x, y0, sin_inv, cos_inv);
        auto [mx1, my1] = rotate_xy(x, y1, sin_inv, cos_inv);

        if (seg_len < robot.footprint.robot_length)
        {
          // Narrow segment -> dispatch to strategy callback if provided.
          if (narrow_cb)
          {
            ScanSegment narrow;
            narrow.start.x = static_cast<float>(mx0);
            narrow.start.y = static_cast<float>(my0);
            narrow.end.x = static_cast<float>(mx1);
            narrow.end.y = static_cast<float>(my1);
            auto strat = narrow_cb(narrow, area, obstacles, robot, mow_angle_rad);
            if (!strat.warning.empty())
              result.warnings.push_back(strat.warning);
            for (auto& wp : strat.waypoints)
            {
              wp.sequence_id = seq++;
              result.waypoints.push_back(std::move(wp));
            }
          }
          // else: silently drop (used by tests that don't supply a callback)
          continue;
        }

        // Full-length swath: emit start + end pair. yaw = direction-of-travel.
        const double yaw = std::atan2(my1 - my0, mx1 - mx0);
        result.waypoints.push_back(
            mk_swath_waypoint(mx0, my0, yaw, mowing_speed, seq++));
        result.waypoints.push_back(
            mk_swath_waypoint(mx1, my1, yaw, mowing_speed, seq++));
      }
    }
  }

  return result;
}

}  // namespace mowgli_coverage_planner
