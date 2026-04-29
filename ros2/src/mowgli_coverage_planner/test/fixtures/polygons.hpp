// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// Reusable polygon test fixtures shared by Plan 01-07's geometry / validator
// tests. All builders return geometry_msgs::msg::Polygon with the closing
// vertex repeated at the end (matches the convention some codebase helpers —
// notably mowgli_geometry::offset_polygon_inward — defensively dedupe).
//
// Coordinates are in metres, map frame (X=east, Y=north). CCW winding
// throughout (interior on the left as you traverse).

#include <cmath>
#include <cstddef>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

namespace mowgli_coverage_planner::test
{

inline geometry_msgs::msg::Point32 mk_p(double x, double y)
{
  geometry_msgs::msg::Point32 p;
  p.x = static_cast<float>(x);
  p.y = static_cast<float>(y);
  p.z = 0.0F;
  return p;
}

/// Axis-aligned square centred at (cx, cy) with side `side`. CCW winding.
inline geometry_msgs::msg::Polygon make_square(double cx, double cy, double side)
{
  const double h = side / 2.0;
  geometry_msgs::msg::Polygon poly;
  poly.points.push_back(mk_p(cx - h, cy - h));
  poly.points.push_back(mk_p(cx + h, cy - h));
  poly.points.push_back(mk_p(cx + h, cy + h));
  poly.points.push_back(mk_p(cx - h, cy + h));
  poly.points.push_back(mk_p(cx - h, cy - h));  // closing vertex
  return poly;
}

/// Narrow strip lying horizontally: width along Y, length along X. Centre
/// at (cx, cy). Used to exercise the narrow-area strategies (R-13).
inline geometry_msgs::msg::Polygon make_horizontal_strip(double cx, double cy,
                                                        double length,
                                                        double width)
{
  const double hl = length / 2.0;
  const double hw = width / 2.0;
  geometry_msgs::msg::Polygon poly;
  poly.points.push_back(mk_p(cx - hl, cy - hw));
  poly.points.push_back(mk_p(cx + hl, cy - hw));
  poly.points.push_back(mk_p(cx + hl, cy + hw));
  poly.points.push_back(mk_p(cx - hl, cy + hw));
  poly.points.push_back(mk_p(cx - hl, cy - hw));
  return poly;
}

/// Approximate circle as an n-segment regular polygon, CCW.
inline geometry_msgs::msg::Polygon make_circle(double cx, double cy,
                                               double radius,
                                               std::size_t n_segments = 16)
{
  geometry_msgs::msg::Polygon poly;
  if (n_segments < 3)
    n_segments = 3;
  for (std::size_t i = 0; i < n_segments; ++i)
  {
    const double t = (2.0 * M_PI * static_cast<double>(i)) /
                     static_cast<double>(n_segments);
    poly.points.push_back(mk_p(cx + radius * std::cos(t),
                               cy + radius * std::sin(t)));
  }
  // Closing vertex.
  poly.points.push_back(poly.points.front());
  return poly;
}

/// L-shaped strip: a horizontal arm + a vertical arm meeting at the origin.
/// Both arms are `arm_len` long and `arm_width` wide. Centre of L is at
/// (cx, cy). CCW winding.
inline geometry_msgs::msg::Polygon make_l_strip(double cx, double cy,
                                                double arm_len,
                                                double arm_width)
{
  // L is the union of two rectangles. We trace the outline CCW.
  //
  //   (cx, cy + arm_len)
  //          *--*  (cx + arm_width, cy + arm_len)
  //          |  |
  //          |  |
  //          |  *--------------*  (cx + arm_len, cy + arm_width)
  //          |                 |
  //          *-----------------*  (cx + arm_len, cy)  → start at (cx, cy)
  //   (cx, cy)
  geometry_msgs::msg::Polygon poly;
  poly.points.push_back(mk_p(cx, cy));
  poly.points.push_back(mk_p(cx + arm_len, cy));
  poly.points.push_back(mk_p(cx + arm_len, cy + arm_width));
  poly.points.push_back(mk_p(cx + arm_width, cy + arm_width));
  poly.points.push_back(mk_p(cx + arm_width, cy + arm_len));
  poly.points.push_back(mk_p(cx, cy + arm_len));
  poly.points.push_back(mk_p(cx, cy));  // closing vertex
  return poly;
}

}  // namespace mowgli_coverage_planner::test
