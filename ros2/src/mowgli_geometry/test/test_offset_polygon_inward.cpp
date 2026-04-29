// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include "mowgli_geometry/geometry.hpp"

using geometry_msgs::msg::Point32;
using mowgli_geometry::offset_polygon_inward;

namespace
{

Point32 make_pt(float x, float y)
{
  Point32 p;
  p.x = x;
  p.y = y;
  p.z = 0.0F;
  return p;
}

// Computes axis-aligned bounding-box extents (max_x - min_x, max_y - min_y)
// of a polygon. Used to assert "the offset square shrank by ~2 m on each
// dimension" without depending on the order of returned vertices.
std::pair<double, double> bbox_extent(const std::vector<Point32>& poly)
{
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto& p : poly)
  {
    min_x = std::min(min_x, static_cast<double>(p.x));
    max_x = std::max(max_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }
  return {max_x - min_x, max_y - min_y};
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Square 10x10, inward offset by 1.0 m → ~8x8 square.
// ─────────────────────────────────────────────────────────────────────────────
TEST(OffsetPolygonInward, ShrinksSquareByDoubleInset)
{
  std::vector<Point32> square = {
    make_pt(0.0F, 0.0F),
    make_pt(10.0F, 0.0F),
    make_pt(10.0F, 10.0F),
    make_pt(0.0F, 10.0F),
  };

  auto out = offset_polygon_inward(square, 1.0);
  ASSERT_EQ(out.size(), 4U);

  auto [ex, ey] = bbox_extent(out);
  EXPECT_NEAR(ex, 8.0, 1e-3);
  EXPECT_NEAR(ey, 8.0, 1e-3);
}

// ─────────────────────────────────────────────────────────────────────────────
// Negative inset = outward offset. Square 10x10, inset = -1.0 → ~12x12.
// ─────────────────────────────────────────────────────────────────────────────
TEST(OffsetPolygonInward, NegativeInsetGrowsSquare)
{
  std::vector<Point32> square = {
    make_pt(0.0F, 0.0F),
    make_pt(10.0F, 0.0F),
    make_pt(10.0F, 10.0F),
    make_pt(0.0F, 10.0F),
  };

  auto out = offset_polygon_inward(square, -1.0);
  ASSERT_EQ(out.size(), 4U);

  auto [ex, ey] = bbox_extent(out);
  EXPECT_NEAR(ex, 12.0, 1e-3);
  EXPECT_NEAR(ey, 12.0, 1e-3);
}

// ─────────────────────────────────────────────────────────────────────────────
// Closing-vertex deduplication: input ends with a duplicate of the first
// vertex. The fix from #50-phase-2 (2026-04-27) must keep all 4 corners
// in the output instead of dropping V0 to a 3-vertex triangle.
// ─────────────────────────────────────────────────────────────────────────────
TEST(OffsetPolygonInward, ClosingVertexDeduplicated)
{
  std::vector<Point32> square_with_dup = {
    make_pt(0.0F, 0.0F),
    make_pt(10.0F, 0.0F),
    make_pt(10.0F, 10.0F),
    make_pt(0.0F, 10.0F),
    make_pt(0.0F, 0.0F),  // duplicate closing vertex
  };

  auto out = offset_polygon_inward(square_with_dup, 1.0);
  ASSERT_EQ(out.size(), 4U)
      << "closing-vertex dedup must produce 4 corners, not 3 (#50-phase-2)";

  auto [ex, ey] = bbox_extent(out);
  EXPECT_NEAR(ex, 8.0, 1e-3);
  EXPECT_NEAR(ey, 8.0, 1e-3);
}

// ─────────────────────────────────────────────────────────────────────────────
// Inset = 0 is a no-op: returns the polygon unchanged.
// ─────────────────────────────────────────────────────────────────────────────
TEST(OffsetPolygonInward, ZeroInsetIsNoOp)
{
  std::vector<Point32> tri = {
    make_pt(0.0F, 0.0F),
    make_pt(4.0F, 0.0F),
    make_pt(2.0F, 3.0F),
  };

  auto out = offset_polygon_inward(tri, 0.0);
  ASSERT_EQ(out.size(), tri.size());
  for (size_t i = 0; i < tri.size(); ++i)
  {
    EXPECT_FLOAT_EQ(out[i].x, tri[i].x);
    EXPECT_FLOAT_EQ(out[i].y, tri[i].y);
  }
}
