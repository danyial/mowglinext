// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point32.hpp>

#include "mowgli_geometry/pca.hpp"

using geometry_msgs::msg::Point32;
using mowgli_geometry::pca_principal_axis;

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

// Normalises an angle to [-π/2, π/2] for axis comparisons. PCA axes are
// undirected, so atan2 of the principal eigenvector and its negation are
// both valid.
double normalise_axis(double a)
{
  while (a >= M_PI / 2.0) a -= M_PI;
  while (a < -M_PI / 2.0) a += M_PI;
  return a;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Long horizontal rectangle 5×0.5 → principal axis ≈ 0 rad (X axis).
// ─────────────────────────────────────────────────────────────────────────────
TEST(PcaPrincipalAxis, LongHorizontalRectangle)
{
  std::vector<Point32> rect = {
    make_pt(0.0F, 0.0F),
    make_pt(5.0F, 0.0F),
    make_pt(5.0F, 0.5F),
    make_pt(0.0F, 0.5F),
  };
  double angle = pca_principal_axis(rect);
  EXPECT_NEAR(normalise_axis(angle), 0.0, 1e-3);
}

// ─────────────────────────────────────────────────────────────────────────────
// Long vertical rectangle 0.5×5 → principal axis ≈ π/2 (Y axis).
// ─────────────────────────────────────────────────────────────────────────────
TEST(PcaPrincipalAxis, LongVerticalRectangle)
{
  std::vector<Point32> rect = {
    make_pt(0.0F, 0.0F),
    make_pt(0.5F, 0.0F),
    make_pt(0.5F, 5.0F),
    make_pt(0.0F, 5.0F),
  };
  double angle = pca_principal_axis(rect);
  // Y-axis: atan2(1,0) = π/2; the normalised axis representation
  // also accepts -π/2.
  double a = normalise_axis(angle);
  EXPECT_TRUE(std::abs(a - M_PI / 2.0) < 1e-3 ||
              std::abs(a + M_PI / 2.0) < 1e-3)
      << "got angle=" << angle << " (normalised=" << a << ")";
}

// ─────────────────────────────────────────────────────────────────────────────
// 45° tilted long rectangle. Principal axis direction is along (1,1)/√2.
// ─────────────────────────────────────────────────────────────────────────────
TEST(PcaPrincipalAxis, FortyFiveDegreeRectangle)
{
  // Original 5×0.5 horizontal rectangle, rotated by 45°.
  const double s = std::sqrt(2.0) / 2.0;
  std::vector<std::pair<double, double>> raw = {
    {0.0, 0.0}, {5.0, 0.0}, {5.0, 0.5}, {0.0, 0.5},
  };
  std::vector<Point32> rotated;
  for (const auto& [x, y] : raw)
  {
    rotated.push_back(make_pt(static_cast<float>(x * s - y * s),
                              static_cast<float>(x * s + y * s)));
  }
  double angle = pca_principal_axis(rotated);
  EXPECT_NEAR(normalise_axis(angle), M_PI / 4.0, 1e-3);
}

// ─────────────────────────────────────────────────────────────────────────────
// DEGENERATE-RANK CASE: collinear vertices on a 1×0.001 strip — the smallest
// eigenvalue is below the threshold (~1e-9 if the y dimension is true 1e-3 m,
// i.e. covariance ~ 1e-7), so the longest-edge fallback path must be taken.
//
// We construct a strict line on a tilted axis to guarantee rank-1 covariance.
// ─────────────────────────────────────────────────────────────────────────────
TEST(PcaPrincipalAxis, CollinearFallsBackToLongestEdge)
{
  // Points strictly on the line y = x (no Y noise) — covariance is rank 1.
  std::vector<Point32> line = {
    make_pt(0.0F, 0.0F),
    make_pt(1.0F, 1.0F),
    make_pt(2.0F, 2.0F),
    make_pt(3.0F, 3.0F),
  };
  double angle = pca_principal_axis(line);
  EXPECT_NEAR(normalise_axis(angle), M_PI / 4.0, 1e-3)
      << "longest-edge fallback should give 45° for the y=x line";
}

// ─────────────────────────────────────────────────────────────────────────────
// Polygon with fewer than 3 vertices returns 0.0.
// ─────────────────────────────────────────────────────────────────────────────
TEST(PcaPrincipalAxis, DegeneratePolygonReturnsZero)
{
  std::vector<Point32> two_pts = {make_pt(0.0F, 0.0F), make_pt(1.0F, 0.0F)};
  EXPECT_DOUBLE_EQ(pca_principal_axis(two_pts), 0.0);
}
