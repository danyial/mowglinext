// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include "mowgli_geometry/footprint.hpp"

using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Polygon;
using mowgli_geometry::FootprintParams;
using mowgli_geometry::disc_inside_polygon;
using mowgli_geometry::footprint_disjoint_obstacles;
using mowgli_geometry::footprint_inside_polygon;
using mowgli_geometry::footprint_polygon;
using mowgli_geometry::rotation_sweep_radius;

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

Polygon make_square(float min_x, float min_y, float side)
{
  Polygon poly;
  poly.points.push_back(make_pt(min_x, min_y));
  poly.points.push_back(make_pt(min_x + side, min_y));
  poly.points.push_back(make_pt(min_x + side, min_y + side));
  poly.points.push_back(make_pt(min_x, min_y + side));
  return poly;
}

constexpr FootprintParams kYf500 = {
  /*robot_length=*/0.60,
  /*robot_width=*/0.40,
  /*drive_axis_x_offset=*/-0.20,  // axle behind chassis centre
  /*drive_axis_y_offset=*/0.0,
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// footprint_polygon: identity rotation at the origin places the 4 body-frame
// corners with the drive axle at origin. With dx=-0.20, the front edge sits
// at x = +0.50 and the rear edge at x = -0.10.
// ─────────────────────────────────────────────────────────────────────────────
TEST(FootprintPolygon, IdentityPoseAtOrigin)
{
  auto fp = footprint_polygon(0.0, 0.0, 0.0, kYf500);
  ASSERT_EQ(fp.size(), 4U);

  // Front-left, front-right, rear-right, rear-left order.
  EXPECT_NEAR(fp[0].first,  0.50, 1e-6);
  EXPECT_NEAR(fp[0].second, 0.20, 1e-6);
  EXPECT_NEAR(fp[1].first,  0.50, 1e-6);
  EXPECT_NEAR(fp[1].second, -0.20, 1e-6);
  EXPECT_NEAR(fp[2].first,  -0.10, 1e-6);
  EXPECT_NEAR(fp[2].second, -0.20, 1e-6);
  EXPECT_NEAR(fp[3].first,  -0.10, 1e-6);
  EXPECT_NEAR(fp[3].second, 0.20, 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// 90° yaw rotates the +x face onto the +y direction.
// ─────────────────────────────────────────────────────────────────────────────
TEST(FootprintPolygon, NinetyDegYaw)
{
  auto fp = footprint_polygon(0.0, 0.0, M_PI / 2.0, kYf500);
  ASSERT_EQ(fp.size(), 4U);

  // Front-left (was +0.50, +0.20) → (-0.20, +0.50)
  EXPECT_NEAR(fp[0].first,  -0.20, 1e-6);
  EXPECT_NEAR(fp[0].second,  0.50, 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// rotation_sweep_radius: for the YardForce 500, the worst-case corner is the
// front-left/front-right (axle at -0.20 → distance 0.50; combined with the
// 0.20 half-width gives sqrt(0.50^2 + 0.20^2) ≈ 0.5385 m).
// ─────────────────────────────────────────────────────────────────────────────
TEST(FootprintPolygon, RotationSweepRadiusYardForce500)
{
  double r = rotation_sweep_radius(kYf500);
  EXPECT_NEAR(r, std::hypot(0.50, 0.20), 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// Footprint at (5, 5, 0) inside a 10×10 area is fully inside.
// Footprint at (0.1, 0.1, 0) has front-right corner outside the area.
// ─────────────────────────────────────────────────────────────────────────────
TEST(FootprintInsidePolygon, CenteredFootprintInside)
{
  Polygon area = make_square(0.0F, 0.0F, 10.0F);
  auto fp = footprint_polygon(5.0, 5.0, 0.0, kYf500);
  EXPECT_TRUE(footprint_inside_polygon(fp, area));
}

TEST(FootprintInsidePolygon, OffEdgeFootprintOutside)
{
  Polygon area = make_square(0.0F, 0.0F, 10.0F);
  auto fp = footprint_polygon(0.1, 0.1, 0.0, kYf500);
  EXPECT_FALSE(footprint_inside_polygon(fp, area))
      << "footprint at (0.1, 0.1) clips the area boundary";
}

// ─────────────────────────────────────────────────────────────────────────────
// disc_inside_polygon: small disc centered at (5,5) inside a 10×10 fits;
// large disc (radius 1.0) at (0.5, 0.5) extends past the polygon edge.
// ─────────────────────────────────────────────────────────────────────────────
TEST(DiscInsidePolygon, SmallCenteredDiscFits)
{
  Polygon area = make_square(0.0F, 0.0F, 10.0F);
  EXPECT_TRUE(disc_inside_polygon(5.0, 5.0, 0.5, area));
}

TEST(DiscInsidePolygon, DiscExtendsBeyondEdge)
{
  Polygon area = make_square(0.0F, 0.0F, 10.0F);
  EXPECT_FALSE(disc_inside_polygon(0.5, 0.5, 1.0, area))
      << "disc at (0.5, 0.5) with r=1 extends past x=0";
}

TEST(DiscInsidePolygon, DegeneratePolygonRejected)
{
  Polygon line;  // < 3 vertices
  line.points.push_back(make_pt(0.0F, 0.0F));
  line.points.push_back(make_pt(1.0F, 0.0F));
  EXPECT_FALSE(disc_inside_polygon(0.0, 0.0, 0.0, line));
}

// ─────────────────────────────────────────────────────────────────────────────
// footprint_disjoint_obstacles: footprint (5,5) is disjoint from an obstacle
// far away at (8,8). Same footprint clipping an obstacle at (5.3, 5) is NOT
// disjoint — must return false on any intersection.
// ─────────────────────────────────────────────────────────────────────────────
TEST(FootprintDisjointObstacles, ClearOfDistantObstacles)
{
  auto fp = footprint_polygon(5.0, 5.0, 0.0, kYf500);
  std::vector<Polygon> obstacles = {make_square(8.0F, 8.0F, 0.5F)};
  EXPECT_TRUE(footprint_disjoint_obstacles(fp, obstacles));
}

TEST(FootprintDisjointObstacles, IntersectingObstacleRejected)
{
  auto fp = footprint_polygon(5.0, 5.0, 0.0, kYf500);
  // Obstacle overlaps the footprint's right side (footprint extends to x=5.5)
  std::vector<Polygon> obstacles = {make_square(5.3F, 4.9F, 0.4F)};
  EXPECT_FALSE(footprint_disjoint_obstacles(fp, obstacles));
}

TEST(FootprintDisjointObstacles, EmptyListIsClear)
{
  auto fp = footprint_polygon(0.0, 0.0, 0.0, kYf500);
  EXPECT_TRUE(footprint_disjoint_obstacles(fp, {}));
}
