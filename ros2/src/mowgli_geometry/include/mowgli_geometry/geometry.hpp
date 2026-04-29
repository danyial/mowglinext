// Copyright (C) 2024-2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

// Header-only polygon geometry helpers used by mowgli_coverage_planner and
// mowgli_map. Functions are PROMOTED VERBATIM from the previous file-scope
// definitions in mowgli_map/src/map_server_node.cpp (lines 1389-1413,
// 2421-2502, 3298-3416 as of HEAD f3413f8a). Behaviour is byte-equivalent
// modulo namespacing + `inline` linkage; do not change algorithms here
// without also updating any consumers that expect identical results.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

namespace mowgli_geometry
{

/// Ray-casting point-in-polygon test. Returns false for polygons with
/// fewer than 3 vertices.
///
/// PROMOTED VERBATIM from mowgli_map/src/map_server_node.cpp lines
/// 1389-1413 (was MapServerNode::point_in_polygon, file-scope `static`
/// before promotion).
inline bool point_in_polygon(const geometry_msgs::msg::Point32& pt,
                             const geometry_msgs::msg::Polygon& polygon) noexcept
{
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 3)
  {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const float xi = pts[i].x, yi = pts[i].y;
    const float xj = pts[j].x, yj = pts[j].y;

    const bool intersect =
        ((yi > pt.y) != (yj > pt.y)) && (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi);

    if (intersect)
    {
      inside = !inside;
    }
  }
  return inside;
}

/// Convex hull (Andrew's monotone chain), O(n log n).
///
/// PROMOTED VERBATIM from mowgli_map/src/map_server_node.cpp lines
/// 2421-2457 (was MapServerNode::convex_hull). Input is a copy because
/// the algorithm sorts in place.
inline std::vector<std::pair<double, double>> convex_hull(
    std::vector<std::pair<double, double>> pts)
{
  auto cross = [](const auto& O, const auto& A, const auto& B)
  {
    return (A.first - O.first) * (B.second - O.second) -
           (A.second - O.second) * (B.first - O.first);
  };

  int n = static_cast<int>(pts.size());
  if (n < 3)
    return pts;

  std::sort(pts.begin(), pts.end());

  std::vector<std::pair<double, double>> hull(2 * n);
  int k = 0;

  // Lower hull
  for (int i = 0; i < n; ++i)
  {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
      k--;
    hull[k++] = pts[i];
  }

  // Upper hull
  for (int i = n - 2, t = k + 1; i >= 0; i--)
  {
    while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
      k--;
    hull[k++] = pts[i];
  }

  hull.resize(k - 1);
  return hull;
}

/// Optimal mowing direction via the Minimum Bounding Rectangle method
/// (rotating-calipers, evaluating each convex-hull edge as the candidate
/// strip-axis). Returns the angle (rad) of the orientation that minimises
/// the perpendicular extent of the hull. Strips run parallel to that
/// direction. Returns 0.0 for polygons that degenerate to fewer than 3
/// hull vertices.
///
/// PROMOTED VERBATIM from mowgli_map/src/map_server_node.cpp lines
/// 2459-2502 (was MapServerNode::compute_optimal_mow_angle).
inline double compute_optimal_mow_angle(const geometry_msgs::msg::Polygon& poly)
{
  std::vector<std::pair<double, double>> pts;
  pts.reserve(poly.points.size());
  for (const auto& p : poly.points)
    pts.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y));

  auto hull = convex_hull(std::move(pts));
  if (hull.size() < 3)
    return 0.0;

  double best_angle = 0.0;
  double min_perp_extent = 1e9;
  int nh = static_cast<int>(hull.size());

  for (int i = 0; i < nh; ++i)
  {
    int j = (i + 1) % nh;
    double edge_dx = hull[j].first - hull[i].first;
    double edge_dy = hull[j].second - hull[i].second;
    double edge_angle = std::atan2(edge_dy, edge_dx);

    double cos_a = std::cos(-edge_angle);
    double sin_a = std::sin(-edge_angle);

    // Compute bounding box of hull rotated to align this edge with X axis
    double min_y = 1e9, max_y = -1e9;
    for (const auto& [hx, hy] : hull)
    {
      double ry = sin_a * hx + cos_a * hy;
      min_y = std::min(min_y, ry);
      max_y = std::max(max_y, ry);
    }

    double perp_extent = max_y - min_y;
    if (perp_extent < min_perp_extent)
    {
      min_perp_extent = perp_extent;
      best_angle = edge_angle;
    }
  }

  return best_angle;
}

// ─────────────────────────────────────────────────────────────────────────────
// Polygon inward Minkowski offset (#50 phase 2). Vertex-bisector
// implementation: for each vertex shift along the bisector of its two
// adjacent inward normals by inset / sin(half_interior_angle). Robust on
// convex polygons; concave polygons may produce self-intersections that
// are tolerated as long as the resulting boundary stays inside the
// polygon — collision_monitor catches the rest at runtime.
//
// PROMOTED VERBATIM from mowgli_map/src/map_server_node.cpp lines
// 3298-3416 (was MapServerNode::offset_polygon_inward, member function
// before promotion). Includes the closing-vertex deduplication and the
// CCW/CW winding detection added during the live #50-phase-2 fix on
// 2026-04-27. DO NOT simplify.
// ─────────────────────────────────────────────────────────────────────────────
inline std::vector<geometry_msgs::msg::Point32> offset_polygon_inward(
    const std::vector<geometry_msgs::msg::Point32>& poly_in, double inset)
{
  // Inset == 0 is a no-op (returns the polygon unchanged). Negative inset
  // is an outward offset (used for obstacle outlines, see compute_outline_-
  // path); both are valid signs for the bisector math below.
  if (poly_in.size() < 3 || std::abs(inset) < 1e-9)
  {
    return poly_in;
  }

  // Dedupe consecutive duplicate vertices and any explicit closing vertex
  // (some upstream sources — e.g. the area-recording / polygon DB —
  // append a copy of the first vertex at the end). Zero-length edges
  // between duplicates would force the bisector code to skip the
  // adjacent vertex, observed live 2026-04-27 to drop V0 from a
  // 4-corner mowing polygon and produce a 3-vertex triangle outline
  // (the X-shape user reported in the plan-preview screenshot).
  std::vector<geometry_msgs::msg::Point32> poly;
  poly.reserve(poly_in.size());
  for (const auto& p : poly_in)
  {
    if (!poly.empty() &&
        std::abs(poly.back().x - p.x) < 1e-6 &&
        std::abs(poly.back().y - p.y) < 1e-6)
    {
      continue;
    }
    poly.push_back(p);
  }
  if (poly.size() >= 2 &&
      std::abs(poly.back().x - poly.front().x) < 1e-6 &&
      std::abs(poly.back().y - poly.front().y) < 1e-6)
  {
    poly.pop_back();
  }

  const std::size_t n = poly.size();
  if (n < 3)
  {
    return poly;
  }

  // Detect winding via shoelace. signed_area > 0: CW. < 0: CCW.
  double signed_area = 0.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const std::size_t j = (i + 1) % n;
    signed_area += static_cast<double>(poly[j].x - poly[i].x) *
                   static_cast<double>(poly[j].y + poly[i].y);
  }
  const bool is_ccw = signed_area < 0.0;

  std::vector<geometry_msgs::msg::Point32> result;
  result.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    const std::size_t prev = (i + n - 1) % n;
    const std::size_t next = (i + 1) % n;

    // Edges: previous-edge points to vertex i; next-edge leaves vertex i.
    const double e1x = static_cast<double>(poly[i].x - poly[prev].x);
    const double e1y = static_cast<double>(poly[i].y - poly[prev].y);
    const double e2x = static_cast<double>(poly[next].x - poly[i].x);
    const double e2y = static_cast<double>(poly[next].y - poly[i].y);

    // Inward normal for each edge. For a CCW polygon (math convention:
    // interior on the LEFT of each edge as you traverse) the inward
    // direction is the LEFT-perpendicular: rotate (dx, dy) by +90° CCW
    // → (-dy, dx). For a CW polygon the interior is on the RIGHT, so
    // we want the RIGHT-perpendicular: (dy, -dx).
    //
    // Earlier draft had this inverted and produced an outward offset
    // polygon, which after passing through the bisector formula made
    // the outline path render as a self-intersecting bowtie (#50
    // phase 2 regression observed live 2026-04-27).
    double n1x = is_ccw ? -e1y : e1y;
    double n1y = is_ccw ?  e1x : -e1x;
    double n2x = is_ccw ? -e2y : e2y;
    double n2y = is_ccw ?  e2x : -e2x;

    const double l1 = std::hypot(n1x, n1y);
    const double l2 = std::hypot(n2x, n2y);
    if (l1 < 1e-9 || l2 < 1e-9)
    {
      continue;  // zero-length edge; skip vertex
    }
    n1x /= l1; n1y /= l1;
    n2x /= l2; n2y /= l2;

    // Bisector of the two inward normals.
    double bx = n1x + n2x;
    double by = n1y + n2y;
    const double bl = std::hypot(bx, by);
    geometry_msgs::msg::Point32 out;
    out.z = 0.0F;
    if (bl < 1e-6)
    {
      // Near-180° corner (collinear adjacent edges) — bisector degenerate.
      // Just shift along n1 by inset.
      out.x = static_cast<float>(poly[i].x + n1x * inset);
      out.y = static_cast<float>(poly[i].y + n1y * inset);
    }
    else
    {
      bx /= bl; by /= bl;
      // sin(half-interior-angle) = bisector · normal.
      const double sin_half = bx * n1x + by * n1y;
      const double dist = (std::abs(sin_half) > 1e-3)
                              ? (inset / sin_half)
                              : inset;  // very acute corner — clamp shift
      out.x = static_cast<float>(poly[i].x + bx * dist);
      out.y = static_cast<float>(poly[i].y + by * dist);
    }
    result.push_back(out);
  }
  return result;
}

}  // namespace mowgli_geometry
