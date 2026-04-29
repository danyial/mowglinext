// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// Eigen3 PCA principal-axis derivation for SPECIAL_PATTERN narrow-area
// centerlines (D-10). Returns the major-eigenvector direction; falls back
// to the longest-edge angle when the polygon is degenerate (rank-1 cov).

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <geometry_msgs/msg/point32.hpp>

namespace mowgli_geometry
{

namespace detail
{

/// Longest-edge fallback: return the atan2 angle of the longest edge of a
/// closed polygon traversal. Used when the PCA covariance is rank-1 (every
/// vertex on a line) so SelfAdjointEigenSolver may produce an ambiguous
/// result for the principal direction.
inline double longest_edge_angle(
    const std::vector<geometry_msgs::msg::Point32>& vertices)
{
  const std::size_t n = vertices.size();
  double best_len2 = 0.0;
  double best_angle = 0.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const std::size_t j = (i + 1) % n;
    const double dx =
        static_cast<double>(vertices[j].x) - static_cast<double>(vertices[i].x);
    const double dy =
        static_cast<double>(vertices[j].y) - static_cast<double>(vertices[i].y);
    const double len2 = dx * dx + dy * dy;
    if (len2 > best_len2)
    {
      best_len2 = len2;
      best_angle = std::atan2(dy, dx);
    }
  }
  return best_angle;
}

}  // namespace detail

/// Principal-axis angle (rad) of a polygon's vertices via PCA.
/// - Eigen3 SelfAdjointEigenSolver<Matrix2d> on the 2×2 covariance.
/// - Eigenvalues sorted ascending; column 1 = principal axis.
/// - Degenerate guard: if smallest eigenvalue < 1e-9 (collinear vertices,
///   rank-1 covariance), falls back to the longest-edge atan2.
/// - Returns 0.0 for polygons with fewer than 3 vertices.
inline double pca_principal_axis(
    const std::vector<geometry_msgs::msg::Point32>& vertices)
{
  const std::size_t n = vertices.size();
  if (n < 3)
  {
    return 0.0;
  }

  // Build N×2 matrix of vertex coordinates.
  Eigen::MatrixXd pts(n, 2);
  for (std::size_t i = 0; i < n; ++i)
  {
    pts(static_cast<Eigen::Index>(i), 0) = static_cast<double>(vertices[i].x);
    pts(static_cast<Eigen::Index>(i), 1) = static_cast<double>(vertices[i].y);
  }

  // Centroid + centred coordinates.
  Eigen::Vector2d centroid = pts.colwise().mean();
  Eigen::MatrixXd centered = pts.rowwise() - centroid.transpose();

  // 2×2 covariance matrix (sample, n-1 normalisation).
  Eigen::Matrix2d cov =
      (centered.transpose() * centered) / static_cast<double>(n - 1);

  // Eigendecomposition. SelfAdjointEigenSolver returns eigenvalues sorted
  // ascending → eigenvectors().col(1) is the principal axis.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
  if (solver.info() != Eigen::Success)
  {
    return detail::longest_edge_angle(vertices);
  }

  // Degenerate-rank guard: if the smaller eigenvalue is essentially zero
  // (rank-1 covariance from collinear vertices), the principal direction
  // is along that line — but we get it more robustly from the longest
  // edge of the polygon traversal (RESEARCH §3.4 + Assumption A3).
  const double smallest_eigenvalue = solver.eigenvalues()(0);
  if (smallest_eigenvalue < 1e-9)
  {
    return detail::longest_edge_angle(vertices);
  }

  Eigen::Vector2d axis = solver.eigenvectors().col(1);
  return std::atan2(axis.y(), axis.x());
}

}  // namespace mowgli_geometry
