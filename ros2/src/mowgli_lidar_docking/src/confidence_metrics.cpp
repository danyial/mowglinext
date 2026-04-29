// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Implementation notes:
//
//   * PROBE.md A1+A2 confirmed: kiss_icp::VoxelHashMap is a struct with
//     default-public access; AddPoints + GetClosestNeighbor are usable
//     directly. We take the production code path with no fallback —
//     no inline voxel-bucket reimplementation.
//
//   * GetClosestNeighbor returns `std::tuple<Eigen::Vector3d, double>`
//     where the second element is the squared distance to the nearest
//     neighbour (per kiss_icp v1.2.0 source confirmed in PROBE.md
//     "Verbatim declaration"). We use that squared distance directly
//     instead of recomputing (p-nn).squaredNorm() — same answer,
//     fewer flops, and immune to a hypothetical future refactor that
//     might change which neighbour is returned versus which one is
//     used internally for the squared-distance bookkeeping.
//
//   * The brute-force overload is independent — no kiss_icp include
//     needed in that path. Used by every gtest in this package so the
//     test harness doesn't need to stand up a VoxelHashMap.

#include "mowgli_lidar_docking/confidence_metrics.hpp"

#include <kiss_icp/core/VoxelHashMap.hpp>

#include <cmath>
#include <limits>
#include <tuple>

namespace mowgli_lidar_docking
{

ConfidenceResult compute_confidence(
    const std::vector<Eigen::Vector3d>& frame,
    const kiss_icp::VoxelHashMap& voxel_map,
    double max_d)
{
  if (frame.empty())
  {
    return {0.0, std::numeric_limits<double>::infinity()};
  }

  std::size_t inliers = 0;
  double sse = 0.0;
  const double max_d2 = max_d * max_d;

  for (const auto& p : frame)
  {
    // PROBE.md A2: signature is
    //   std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(
    //       const Eigen::Vector3d& query) const;
    // The returned `double` is the squared distance, not the linear
    // distance — kiss_icp internally compares against the squared max
    // correspondence distance for the same reason we do here.
    const auto [neighbor, d2] = voxel_map.GetClosestNeighbor(p);
    (void)neighbor;  // we only need the distance for confidence math

    if (d2 <= max_d2)
    {
      ++inliers;
      sse += d2;
    }
  }

  if (inliers == 0)
  {
    return {0.0, std::numeric_limits<double>::infinity()};
  }

  const double ratio =
      static_cast<double>(inliers) / static_cast<double>(frame.size());
  const double rmse = std::sqrt(sse / static_cast<double>(inliers));
  return {ratio, rmse};
}

ConfidenceResult compute_confidence_brute_force(
    const std::vector<Eigen::Vector3d>& frame,
    const std::vector<Eigen::Vector3d>& reference_cloud,
    double max_d)
{
  if (frame.empty())
  {
    return {0.0, std::numeric_limits<double>::infinity()};
  }

  std::size_t inliers = 0;
  double sse = 0.0;
  const double max_d2 = max_d * max_d;

  for (const auto& p : frame)
  {
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto& q : reference_cloud)
    {
      const double d2 = (p - q).squaredNorm();
      if (d2 < best_d2) best_d2 = d2;
    }
    if (best_d2 <= max_d2)
    {
      ++inliers;
      sse += best_d2;
    }
  }

  if (inliers == 0)
  {
    return {0.0, std::numeric_limits<double>::infinity()};
  }

  const double ratio =
      static_cast<double>(inliers) / static_cast<double>(frame.size());
  const double rmse = std::sqrt(sse / static_cast<double>(inliers));
  return {ratio, rmse};
}

bool is_trusted(const ConfidenceResult& r,
                double min_inlier_ratio,
                double max_rmse_m)
{
  return std::isfinite(r.rmse_m) &&
         r.inlier_ratio >= min_inlier_ratio &&
         r.rmse_m <= max_rmse_m;
}

}  // namespace mowgli_lidar_docking
