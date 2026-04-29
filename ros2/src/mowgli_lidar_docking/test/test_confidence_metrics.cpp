// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Tests for confidence_metrics.{hpp,cpp} (SPEC R-3 trust gate).
//
// Four cases — all use the brute-force overload so the test harness does
// not have to stand up a kiss_icp::VoxelHashMap. The production overload
// shares the same SSE/inlier math with the brute-force path, so this is
// a representative sample of the contract.
//
// Cases:
//   HighConfidenceOnIdenticalCloud - registered_frame == reference_cloud:
//                                    inlier_ratio≈1.0, rmse≈0, trusted=true.
//   LowConfidenceOnNoise           - 80% replaced with random offsets >1m:
//                                    inlier_ratio<0.30, trusted=false (R-3
//                                    "trusted=false within 1 s of corruption"
//                                    substrate test).
//   FlippedYawHallucination        - L-shape rotated 180°: scarce overlap →
//                                    not trusted (RESEARCH §Pitfall 1
//                                    documentation case for the metric).
//   EmptyFrame                     - frame={}: inlier_ratio=0, rmse=+inf,
//                                    !trusted, no division-by-zero.
//
// All randomness uses a fixed mt19937 seed so test outcomes are
// deterministic across CI runs (CLAUDE.md rule).

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "mowgli_lidar_docking/confidence_metrics.hpp"

using mowgli_lidar_docking::compute_confidence_brute_force;
using mowgli_lidar_docking::ConfidenceResult;
using mowgli_lidar_docking::is_trusted;

namespace
{

/// Build an L-shape outline of ~200 points on the perimeter of a
/// non-symmetric L (avoids 180°-flip self-aliasing — a square would
/// match itself perfectly under a 180° rotation).
std::vector<Eigen::Vector3d> l_shape_outline()
{
  std::vector<Eigen::Vector3d> pts;
  // Bottom edge from (0,0) to (2,0)
  for (int i = 0; i < 50; ++i)
  {
    pts.emplace_back(0.04 * i, 0.0, 0.0);
  }
  // Right vertical from (2,0) to (2,1)
  for (int i = 0; i < 25; ++i)
  {
    pts.emplace_back(2.0, 0.04 * i, 0.0);
  }
  // Inner horizontal back from (2,1) to (1,1)
  for (int i = 0; i < 25; ++i)
  {
    pts.emplace_back(2.0 - 0.04 * i, 1.0, 0.0);
  }
  // Inner vertical up from (1,1) to (1,2)
  for (int i = 0; i < 25; ++i)
  {
    pts.emplace_back(1.0, 1.0 + 0.04 * i, 0.0);
  }
  // Top edge from (1,2) to (0,2)
  for (int i = 0; i < 25; ++i)
  {
    pts.emplace_back(1.0 - 0.04 * i, 2.0, 0.0);
  }
  // Left vertical down from (0,2) to (0,0)
  for (int i = 0; i < 50; ++i)
  {
    pts.emplace_back(0.0, 2.0 - 0.04 * i, 0.0);
  }
  return pts;
}

/// Square outline (200 points). Symmetric — used for the
/// HighConfidenceOnIdenticalCloud case where the cloud matches itself.
std::vector<Eigen::Vector3d> square_outline()
{
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i < 50; ++i)
  {
    pts.emplace_back(0.02 * i, 0.0, 0.0);
    pts.emplace_back(1.0, 0.02 * i, 0.0);
    pts.emplace_back(1.0 - 0.02 * i, 1.0, 0.0);
    pts.emplace_back(0.0, 1.0 - 0.02 * i, 0.0);
  }
  return pts;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// frame == reference: inlier_ratio≈1.0, rmse≈0, trusted=true.
// ─────────────────────────────────────────────────────────────────────────────
TEST(ConfidenceMetricsTest, HighConfidenceOnIdenticalCloud)
{
  const auto cloud = square_outline();
  const auto r = compute_confidence_brute_force(cloud, cloud, 0.05);

  EXPECT_GE(r.inlier_ratio, 0.95);
  EXPECT_LE(r.rmse_m, 1e-6);
  EXPECT_TRUE(is_trusted(r, 0.70, 0.05));
}

// ─────────────────────────────────────────────────────────────────────────────
// Replace 80% of frame points with random offsets >1m: inlier_ratio<0.30,
// trusted=false. Substrate for SPEC R-3 "trusted=false within 1 s".
// ─────────────────────────────────────────────────────────────────────────────
TEST(ConfidenceMetricsTest, LowConfidenceOnNoise)
{
  const auto reference = square_outline();
  std::vector<Eigen::Vector3d> noisy = reference;

  std::mt19937 gen(42);  // fixed seed → deterministic CI
  std::uniform_real_distribution<double> off(2.0, 10.0);

  // Replace 80% of points with random faraway points (still distinct
  // from any reference point).
  const std::size_t n_to_replace = (noisy.size() * 4) / 5;
  for (std::size_t i = 0; i < n_to_replace; ++i)
  {
    noisy[i] = Eigen::Vector3d(off(gen), off(gen), 0.0);
  }

  const auto r = compute_confidence_brute_force(noisy, reference, 0.05);

  EXPECT_LT(r.inlier_ratio, 0.30);
  EXPECT_FALSE(is_trusted(r, 0.70, 0.05));
}

// ─────────────────────────────────────────────────────────────────────────────
// 180° rotation of an L-shape: most points land >5cm from any reference
// point, so inlier_ratio is low and the result is not trusted. This
// documents the metric's behaviour on the Pitfall 1 hallucination case;
// the matcher's pose-correction logic against this is asserted in Plan
// 02-04's integration test.
// ─────────────────────────────────────────────────────────────────────────────
TEST(ConfidenceMetricsTest, FlippedYawHallucination)
{
  const auto reference = l_shape_outline();

  // Rotate 180° about the centroid of the L (~(0.83, 0.83)).
  const double cx = 0.83;
  const double cy = 0.83;
  std::vector<Eigen::Vector3d> flipped;
  flipped.reserve(reference.size());
  for (const auto& p : reference)
  {
    flipped.emplace_back(2.0 * cx - p.x(), 2.0 * cy - p.y(), p.z());
  }

  const auto r = compute_confidence_brute_force(flipped, reference, 0.05);

  // Some corner points may incidentally align (the L is not point-symmetric
  // about its centroid, but a few flipped points still happen to land
  // within 5 cm of an original). The contract is that it's not trusted.
  EXPECT_FALSE(is_trusted(r, 0.70, 0.05))
      << "180°-flipped L-shape returned trusted=true (inlier_ratio="
      << r.inlier_ratio << ", rmse=" << r.rmse_m << ")";
}

// ─────────────────────────────────────────────────────────────────────────────
// Empty registered_frame: inlier_ratio=0, rmse=+inf, !trusted.
// No division-by-zero, no NaN.
// ─────────────────────────────────────────────────────────────────────────────
TEST(ConfidenceMetricsTest, EmptyFrame)
{
  const auto reference = square_outline();
  const std::vector<Eigen::Vector3d> empty;

  const auto r = compute_confidence_brute_force(empty, reference, 0.05);

  EXPECT_DOUBLE_EQ(r.inlier_ratio, 0.0);
  EXPECT_TRUE(std::isinf(r.rmse_m));
  EXPECT_FALSE(is_trusted(r, 0.70, 0.05));
}
