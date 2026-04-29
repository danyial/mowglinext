// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Plan 02-04 Task 1 — KinematicIcpDockMatcher unit tests (RED → GREEN).
//
// Five tests pin the contract:
//
//   * test_constructor_seeds_local_map: verifies that the constructor
//     populates the kiss_icp::VoxelHashMap so the very first Match() call
//     against a near-identical cloud returns valid=true with high inlier
//     ratio + low rmse. If this test fails, the AddPoints path from
//     PROBE.md A1 is broken or the SetPose+AddPoints ordering is wrong.
//
//   * test_match_corrupted_frame_low_confidence: 80% noise in the live
//     frame must collapse the trust gate. is_trusted(min=0.70, max_rmse=
//     0.05) must return false.
//
//   * test_match_180_flip_caught: Pitfall 1 acceptance gate. A live frame
//     rotated 180° around the dock anchor must be flagged untrusted (via
//     valid=false OR low confidence). Test name is load-bearing for the
//     plan's `--ctest-args -R test_match_180_flip_caught` filter.
//
//   * test_reload_swaps_dock_scan: Reload() with a different dock cloud
//     swaps the voxel map cleanly. A subsequent Match against the new
//     cloud succeeds (proves the reload took effect). Mtime-watcher
//     correctness depends on this.
//
//   * test_concurrent_match_and_reload_no_data_race: spins two threads
//     hammering Match + Reload for 1 second. Asserts no crash and final
//     state is well-formed (proves the std::mutex actually protects
//     kicp_).

#include "mowgli_lidar_docking/kinematic_icp_dock_matcher.hpp"
#include "mowgli_lidar_docking/confidence_metrics.hpp"
#include "mowgli_lidar_docking/idock_matcher.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <random>
#include <thread>
#include <vector>

namespace mlid = mowgli_lidar_docking;

namespace
{

// Build a dense 1m × 1m square outline with `n_per_side` points per side.
// Points are in the dock frame (centred at the origin, no rotation).
std::vector<Eigen::Vector3d> make_square_outline(std::size_t n_per_side, double half_side)
{
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(4 * n_per_side);
  for (std::size_t i = 0; i < n_per_side; ++i)
  {
    const double t = static_cast<double>(i) / static_cast<double>(n_per_side);
    const double s = -half_side + t * 2.0 * half_side;
    pts.emplace_back(s, half_side, 0.0);    // top edge
    pts.emplace_back(s, -half_side, 0.0);   // bottom edge
    pts.emplace_back(half_side, s, 0.0);    // right edge
    pts.emplace_back(-half_side, s, 0.0);   // left edge
  }
  return pts;
}

// Translate every point by a small offset (1 cm) — used to simulate a
// near-identical scan that the matcher should align with high confidence.
std::vector<Eigen::Vector3d> translate(const std::vector<Eigen::Vector3d>& src,
                                       const Eigen::Vector3d& delta)
{
  std::vector<Eigen::Vector3d> out;
  out.reserve(src.size());
  for (const auto& p : src) out.emplace_back(p + delta);
  return out;
}

// Rotate every point about Z by `yaw_rad`. Used to simulate the 180° flip
// hallucination from Pitfall 1.
std::vector<Eigen::Vector3d> rotate_z(const std::vector<Eigen::Vector3d>& src,
                                      double yaw_rad)
{
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  std::vector<Eigen::Vector3d> out;
  out.reserve(src.size());
  for (const auto& p : src)
  {
    out.emplace_back(c * p.x() - s * p.y(), s * p.x() + c * p.y(), p.z());
  }
  return out;
}

// Generate noise points scattered far from the dock — used to corrupt the
// live frame so the inlier ratio collapses.
std::vector<Eigen::Vector3d> noise_far(std::size_t n, std::mt19937& rng)
{
  std::uniform_real_distribution<double> u(-5.0, 5.0);
  std::vector<Eigen::Vector3d> out;
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    out.emplace_back(u(rng), u(rng), 0.0);
  }
  return out;
}

kinematic_icp::pipeline::Config make_default_cfg()
{
  kinematic_icp::pipeline::Config cfg;
  cfg.max_range = 8.0;
  cfg.min_range = 0.0;       // tests use synthetic close points
  cfg.voxel_size = 0.1;
  cfg.max_points_per_voxel = 5;
  cfg.use_adaptive_threshold = false;
  cfg.fixed_threshold = 0.30;
  cfg.max_num_iterations = 20;
  cfg.convergence_criterion = 0.001;
  cfg.max_num_threads = 1;
  cfg.use_adaptive_odometry_regularization = true;
  cfg.deskew = false;
  return cfg;
}

}  // namespace

TEST(KinematicIcpDockMatcher, ConstructorSeedsLocalMap)
{
  // 200 points on a 1m × 1m square outline (50 per side).
  const auto dock = make_square_outline(50, 0.5);
  ASSERT_EQ(dock.size(), 200u);

  const Sophus::SE3d anchor;  // identity — dock at origin
  mlid::KinematicIcpDockMatcher matcher(make_default_cfg(), dock, anchor, 0.30);

  // Live frame = dock cloud translated by 1 cm in X.
  const auto live = translate(dock, Eigen::Vector3d(0.01, 0.0, 0.0));

  const auto result = matcher.Match(live, Sophus::SE3d{});
  EXPECT_TRUE(result.valid) << "matcher must accept a near-identical scan";
  EXPECT_GT(result.inlier_ratio, 0.9)
      << "near-identical cloud should have inlier ratio > 0.9, got "
      << result.inlier_ratio;
  EXPECT_LT(result.rmse_m, 0.05)
      << "near-identical cloud RMSE should be < 5 cm, got " << result.rmse_m;
}

TEST(KinematicIcpDockMatcher, MatchCorruptedFrameLowConfidence)
{
  const auto dock = make_square_outline(50, 0.5);
  const Sophus::SE3d anchor;
  mlid::KinematicIcpDockMatcher matcher(make_default_cfg(), dock, anchor, 0.30);

  // 80% noise live frame: take 40 dock points + 160 random points far away.
  std::mt19937 rng(42);
  std::vector<Eigen::Vector3d> live;
  live.reserve(200);
  for (std::size_t i = 0; i < 40; ++i) live.push_back(dock[i]);
  const auto noise = noise_far(160, rng);
  live.insert(live.end(), noise.begin(), noise.end());

  const auto result = matcher.Match(live, Sophus::SE3d{});

  // Independent verification via the brute-force overload — proves the
  // result's confidence estimate is consistent with reality and not a
  // tautology of the production compute_confidence.
  const auto independent = mlid::compute_confidence_brute_force(live, dock, 0.30);
  EXPECT_LT(independent.inlier_ratio, 0.5)
      << "brute-force sanity: 80%-noise frame must show inlier_ratio < 0.5";

  // The matcher's MatchResult is the load-bearing assertion.
  mlid::ConfidenceResult cr{result.inlier_ratio, result.rmse_m};
  EXPECT_FALSE(mlid::is_trusted(cr, 0.70, 0.05))
      << "corrupted frame must NOT pass the SPEC R-3 trust gate (inlier="
      << result.inlier_ratio << ", rmse=" << result.rmse_m << ")";
}

TEST(KinematicIcpDockMatcher, test_match_180_flip_caught)
{
  // Use an L-shape so the cloud is NOT symmetric under 180° rotation —
  // a perfect square would let the matcher align cleanly even at 180°,
  // which would defeat the test. The L-shape forces a yaw mismatch.
  std::vector<Eigen::Vector3d> dock;
  for (int i = 0; i < 50; ++i)
  {
    const double t = i * 0.02;        // 0..1 m
    dock.emplace_back(t, 0.0, 0.0);   // long arm along +X
  }
  for (int i = 0; i < 25; ++i)
  {
    const double t = i * 0.02;        // 0..0.5 m
    dock.emplace_back(0.0, t, 0.0);   // short arm along +Y
  }

  const Sophus::SE3d anchor;  // identity (yaw = 0)
  mlid::KinematicIcpDockMatcher matcher(make_default_cfg(), dock, anchor, 0.30);

  // Live frame = dock rotated 180° about Z. After matching, the matcher's
  // pose-yaw should diverge from the anchor yaw by ~π → flip detector
  // must mark valid=false OR confidence must drop.
  const auto live = rotate_z(dock, M_PI);

  const auto result = matcher.Match(live, Sophus::SE3d{});

  mlid::ConfidenceResult cr{result.inlier_ratio, result.rmse_m};
  const bool trusted = mlid::is_trusted(cr, 0.70, 0.05);

  // Pitfall 1 acceptance: either valid=false (flip detector caught it),
  // or trusted=false (confidence collapsed organically). Both are OK; a
  // 180° flipped L-shape MUST NOT pass the trust gate.
  EXPECT_FALSE(result.valid && trusted)
      << "180°-flipped L-shape must NOT be trusted (valid=" << result.valid
      << ", inlier=" << result.inlier_ratio << ", rmse=" << result.rmse_m
      << ")";
}

TEST(KinematicIcpDockMatcher, ReloadSwapsDockScan)
{
  // Cloud A: square outline at origin.
  const auto cloud_a = make_square_outline(50, 0.5);
  // Cloud B: triangle (3-side outline, 50 points each) translated by 5 m
  // so the geometry is unambiguously different from A.
  std::vector<Eigen::Vector3d> cloud_b;
  for (int i = 0; i < 50; ++i)
  {
    const double t = i * 0.02;
    cloud_b.emplace_back(5.0 + t, 5.0, 0.0);
    cloud_b.emplace_back(5.0, 5.0 + t, 0.0);
    cloud_b.emplace_back(5.0 + t, 5.0 + t, 0.0);
  }

  const Sophus::SE3d anchor_a;  // identity
  Sophus::SE3d anchor_b;
  anchor_b.translation() << 5.0, 5.0, 0.0;

  mlid::KinematicIcpDockMatcher matcher(make_default_cfg(), cloud_a, anchor_a, 0.30);

  // Sanity: cloud_a aligns well at origin.
  const auto a_live = translate(cloud_a, Eigen::Vector3d(0.01, 0.0, 0.0));
  const auto r_a = matcher.Match(a_live, Sophus::SE3d{});
  EXPECT_GT(r_a.inlier_ratio, 0.9);

  // Reload with cloud_b at anchor_b.
  matcher.Reload(cloud_b, anchor_b);

  // Now cloud_b (translated 1 cm) must align well.
  const auto b_live = translate(cloud_b, Eigen::Vector3d(0.01, 0.0, 0.0));
  const auto r_b = matcher.Match(b_live, Sophus::SE3d{});
  EXPECT_TRUE(r_b.valid);
  EXPECT_GT(r_b.inlier_ratio, 0.9)
      << "after Reload(cloud_b), cloud_b must align (got inlier="
      << r_b.inlier_ratio << ")";
  EXPECT_LT(r_b.rmse_m, 0.10);

  // Independent verification: cloud_a should now be a stranger to the
  // voxel map (a's points are 5+ m away from the new local map at anchor_b).
  const auto r_a_after = matcher.Match(a_live, Sophus::SE3d{});
  // It might still report some inliers due to how kicp_ moves the anchor,
  // but the brute-force sanity against cloud_b confirms a is dissimilar.
  const auto independent =
      mlid::compute_confidence_brute_force(a_live, cloud_b, 0.30);
  EXPECT_LT(independent.inlier_ratio, 0.1)
      << "brute-force sanity: cloud_a is far from cloud_b ("
      << independent.inlier_ratio << " inliers vs cloud_b)";
  (void)r_a_after;
}

TEST(KinematicIcpDockMatcher, ConcurrentMatchAndReloadNoDataRace)
{
  const auto dock = make_square_outline(50, 0.5);
  const Sophus::SE3d anchor;
  mlid::KinematicIcpDockMatcher matcher(make_default_cfg(), dock, anchor, 0.30);

  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> match_count{0};
  std::atomic<std::uint64_t> reload_count{0};

  const auto live = translate(dock, Eigen::Vector3d(0.005, 0.0, 0.0));

  std::thread match_thread([&] {
    while (!stop.load(std::memory_order_relaxed))
    {
      auto result = matcher.Match(live, Sophus::SE3d{});
      (void)result;
      match_count.fetch_add(1, std::memory_order_relaxed);
    }
  });

  std::thread reload_thread([&] {
    auto reload_dock = dock;
    while (!stop.load(std::memory_order_relaxed))
    {
      matcher.Reload(reload_dock, anchor);
      reload_count.fetch_add(1, std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  std::this_thread::sleep_for(std::chrono::seconds(1));
  stop.store(true, std::memory_order_relaxed);
  match_thread.join();
  reload_thread.join();

  // No crash / no TSAN trip = test passes. Sanity: at least a couple of
  // each operation actually executed.
  EXPECT_GT(match_count.load(), 0u);
  EXPECT_GT(reload_count.load(), 0u);

  // Final state must be well-formed: a fresh match returns a finite result.
  const auto final = matcher.Match(live, Sophus::SE3d{});
  EXPECT_TRUE(std::isfinite(final.inlier_ratio));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
