// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Tests for idock_matcher.hpp (D-16 abstract base contract).
//
// Single case: define a MockMatcher : public IDockMatcher inline in this
// test, populate a fixed MatchResult, and verify it is returned verbatim
// from a Match call. This proves:
//   * the abstract base is concretely-derivable from a unit-test seat;
//   * the MatchResult struct's defaults play nicely with brace-init;
//   * Plan 02-06 BT tests can reuse this MockMatcher pattern locally
//     without promoting it to a public header (the duplication cost is
//     ~10 lines per test file — cheap).
//
// If duplication of MockMatcher across BT tests becomes painful in
// Plan 02-06, a future plan may promote it to
// `include/mowgli_lidar_docking/test_doubles/mock_dock_matcher.hpp`
// (decision deferred to Plan 02-06's executor).

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <sophus/se3.hpp>

#include <vector>

#include "mowgli_lidar_docking/idock_matcher.hpp"

using mowgli_lidar_docking::IDockMatcher;
using mowgli_lidar_docking::MatchResult;

namespace
{

/// Test double — returns whatever MatchResult is preset on it. Mirrors
/// the pattern Plan 02-06's BT tests will use to assert trust-gating
/// without standing up a real kiss_icp::VoxelHashMap.
class MockMatcher : public IDockMatcher
{
public:
  MatchResult Match(const std::vector<Eigen::Vector3d>& live_frame,
                    const Sophus::SE3d& lidar_to_base) override
  {
    (void)live_frame;
    (void)lidar_to_base;
    ++call_count_;
    return preset_;
  }

  MatchResult preset_{};
  int call_count_{0};
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// MockMatcher returns the operator-supplied fixed MatchResult unchanged.
// Demonstrates that IDockMatcher is implementable via standard polymorphism
// (D-16 acceptance: "MockDockMatcher returns operator-supplied fixed pose
// + confidence").
// ─────────────────────────────────────────────────────────────────────────────
TEST(IDockMatcherTest, MockReturnsFixedResult)
{
  MockMatcher mock;
  mock.preset_.pose_in_map = Sophus::SE3d{};  // identity
  mock.preset_.inlier_ratio = 0.85;
  mock.preset_.rmse_m = 0.02;
  mock.preset_.valid = true;

  // Arbitrary inputs — MockMatcher ignores them.
  const std::vector<Eigen::Vector3d> live_frame{
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
  };
  const Sophus::SE3d lidar_to_base{};

  // Use the abstract base pointer to confirm IDockMatcher is the
  // dispatch surface (not the derived class directly).
  IDockMatcher* iface = &mock;
  const MatchResult r = iface->Match(live_frame, lidar_to_base);

  EXPECT_DOUBLE_EQ(r.inlier_ratio, 0.85);
  EXPECT_DOUBLE_EQ(r.rmse_m, 0.02);
  EXPECT_TRUE(r.valid);
  // pose_in_map is identity — translation should be zero.
  EXPECT_NEAR(r.pose_in_map.translation().norm(), 0.0, 1e-12);
  // Rotation is identity → quaternion w=1 (or w=-1; |w|=1 either way).
  EXPECT_NEAR(std::abs(r.pose_in_map.unit_quaternion().w()), 1.0, 1e-12);

  EXPECT_EQ(mock.call_count_, 1);
}
