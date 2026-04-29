// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Tests for dock_scan_io.{hpp,cpp} (D-06 PCD ASCII + R-11 atomic write).
//
// Three cases:
//   PCDRoundTrip       - save_dock_scan_pcd / load_dock_scan_pcd round-trips
//                        a 10-point cloud within 1e-4 m (PCL ASCII float32
//                        truncation is the dominant error term).
//   PCDAtomicWrite     - save_dock_scan_pcd_atomic + load_dock_scan_pcd
//                        round-trips with the same precision; .tmp orphan
//                        check.
//   PCDLoadMissing     - load_dock_scan_pcd on a non-existent path returns
//                        false and leaves out_points untouched.

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "mowgli_lidar_docking/dock_scan_io.hpp"

using mowgli_lidar_docking::load_dock_scan_pcd;
using mowgli_lidar_docking::save_dock_scan_pcd;
using mowgli_lidar_docking::save_dock_scan_pcd_atomic;

namespace
{

std::string unique_temp_path(const std::string& tag)
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  std::ostringstream oss;
  oss << "mowgli_lidar_docking_" << tag << "_" << std::hex << dist(gen)
      << ".pcd";
  return (std::filesystem::temp_directory_path() / oss.str()).string();
}

void cleanup_path(const std::string& path)
{
  std::error_code ec;
  std::filesystem::remove(path, ec);
  std::filesystem::remove(path + ".tmp", ec);
}

std::vector<Eigen::Vector3d> known_points()
{
  // 10 points: corners of a 1m x 1m square at z=0.05 plus diagonals.
  return {
      {0.0, 0.0, 0.05},
      {1.0, 0.0, 0.05},
      {1.0, 1.0, 0.05},
      {0.0, 1.0, 0.05},
      {0.5, 0.5, 0.05},
      {0.25, 0.25, 0.05},
      {0.75, 0.25, 0.05},
      {0.75, 0.75, 0.05},
      {0.25, 0.75, 0.05},
      {0.5, 0.0, 0.05},
  };
}

void expect_points_close(const std::vector<Eigen::Vector3d>& a,
                         const std::vector<Eigen::Vector3d>& b)
{
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    EXPECT_NEAR(a[i].x(), b[i].x(), 1e-4) << "x mismatch at index " << i;
    EXPECT_NEAR(a[i].y(), b[i].y(), 1e-4) << "y mismatch at index " << i;
    EXPECT_NEAR(a[i].z(), b[i].z(), 1e-4) << "z mismatch at index " << i;
  }
}

class DockScanIoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = unique_temp_path("io");
    cleanup_path(path_);
  }
  void TearDown() override { cleanup_path(path_); }
  std::string path_;
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Save → load round-trip via the standard PCL save path. PCL ASCII writes
// float32 internally so 1e-4 m is the realistic precision floor.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockScanIoTest, PCDRoundTrip)
{
  const auto in = known_points();
  ASSERT_TRUE(save_dock_scan_pcd(path_, in));
  ASSERT_TRUE(std::filesystem::exists(path_));

  std::vector<Eigen::Vector3d> out;
  ASSERT_TRUE(load_dock_scan_pcd(path_, out));
  expect_points_close(in, out);
}

// ─────────────────────────────────────────────────────────────────────────────
// Atomic save: temp + fsync + rename + dir fsync. Confirms the .tmp
// orphan is removed by atomic_write on success (R-11 power-loss-safe).
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockScanIoTest, PCDAtomicWrite)
{
  const auto in = known_points();
  ASSERT_TRUE(save_dock_scan_pcd_atomic(path_, in));
  ASSERT_TRUE(std::filesystem::exists(path_));

  // No .tmp orphan — atomic_write must rename, not copy.
  EXPECT_FALSE(std::filesystem::exists(path_ + ".tmp"))
      << "save_dock_scan_pcd_atomic must rename the .tmp file, not leave it";

  std::vector<Eigen::Vector3d> out;
  ASSERT_TRUE(load_dock_scan_pcd(path_, out));
  expect_points_close(in, out);
}

// ─────────────────────────────────────────────────────────────────────────────
// Load on a non-existent path returns false; out_points unchanged.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockScanIoTest, PCDLoadMissing)
{
  std::vector<Eigen::Vector3d> out;
  out.emplace_back(7.7, 8.8, 9.9);  // sentinel — must survive

  EXPECT_FALSE(load_dock_scan_pcd("/nonexistent/path/dock_scan.pcd", out));

  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].x(), 7.7);
  EXPECT_DOUBLE_EQ(out[0].y(), 8.8);
  EXPECT_DOUBLE_EQ(out[0].z(), 9.9);
}
