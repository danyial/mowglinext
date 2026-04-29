// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Tests for dock_scan_meta_loader.{hpp,cpp} (D-17 schema, R-11, T-02-06).
//
// Four cases:
//   RoundTrip                 - save → load round-trips all 10 fields.
//   MissingField              - load returns nullopt if any required key
//                               is absent (T-02-05 visibility on partial
//                               writes — the loader rejects rather than
//                               silently filling defaults).
//   AgeExceeds_TrueAt8Days    - captured 8 days ago, threshold 7 → true.
//   AgeExceeds_FalseAt6Days   - captured 6 days ago, threshold 7 → false.
//
// The age-check uses real day arithmetic (timegm-based) per T-02-06
// mitigation; raw ISO lex compare is the documented failure mode.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include "mowgli_lidar_docking/dock_scan_meta_loader.hpp"

using mowgli_lidar_docking::dock_scan_meta_age_exceeds;
using mowgli_lidar_docking::DockScanMeta;
using mowgli_lidar_docking::load_dock_scan_meta_yaml;
using mowgli_lidar_docking::save_dock_scan_meta_yaml;

namespace
{

std::string unique_temp_yaml(const std::string& tag)
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  std::ostringstream oss;
  oss << "mowgli_dock_scan_meta_" << tag << "_" << std::hex << dist(gen)
      << ".yaml";
  return (std::filesystem::temp_directory_path() / oss.str()).string();
}

void cleanup_path(const std::string& path)
{
  std::error_code ec;
  std::filesystem::remove(path, ec);
  std::filesystem::remove(path + ".tmp", ec);
}

void write_raw(const std::string& path, const std::string& content)
{
  std::ofstream f(path);
  f << content;
}

class DockScanMetaLoaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = unique_temp_yaml("meta");
    cleanup_path(path_);
  }
  void TearDown() override { cleanup_path(path_); }
  std::string path_;
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Round-trip all 10 fields.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockScanMetaLoaderTest, RoundTrip)
{
  const DockScanMeta in{
      "/ros2_ws/maps/dock_scan.pcd",
      1.234567,    // dock_pose_x
      -2.345678,   // dock_pose_y
      0.5235988,   // dock_pose_yaw_rad ~30°
      0.180000,    // sensor_extrinsic_x
      0.000000,    // sensor_extrinsic_y
      0.000000,    // sensor_extrinsic_yaw_rad
      450,         // point_count
      "2026-04-29T15:30:00Z",
      "RTK_FIXED",
  };

  ASSERT_TRUE(save_dock_scan_meta_yaml(path_, in));
  const auto loaded = load_dock_scan_meta_yaml(path_);
  ASSERT_TRUE(loaded.has_value());

  EXPECT_EQ(loaded->dock_scan_pcd_path, in.dock_scan_pcd_path);
  EXPECT_NEAR(loaded->dock_pose_x, in.dock_pose_x, 1e-5);
  EXPECT_NEAR(loaded->dock_pose_y, in.dock_pose_y, 1e-5);
  EXPECT_NEAR(loaded->dock_pose_yaw_rad, in.dock_pose_yaw_rad, 1e-5);
  EXPECT_NEAR(loaded->sensor_extrinsic_x, in.sensor_extrinsic_x, 1e-5);
  EXPECT_NEAR(loaded->sensor_extrinsic_y, in.sensor_extrinsic_y, 1e-5);
  EXPECT_NEAR(loaded->sensor_extrinsic_yaw_rad, in.sensor_extrinsic_yaw_rad, 1e-5);
  EXPECT_EQ(loaded->point_count, in.point_count);
  EXPECT_EQ(loaded->captured_at, in.captured_at);
  EXPECT_EQ(loaded->fix_type, in.fix_type);
}

// ─────────────────────────────────────────────────────────────────────────────
// Missing `dock_scan_pcd_path` → load fails. The loader rejects partial
// files rather than filling silent defaults — operator visibility on
// corrupted writes (T-02-05 mitigation).
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockScanMetaLoaderTest, MissingField)
{
  write_raw(path_,
            // dock_scan_pcd_path intentionally omitted
            "dock_pose_x: 1.0\n"
            "dock_pose_y: 2.0\n"
            "dock_pose_yaw_rad: 0.5\n"
            "sensor_extrinsic_x: 0.18\n"
            "sensor_extrinsic_y: 0.0\n"
            "sensor_extrinsic_yaw_rad: 0.0\n"
            "point_count: 450\n"
            "captured_at: 2026-04-29T15:30:00Z\n"
            "fix_type: RTK_FIXED\n");

  const auto loaded = load_dock_scan_meta_yaml(path_);
  EXPECT_FALSE(loaded.has_value());
}

// ─────────────────────────────────────────────────────────────────────────────
// 8 days old, threshold 7 → exceeds = true.
// ─────────────────────────────────────────────────────────────────────────────
TEST(DockScanMetaAgeTest, AgeExceeds_TrueAt8Days)
{
  DockScanMeta meta{};
  meta.captured_at = "2026-04-21T10:00:00Z";
  // now is 8 days later
  EXPECT_TRUE(dock_scan_meta_age_exceeds(meta, 7, "2026-04-29T10:00:00Z"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 6 days old, threshold 7 → exceeds = false.
// ─────────────────────────────────────────────────────────────────────────────
TEST(DockScanMetaAgeTest, AgeExceeds_FalseAt6Days)
{
  DockScanMeta meta{};
  meta.captured_at = "2026-04-23T10:00:00Z";
  EXPECT_FALSE(dock_scan_meta_age_exceeds(meta, 7, "2026-04-29T10:00:00Z"));
}
