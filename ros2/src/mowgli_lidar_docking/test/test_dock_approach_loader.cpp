// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Tests for dock_approach_loader.{hpp,cpp} (D-04 schema, T-02-01).
//
// Three cases:
//   RoundTrip      - save → load round-trips all 5 fields (1e-5 m for
//                    floats, exact match for strings).
//   InvalidSource  - a manually-written yaml with `source: garbage` is
//                    rejected (T-02-01 mitigation).
//   MissingField   - a yaml without `dock_approach_yaw_to_dock_rad` is
//                    rejected as a whole (any missing required key →
//                    nullopt).

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include "mowgli_lidar_docking/dock_approach_loader.hpp"

using mowgli_lidar_docking::DockApproach;
using mowgli_lidar_docking::load_dock_approach_yaml;
using mowgli_lidar_docking::save_dock_approach_yaml;

namespace
{

std::string unique_temp_yaml(const std::string& tag)
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  std::ostringstream oss;
  oss << "mowgli_dock_approach_" << tag << "_" << std::hex << dist(gen)
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

class DockApproachLoaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = unique_temp_yaml("loader");
    cleanup_path(path_);
  }
  void TearDown() override { cleanup_path(path_); }
  std::string path_;
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Save → load round-trips all 5 fields. Floats compared at 1e-5 m / rad
// (writer uses std::fixed << std::setprecision(6) so 1e-5 has slack).
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockApproachLoaderTest, RoundTrip)
{
  const DockApproach in{
      1.234567,
      -2.345678,
      0.5235988,  // ~30°
      "lidar",
      "2026-04-29T15:30:00Z",
  };

  ASSERT_TRUE(save_dock_approach_yaml(path_, in));
  const auto loaded = load_dock_approach_yaml(path_);
  ASSERT_TRUE(loaded.has_value());

  EXPECT_NEAR(loaded->x, in.x, 1e-5);
  EXPECT_NEAR(loaded->y, in.y, 1e-5);
  EXPECT_NEAR(loaded->yaw_to_dock_rad, in.yaw_to_dock_rad, 1e-5);
  EXPECT_EQ(loaded->source, in.source);
  EXPECT_EQ(loaded->captured_at, in.captured_at);
}

// ─────────────────────────────────────────────────────────────────────────────
// `source: garbage` is rejected (T-02-01 mitigation). The loader does NOT
// fall back to "lidar" or "tf" — invalid input is a load failure.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockApproachLoaderTest, InvalidSource)
{
  write_raw(path_,
            "dock_approach_x: 1.0\n"
            "dock_approach_y: 2.0\n"
            "dock_approach_yaw_to_dock_rad: 0.5\n"
            "source: garbage\n"
            "captured_at: 2026-04-29T15:30:00Z\n");

  const auto loaded = load_dock_approach_yaml(path_);
  EXPECT_FALSE(loaded.has_value())
      << "load_dock_approach_yaml accepted source=garbage; T-02-01 broken";
}

// ─────────────────────────────────────────────────────────────────────────────
// Missing required field → nullopt. The whole file is rejected (no
// silent fallback to default values).
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockApproachLoaderTest, MissingField)
{
  write_raw(path_,
            "dock_approach_x: 1.0\n"
            "dock_approach_y: 2.0\n"
            // dock_approach_yaw_to_dock_rad intentionally omitted
            "source: lidar\n"
            "captured_at: 2026-04-29T15:30:00Z\n");

  const auto loaded = load_dock_approach_yaml(path_);
  EXPECT_FALSE(loaded.has_value());
}
