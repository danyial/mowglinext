// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// 10 unit tests pinning the contract of mowgli_geometry::key_value_parser.
// Plan 02-01 Task 3, behaviour spec from the plan's <behavior> block.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "mowgli_geometry/key_value_parser.hpp"

using mowgli_geometry::DockCalibrationFile;
using mowgli_geometry::load_dock_calibration_file;
using mowgli_geometry::parse_yaml_double;
using mowgli_geometry::parse_yaml_int;
using mowgli_geometry::parse_yaml_string;

namespace
{

std::string unique_temp_path()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  std::uniform_int_distribution<std::uint64_t> dist;

  const auto base = std::filesystem::temp_directory_path();
  std::ostringstream oss;
  oss << "mowgli_kvp_test_" << std::hex << dist(gen) << ".yaml";
  return (base / oss.str()).string();
}

void cleanup_path(const std::string& path)
{
  std::error_code ec;
  std::filesystem::remove(path, ec);
}

void write_file(const std::string& path, const std::string& content)
{
  std::ofstream f(path, std::ios::binary);
  f << content;
}

class DockCalFileTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = unique_temp_path();
    cleanup_path(path_);
  }
  void TearDown() override { cleanup_path(path_); }
  std::string path_;
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Test 1 — parse_yaml_double happy path.
// ─────────────────────────────────────────────────────────────────────────────
TEST(KeyValueParser, ParseDoubleHappyPath)
{
  const auto v = parse_yaml_double("key: 3.14\n", "key");
  ASSERT_TRUE(v.has_value());
  EXPECT_DOUBLE_EQ(*v, 3.14);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2 — parse_yaml_double missing key.
// ─────────────────────────────────────────────────────────────────────────────
TEST(KeyValueParser, ParseDoubleMissingKey)
{
  const auto v = parse_yaml_double("other: 1.0\n", "key");
  EXPECT_FALSE(v.has_value());
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3 — parse_yaml_double bad value.
// ─────────────────────────────────────────────────────────────────────────────
TEST(KeyValueParser, ParseDoubleBadValue)
{
  const auto v = parse_yaml_double("key: not_a_number\n", "key");
  EXPECT_FALSE(v.has_value());
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4 — parse_yaml_double trims leading whitespace + handles negatives.
// ─────────────────────────────────────────────────────────────────────────────
TEST(KeyValueParser, ParseDoubleLeadingWhitespaceNegative)
{
  const auto v = parse_yaml_double("key:    -2.5\n", "key");
  ASSERT_TRUE(v.has_value());
  EXPECT_DOUBLE_EQ(*v, -2.5);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5 — parse_yaml_string happy path (bare token).
// ─────────────────────────────────────────────────────────────────────────────
TEST(KeyValueParser, ParseStringHappyPath)
{
  const auto v = parse_yaml_string("source: lidar\n", "source");
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, "lidar");
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 6 — parse_yaml_int happy path.
// ─────────────────────────────────────────────────────────────────────────────
TEST(KeyValueParser, ParseIntHappyPath)
{
  const auto v = parse_yaml_int("point_count: 450\n", "point_count");
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, 450);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 7 — parse_yaml_int negative.
// ─────────────────────────────────────────────────────────────────────────────
TEST(KeyValueParser, ParseIntNegative)
{
  const auto v = parse_yaml_int("delta: -7\n", "delta");
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, -7);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 8 — load_dock_calibration_file with all 3 fields present round-trips.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockCalFileTest, LoadAllThreeFields)
{
  write_file(path_,
             "dock_pose_x: 1.234\n"
             "dock_pose_y: -5.678\n"
             "dock_pose_yaw_rad: 0.7854\n");
  const auto cal = load_dock_calibration_file(path_);
  ASSERT_TRUE(cal.has_value());
  EXPECT_NEAR(cal->x, 1.234, 1e-9);
  EXPECT_NEAR(cal->y, -5.678, 1e-9);
  EXPECT_NEAR(cal->yaw_rad, 0.7854, 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 9 — load_dock_calibration_file rejects partial input.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(DockCalFileTest, LoadMissingYawReturnsNullopt)
{
  write_file(path_,
             "dock_pose_x: 1.0\n"
             "dock_pose_y: 2.0\n");  // dock_pose_yaw_rad omitted
  const auto cal = load_dock_calibration_file(path_);
  EXPECT_FALSE(cal.has_value());
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 10 — cross-key collision regression (line-anchored parser).
// `dock_pose_x_offset` must NOT shadow `dock_pose_x` even though it shares
// the prefix. Without the BOL anchor (the historic `find(key + ":")`
// implementation), `parse_yaml_double(..., "dock_pose_x")` would match the
// first `_offset` line and return 1.0 instead of 2.0. This test pins the
// fix that makes Plan 02-04's dock_scan_meta loader safe to have both
// `dock_pose_x` and `sensor_extrinsic_x` keys.
// ─────────────────────────────────────────────────────────────────────────────
TEST(KeyValueParser, CrossKeyCollisionRegression)
{
  const std::string content =
      "dock_pose_x_offset: 1.0\n"
      "dock_pose_x: 2.0\n";
  const auto v = parse_yaml_double(content, "dock_pose_x");
  ASSERT_TRUE(v.has_value());
  EXPECT_DOUBLE_EQ(*v, 2.0)
      << "Parser must anchor on beginning-of-line; otherwise "
      << "`dock_pose_x_offset` would alias `dock_pose_x` and "
      << "Plan 02-04's dock_scan_meta loader would silently read the "
      << "wrong field.";
}
