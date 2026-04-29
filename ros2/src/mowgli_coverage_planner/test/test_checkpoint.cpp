// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// Round-trip + corruption-detection + atomic-write coverage for the
// Checkpoint .kv format. Mirrors SPEC R-10 acceptance bits (atomic
// per-area sidecar, full field set, corruption -> RESUME_CHECKPOINT_INVALID
// at the call-site).

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>
#include <mowgli_interfaces/msg/checkpoint.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "mowgli_coverage_planner/checkpoint_io.hpp"

namespace fs = std::filesystem;
using mowgli_coverage_planner::parse_kv;
using mowgli_coverage_planner::read_checkpoint_file;
using mowgli_coverage_planner::serialize_checkpoint;
using mowgli_coverage_planner::write_checkpoint_file;
using Checkpoint = mowgli_interfaces::msg::Checkpoint;

namespace
{

/// Build a populated Checkpoint with deterministic values (and a real yaw
/// in the quaternion via tf2 so the round-trip exercises the orientation
/// path).
Checkpoint make_checkpoint()
{
  Checkpoint ck;
  ck.area_index = 7u;
  ck.current_outline_index = 2u;
  ck.current_swath_index = 14u;
  ck.swath_direction = Checkpoint::SWATH_DIRECTION_FORWARD;
  ck.last_completed_swath_index = 13u;
  ck.next_open_swath_index = 14u;
  ck.last_mow_angle_deg = 45.0;

  ck.last_swath_endpoint.position.x = 12.345678;
  ck.last_swath_endpoint.position.y = 9.876543;
  ck.last_swath_endpoint.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 1.570796);  // yaw = pi/2
  ck.last_swath_endpoint.orientation.x = q.x();
  ck.last_swath_endpoint.orientation.y = q.y();
  ck.last_swath_endpoint.orientation.z = q.z();
  ck.last_swath_endpoint.orientation.w = q.w();
  return ck;
}

/// Per-test temp directory; cleaned up in TearDown.
class CheckpointFsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto base = fs::temp_directory_path();
    char tpl[] = "mowgli_ckpt_test_XXXXXX";
    // mkdtemp wants a writable buffer; use mkdtemp via a dynamic copy.
    std::string tpath = (base / tpl).string();
    // Use std::filesystem to create a unique dir.
    int counter = 0;
    while (true)
    {
      auto cand = base / ("mowgli_ckpt_test_" + std::to_string(::getpid()) +
                          "_" + std::to_string(counter));
      std::error_code ec;
      if (fs::create_directory(cand, ec))
      {
        tmpdir_ = cand.string();
        break;
      }
      ++counter;
      ASSERT_LT(counter, 1024) << "could not create unique tmpdir";
    }
  }
  void TearDown() override
  {
    if (!tmpdir_.empty())
    {
      std::error_code ec;
      fs::remove_all(tmpdir_, ec);
    }
  }
  std::string tmpdir_;
};

}  // namespace

// ----- Round-trip ----------------------------------------------------------

TEST(CheckpointSerialize, RoundTrip)
{
  Checkpoint ck = make_checkpoint();
  const std::string text = serialize_checkpoint(ck);

  // Sanity: 9 key=value lines, each ending in '\n'. (Trailing newline OK.)
  std::size_t newlines = 0;
  for (char c : text) if (c == '\n') ++newlines;
  EXPECT_EQ(newlines, 9u) << "serialize_checkpoint must emit exactly 9 lines";

  // Parser does NOT fill area_index — caller does.
  auto parsed_opt = parse_kv(text);
  ASSERT_TRUE(parsed_opt.has_value()) << "parse_kv rejected its own output";
  Checkpoint parsed = *parsed_opt;

  EXPECT_EQ(parsed.current_outline_index, ck.current_outline_index);
  EXPECT_EQ(parsed.current_swath_index, ck.current_swath_index);
  EXPECT_EQ(parsed.swath_direction, ck.swath_direction);
  EXPECT_EQ(parsed.last_completed_swath_index, ck.last_completed_swath_index);
  EXPECT_EQ(parsed.next_open_swath_index, ck.next_open_swath_index);
  EXPECT_NEAR(parsed.last_mow_angle_deg, ck.last_mow_angle_deg, 1e-5);
  EXPECT_NEAR(parsed.last_swath_endpoint.position.x,
              ck.last_swath_endpoint.position.x, 1e-5);
  EXPECT_NEAR(parsed.last_swath_endpoint.position.y,
              ck.last_swath_endpoint.position.y, 1e-5);

  // Yaw must round-trip to within 1e-5 (we serialise as a single yaw scalar
  // and parse it back into a quaternion; tf2::getYaw of the parsed
  // quaternion should equal the original yaw).
  tf2::Quaternion qparsed(parsed.last_swath_endpoint.orientation.x,
                          parsed.last_swath_endpoint.orientation.y,
                          parsed.last_swath_endpoint.orientation.z,
                          parsed.last_swath_endpoint.orientation.w);
  double r{}, p{}, y{};
  tf2::Matrix3x3(qparsed).getRPY(r, p, y);
  EXPECT_NEAR(y, 1.570796, 1e-4);
}

TEST(CheckpointSerialize, ReverseDirectionToken)
{
  Checkpoint ck = make_checkpoint();
  ck.swath_direction = Checkpoint::SWATH_DIRECTION_REVERSE;
  const std::string text = serialize_checkpoint(ck);
  EXPECT_NE(text.find("swath_direction=REVERSE"), std::string::npos)
      << "REVERSE direction must serialise as a string token";
  auto parsed = parse_kv(text);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->swath_direction, Checkpoint::SWATH_DIRECTION_REVERSE);
}

// ----- Corruption detection ------------------------------------------------

TEST(CheckpointParse, RejectsGarbage)
{
  EXPECT_FALSE(parse_kv("garbage with no equals").has_value());
}

TEST(CheckpointParse, RejectsMissingRequiredKey)
{
  // Build a valid blob then strip one required line.
  Checkpoint ck = make_checkpoint();
  std::string text = serialize_checkpoint(ck);

  const std::string needle = "last_mow_angle_deg=";
  const auto pos = text.find(needle);
  ASSERT_NE(pos, std::string::npos);
  const auto eol = text.find('\n', pos);
  ASSERT_NE(eol, std::string::npos);
  text.erase(pos, eol - pos + 1);

  EXPECT_FALSE(parse_kv(text).has_value())
      << "parse_kv must reject when a required key is missing";
}

TEST(CheckpointParse, RejectsNonNumericFloat)
{
  // Replace the angle line's value with a non-numeric string.
  Checkpoint ck = make_checkpoint();
  std::string text = serialize_checkpoint(ck);
  const std::string needle = "last_mow_angle_deg=";
  const auto pos = text.find(needle);
  ASSERT_NE(pos, std::string::npos);
  const auto val_start = pos + needle.size();
  const auto eol = text.find('\n', val_start);
  ASSERT_NE(eol, std::string::npos);
  text.replace(val_start, eol - val_start, "not-a-number");

  EXPECT_FALSE(parse_kv(text).has_value())
      << "parse_kv must reject non-numeric float values";
}

TEST(CheckpointParse, RejectsUnknownSwathDirectionToken)
{
  Checkpoint ck = make_checkpoint();
  std::string text = serialize_checkpoint(ck);
  const std::string needle = "swath_direction=";
  const auto pos = text.find(needle);
  ASSERT_NE(pos, std::string::npos);
  const auto val_start = pos + needle.size();
  const auto eol = text.find('\n', val_start);
  ASSERT_NE(eol, std::string::npos);
  text.replace(val_start, eol - val_start, "SIDEWAYS");

  EXPECT_FALSE(parse_kv(text).has_value())
      << "parse_kv must reject unknown swath_direction tokens";
}

// ----- Atomic write + filesystem round-trip --------------------------------

TEST_F(CheckpointFsTest, AtomicWriteAndReadBack)
{
  Checkpoint ck = make_checkpoint();
  std::string err;
  const bool ok = write_checkpoint_file(tmpdir_, ck, &err);
  ASSERT_TRUE(ok) << "write_checkpoint_file failed: " << err;

  const auto path = fs::path(tmpdir_) /
                    ("coverage_" + std::to_string(ck.area_index) + ".kv");
  ASSERT_TRUE(fs::exists(path)) << "expected .kv file at " << path;

  // No .tmp orphan.
  EXPECT_FALSE(fs::exists(path.string() + ".tmp"))
      << ".tmp orphan present after successful atomic_write";

  // File content has exactly 9 newlines.
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  std::size_t newlines = 0;
  for (char c : ss.str()) if (c == '\n') ++newlines;
  EXPECT_EQ(newlines, 9u);

  // Round-trip via read_checkpoint_file.
  auto loaded_opt = read_checkpoint_file(tmpdir_, ck.area_index);
  ASSERT_TRUE(loaded_opt.has_value());
  Checkpoint loaded = *loaded_opt;
  EXPECT_EQ(loaded.area_index, ck.area_index);
  EXPECT_EQ(loaded.current_swath_index, ck.current_swath_index);
  EXPECT_EQ(loaded.swath_direction, ck.swath_direction);
  EXPECT_NEAR(loaded.last_mow_angle_deg, ck.last_mow_angle_deg, 1e-5);
  EXPECT_NEAR(loaded.last_swath_endpoint.position.x,
              ck.last_swath_endpoint.position.x, 1e-5);
  EXPECT_NEAR(loaded.last_swath_endpoint.position.y,
              ck.last_swath_endpoint.position.y, 1e-5);
}

TEST_F(CheckpointFsTest, RejectsNonFiniteCheckpointValues)
{
  // T-05-02 mitigation: write_checkpoint_file refuses NaN endpoint coords.
  Checkpoint ck = make_checkpoint();
  ck.last_swath_endpoint.position.x = std::nan("");

  std::string err;
  const bool ok = write_checkpoint_file(tmpdir_, ck, &err);
  EXPECT_FALSE(ok) << "non-finite checkpoint values must be rejected";
  EXPECT_FALSE(err.empty()) << "error_out must be populated on rejection";

  // No file / .tmp must have been created.
  const auto path = fs::path(tmpdir_) /
                    ("coverage_" + std::to_string(ck.area_index) + ".kv");
  EXPECT_FALSE(fs::exists(path));
  EXPECT_FALSE(fs::exists(path.string() + ".tmp"));
}

TEST_F(CheckpointFsTest, ReadCheckpointMissingFileReturnsNullopt)
{
  auto loaded = read_checkpoint_file(tmpdir_, 99u);
  EXPECT_FALSE(loaded.has_value());
}

TEST_F(CheckpointFsTest, ReadCheckpointCorruptedFileReturnsNullopt)
{
  // Write garbage by hand (bypassing write_checkpoint_file).
  const std::uint32_t area_index = 3u;
  const auto path = fs::path(tmpdir_) /
                    ("coverage_" + std::to_string(area_index) + ".kv");
  std::ofstream of(path);
  of << "this is not a valid checkpoint file\n";
  of.close();
  ASSERT_TRUE(fs::exists(path));

  auto loaded = read_checkpoint_file(tmpdir_, area_index);
  EXPECT_FALSE(loaded.has_value())
      << "read_checkpoint_file must return nullopt on corrupted content "
         "(triggers ERROR_RESUME_CHECKPOINT_INVALID at the call site)";
}
