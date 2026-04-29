// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "mowgli_geometry/atomic_write.hpp"

using mowgli_geometry::atomic_write;

namespace
{

std::string unique_temp_path()
{
  // Build a per-test unique path under /tmp (or std::filesystem::temp_directory_path).
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;

  auto base = std::filesystem::temp_directory_path();
  std::ostringstream oss;
  oss << "mowgli_geom_test_" << std::hex << dist(gen) << ".kv";
  return (base / oss.str()).string();
}

void cleanup_path(const std::string& path)
{
  std::error_code ec;
  std::filesystem::remove(path, ec);
  std::filesystem::remove(path + ".tmp", ec);
}

std::string read_all(const std::string& path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) return {};
  std::ostringstream oss;
  oss << f.rdbuf();
  return oss.str();
}

class AtomicWriteTest : public ::testing::Test
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
// Happy path: write "hello" → file exists, contents match, no .tmp orphan.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(AtomicWriteTest, WritesContentSuccessfully)
{
  ASSERT_TRUE(atomic_write(path_, "hello"));

  EXPECT_TRUE(std::filesystem::exists(path_));
  EXPECT_EQ(read_all(path_), "hello");

  // No .tmp orphan left over.
  EXPECT_FALSE(std::filesystem::exists(path_ + ".tmp"))
      << "atomic_write must rename the .tmp file, not leave it behind";
}

// ─────────────────────────────────────────────────────────────────────────────
// Overwrite: write twice — second write replaces first content atomically.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(AtomicWriteTest, OverwritesExistingFile)
{
  ASSERT_TRUE(atomic_write(path_, "first"));
  ASSERT_TRUE(atomic_write(path_, "second-much-longer-content"));
  EXPECT_EQ(read_all(path_), "second-much-longer-content");
  EXPECT_FALSE(std::filesystem::exists(path_ + ".tmp"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Multi-line content survives intact (newlines / trailing newline preserved).
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(AtomicWriteTest, MultiLineContentRoundTrips)
{
  const std::string content =
      "current_outline_index=2\n"
      "current_swath_index=14\n"
      "swath_direction=FORWARD\n"
      "checksum=DEADBEEF\n";
  ASSERT_TRUE(atomic_write(path_, content));
  EXPECT_EQ(read_all(path_), content);
}

// ─────────────────────────────────────────────────────────────────────────────
// Failure path: writing to a non-existent directory must return false (no
// crash, no partially-created file).
// ─────────────────────────────────────────────────────────────────────────────
TEST(AtomicWriteFailure, NonExistentDirectoryReturnsFalse)
{
  std::string bad_path = "/no/such/directory/mowgli_geom_test.kv";
  EXPECT_FALSE(atomic_write(bad_path, "hello"));
  EXPECT_FALSE(std::filesystem::exists(bad_path));
}

// ─────────────────────────────────────────────────────────────────────────────
// Empty content produces an empty file (no error).
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(AtomicWriteTest, EmptyContentSucceeds)
{
  ASSERT_TRUE(atomic_write(path_, ""));
  EXPECT_TRUE(std::filesystem::exists(path_));
  EXPECT_EQ(read_all(path_), "");
}
