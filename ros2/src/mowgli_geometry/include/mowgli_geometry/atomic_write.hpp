// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// Power-loss-safe file write for coverage-planner checkpoint sidecars
// (RESEARCH §7.2). Implements the 4-step pattern:
//   1. open(tmp, O_WRONLY|O_CREAT|O_TRUNC) + write + fsync(fd) + close
//   2. rename(tmp, path)
//   3. open(parent_dir, O_RDONLY|O_DIRECTORY) + fsync(dir_fd) + close
// Skipping step 3 risks the rename being lost on power loss on ext4 / SD.
//
// STUB MARKER (RED gate, plan 01-02 task 2): the body is replaced in
// the GREEN gate of this same task with the full 4-step sequence.

#include <string>

namespace mowgli_geometry
{

/// Atomic small-file writer. Returns true on success; false on any errno
/// failure. Caller is responsible for path validation (the helper trusts
/// the supplied path).
inline bool atomic_write(const std::string& /*path*/, const std::string& /*content*/)
{
  return false;  // STUB — fails the success-path test
}

}  // namespace mowgli_geometry
