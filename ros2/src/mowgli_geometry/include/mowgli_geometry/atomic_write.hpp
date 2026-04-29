// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// Power-loss-safe file write for coverage-planner checkpoint sidecars
// (RESEARCH §7.2). Implements the 4-step pattern verbatim:
//   1. open(tmp, O_WRONLY|O_CREAT|O_TRUNC) + write + fsync(fd) + close
//   2. rename(tmp, path)
//   3. open(parent_dir, O_RDONLY|O_DIRECTORY) + fsync(dir_fd) + close
// Skipping step 3 risks the rename being lost on power loss on ext4 / SD
// (LWN.net "A way to do atomic writes" + kernel.org ext4 docs).
//
// The caller is responsible for path validation. T-02-01 is mitigated at
// the call site (Plan 05 ValidatorPipeline) by deriving paths from the
// configured `areas_dir` parameter.

#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace mowgli_geometry
{

namespace detail
{

/// Best-effort cleanup of a stale .tmp file (errors silently swallowed —
/// caller has already failed and the OS's temp-file cleanup will sweep
/// the rest).
inline void unlink_tmp(const std::string& tmp_path) noexcept
{
  ::unlink(tmp_path.c_str());
}

}  // namespace detail

/// Atomic small-file writer. Returns true on success; false on any errno
/// failure (open, write, fsync, rename). On failure, no .tmp orphan
/// remains beyond what the OS's normal cleanup handles.
inline bool atomic_write(const std::string& path, const std::string& content)
{
  // Step 1: write to a sibling .tmp file.
  const std::string tmp_path = path + ".tmp";
  const int fd = ::open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0)
  {
    return false;
  }

  // Write the full content. write(2) may return short on signals, so loop.
  std::size_t total_written = 0;
  const char* buf = content.data();
  const std::size_t len = content.size();
  while (total_written < len)
  {
    ssize_t n = ::write(fd, buf + total_written, len - total_written);
    if (n < 0)
    {
      if (errno == EINTR) continue;  // retry on signal
      ::close(fd);
      detail::unlink_tmp(tmp_path);
      return false;
    }
    total_written += static_cast<std::size_t>(n);
  }

  // fsync(fd): force data + metadata to storage before rename.
  if (::fsync(fd) != 0)
  {
    ::close(fd);
    detail::unlink_tmp(tmp_path);
    return false;
  }
  if (::close(fd) != 0)
  {
    detail::unlink_tmp(tmp_path);
    return false;
  }

  // Step 2: atomic rename onto the target path. POSIX rename is atomic
  // within a filesystem.
  if (::rename(tmp_path.c_str(), path.c_str()) != 0)
  {
    detail::unlink_tmp(tmp_path);
    return false;
  }

  // Step 3: fsync the parent directory so the rename survives a crash on
  // ext4 / SD where the directory entry update may otherwise sit in the
  // page cache. Best-effort: directory fsync failures are non-fatal (the
  // file is already in place — only durability across power loss is at
  // risk), but we still report failure to the caller for visibility.
  std::string dir;
  const auto last_slash = path.rfind('/');
  if (last_slash == std::string::npos)
  {
    dir = ".";
  }
  else if (last_slash == 0)
  {
    dir = "/";
  }
  else
  {
    dir = path.substr(0, last_slash);
  }

  const int dir_fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY);
  if (dir_fd < 0)
  {
    return false;
  }
  const int dir_sync_rc = ::fsync(dir_fd);
  ::close(dir_fd);
  if (dir_sync_rc != 0)
  {
    return false;
  }

  return true;
}

}  // namespace mowgli_geometry
