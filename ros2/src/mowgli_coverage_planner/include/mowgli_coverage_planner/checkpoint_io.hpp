// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// Checkpoint .kv serializer / parser for the BT-delegates-IO-to-planner
// pattern locked in Plan 01-01 (RESEARCH §10 Q1). The .kv format is the
// 9-key key=value text described in RESEARCH §7.1. Atomic write delegates
// to mowgli_geometry::atomic_write (Plan 01-02).
//
// Threat model:
//   - T-05-01 (path traversal): area_index is uint32, interpolated via
//     std::to_string -> digit-only output. No `..` or slash escape.
//   - T-05-02 (NaN poses): write_checkpoint_file() rejects non-finite
//     endpoint coordinates before serialize.
//   - T-05-03 (disk corruption): parse_kv() returns nullopt on any failure;
//     caller maps to ERROR_RESUME_CHECKPOINT_INVALID per SPEC R-12.
//   - T-05-04 (mid-write crash): atomic_write fsyncs both the file and the
//     parent directory before returning success.

#include <cstdint>
#include <optional>
#include <string>

#include <mowgli_interfaces/msg/checkpoint.hpp>

namespace mowgli_coverage_planner
{

/// Serializes a Checkpoint to the 9-line key=value text format.
/// Floats are written with std::fixed + std::setprecision(6).
/// swath_direction emits "FORWARD" / "REVERSE" string tokens (not the
/// uint8 numeric value) so the file is human-inspectable.
/// area_index is NOT included in the body — it is encoded in the filename.
std::string serialize_checkpoint(const mowgli_interfaces::msg::Checkpoint& ck);

/// Parses .kv content. Returns nullopt on:
///   - any required key missing
///   - non-numeric float value (std::stod throws)
///   - non-uint32 index value (std::stoul throws / out of range)
///   - unknown swath_direction token
/// area_index is NOT populated by parse_kv — caller fills it from the
/// filename.
std::optional<mowgli_interfaces::msg::Checkpoint> parse_kv(
    const std::string& content);

/// Writes Checkpoint to <areas_dir>/coverage_<area_index>.kv via
/// mowgli_geometry::atomic_write. Returns true on success. On failure,
/// *error_out (when non-null) is populated with a human-readable string
/// describing the rejection reason (e.g. "non-finite checkpoint values
/// rejected", "atomic_write failed for path: <path>").
bool write_checkpoint_file(const std::string& areas_dir,
                           const mowgli_interfaces::msg::Checkpoint& ck,
                           std::string* error_out);

/// Reads <areas_dir>/coverage_<area_index>.kv. Returns nullopt if the file
/// does not exist or parse_kv() rejects the content. On success, the
/// returned Checkpoint has area_index set from the caller's argument.
std::optional<mowgli_interfaces::msg::Checkpoint> read_checkpoint_file(
    const std::string& areas_dir, std::uint32_t area_index);

}  // namespace mowgli_coverage_planner
