// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// SPEC R-12 pre-flight + post-geometry validation pipeline. Each validator
// runs in fixed order, fail-fast: the first non-nullopt PlanError aborts
// the pipeline. The 8 PlanError.error_code values map to validator types
// per CONTEXT §"Pre-geometry validations" + §"Post-geometry validations".
//
// Threat model T-07-01..T-07-08:
//  - InputSanityValidator (T-07-05) — first PreGeometry validator; rejects
//    NaN / Inf coordinates in start_pose / dock_pose with ERROR_INTERNAL.
//  - SegmentTypeInvariantValidator (T-07-07) — guards blade-on inside
//    navigation areas (HIGH severity physical-safety mitigation).
//  - FootprintDisjointObstaclesValidator (T-07-08) — guards footprint
//    intersecting obstacles (HIGH severity physical-safety mitigation).

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mowgli_interfaces/msg/plan_error.hpp>

#include "mowgli_coverage_planner/plan_context.hpp"

namespace mowgli_coverage_planner
{

/// Abstract validator. `check()` returns nullopt on success and a populated
/// PlanError on failure (the pipeline aborts at the first non-nullopt result).
class Validator
{
public:
  virtual ~Validator() = default;
  virtual std::optional<mowgli_interfaces::msg::PlanError> check(
      const PlanContext& ctx) const = 0;
  virtual const char* name() const = 0;
};

/// Ordered fail-fast pipeline. `add_pre_geometry_validators` and
/// `add_post_geometry_validators` register the SPEC R-12 validation points
/// in the canonical order. The two phases share this class but are
/// instantiated separately so `execute()` can run them around the plan
/// builder per the pipeline locked in CONTEXT.md.
class ValidatorPipeline
{
public:
  ValidatorPipeline() = default;

  /// Register the 6 pre-geometry validators (input sanity, no-areas,
  /// dock-in-area, area-width, obstacle-coverage, resume-checkpoint,
  /// obstacle-offset). Order is fixed; tests rely on it.
  void add_pre_geometry_validators();

  /// Register the 5 post-geometry validators (footprint-inside-area,
  /// footprint-disjoint-obstacles, segment-type-invariants, path-spacing,
  /// dock-segments-collision-free).
  void add_post_geometry_validators();

  /// Run all registered validators in order. Returns the first non-nullopt
  /// PlanError; on success returns nullopt and `run_log_for_test_only`
  /// contains every validator name that was evaluated (always == size of
  /// the pipeline on the success path; less on failure).
  std::optional<mowgli_interfaces::msg::PlanError> run(const PlanContext& ctx);

  /// Pipeline introspection. Test-only hook used by test_validation_pipeline
  /// to assert that all SPEC R-12 validation points are evaluated on the
  /// success path. Caller must call `run()` first.
  const std::vector<std::string>& run_log_for_test_only() const
  {
    return run_log_;
  }

  /// Number of validators registered.
  std::size_t size() const { return validators_.size(); }

private:
  std::vector<std::unique_ptr<Validator>> validators_;
  std::vector<std::string> run_log_;
};

}  // namespace mowgli_coverage_planner
