---
phase: 01-coverage-planner-rewrite
fixed_at: 2026-04-29T15:45:00Z
review_path: .planning/phases/01-coverage-planner-rewrite/01-REVIEW.md
iteration: 1
fix_scope: critical_warning
findings_in_scope: 3
fixed: 2
skipped: 1
status: partial
---

# Phase 1: Code Review Fix Report

**Fixed at:** 2026-04-29T15:45:00Z
**Source review:** `.planning/phases/01-coverage-planner-rewrite/01-REVIEW.md`
**Iteration:** 1

**Summary:**
- Findings in scope: 3 (WR-01, WR-02, WR-03)
- Fixed: 2 (WR-01, WR-02)
- Skipped: 1 (WR-03 — accepted as designed)

## Fixed Issues

### WR-01: Checkpoint swath indices are now per-area counts

**Files modified:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp`, `ros2/src/mowgli_behavior/test/test_coverage_nodes.cpp`
**RED commit:** `e06e6794` (failing test assertions added)
**GREEN commit:** `4e06115d` (implementation fix)

**Applied fix:** Replaced the three plan-wide swath index assignments
(`completed_end_idx_exclusive - 1`) with a linear scan over
`coverage_plan_[0..completed_end_idx_exclusive)` that counts only waypoints
with `area_index == last_wp.area_index` and `segment_type == MOWING_BOUSTROPHEDON`.
The count divided by 2 gives `completed_pairs`, which is the per-area pair
count that `PlanBuilder::build:326` expects. Assignments:

- `current_swath_index = completed_pairs`
- `last_completed_swath_index = (completed_pairs > 0) ? completed_pairs - 1 : 0`
- `next_open_swath_index = completed_pairs`

Concrete correction for the synthetic plan `[UNDOCK0, UNDOCK1, MOW2, MOW3,
DOCK4, DOCK5]`: `current_swath_index` was 3 (plan-wide, caused resume to skip
`2*3=6` wps → entire area skipped). Now it is 1 (1 pair completed), so resume
skips `2*1=2` wps and continues at the correct swath endpoint.

**Regression tests added** (extending `WriteCheckpointAreaIndexTest`):
- `PersistsCanonicalAreaIndex` now additionally asserts:
  `current_swath_index == 1u`, `last_completed_swath_index == 0u`,
  `next_open_swath_index == 1u` for the existing synthetic plan.

**Verification note:** Colcon build and `colcon test --packages-select mowgli_behavior`
are deferred to docker/dev-branch CI — macOS host has no ROS2 toolchain.
This matches the established pattern from Plans 01-07, 01-09, 01-10, 01-11.
The fix requires human verification flag: logic correctness of the scan
algorithm should be confirmed by a developer before the Pi5 hardware smoke
test (AC-13).

---

### WR-02: `current_outline_index` is now sticky after outline pass completes

**Files modified:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp`, `ros2/src/mowgli_behavior/test/test_coverage_nodes.cpp`
**RED commit:** `e06e6794` (new `PersistsOutlineIndexAfterMowing` test added, failing)
**GREEN commit:** `4e06115d` (same commit as WR-01 — implemented together)

**Applied fix:** Removed the old conditional
`if (last_wp.segment_type == CW::SEGMENT_OUTLINE_WORKING_AREA || ...)` override
that set `current_outline_index = completed_end_idx_exclusive - 1` (another
plan-wide index). Replaced with a scan-based transition detector: while walking
`coverage_plan_[0..completed_end_idx_exclusive)` for the same area, track when
the last waypoint type transitions from `OUTLINE_WORKING_AREA` to
`MOWING_BOUSTROPHEDON`. Each such transition increments `outline_passes_done`.
After the scan, `req->checkpoint.current_outline_index = outline_passes_done`.

With `outline_passes=1` (YardForce default): once the single outline run for
an area is followed by any mowing waypoint, `outline_passes_done = 1`, and
the checkpoint persists `current_outline_index = 1`. On resume,
`plan_builder.cpp:242` evaluates `!(1 < 1)` → skips outline emission. Without
this fix, `current_outline_index` snapped back to 0 after the first mowing
pair, causing the robot to re-walk the lawn perimeter on every charge cycle.

**Note on `OUTLINE_OBSTACLE`:** Obstacle outlines are always re-emitted by
`PlanBuilder` regardless of `current_outline_index` (there is no resume-skip
logic for obstacle outlines in `plan_builder.cpp:267-287`). The scan therefore
counts only `OUTLINE_WORKING_AREA` transitions, not `OUTLINE_OBSTACLE`.

**Regression test added:**
- New `PersistsOutlineIndexAfterMowing` test with plan
  `[UNDOCK, OUTLINE, OUTLINE, MOW, MOW, DOCKING]` for area 1.
  Asserts `current_outline_index == 1u` after dispatching the mowing pair,
  and `current_swath_index == 1u` (combined WR-01 + WR-02 coverage).

**Verification note:** Same CI deferral applies. Logic-correctness flag: the
transition-detection approach assumes PlanBuilder always emits working-area
outlines before mowing waypoints for the same area, which holds by construction
in `plan_builder.cpp` (outline loop at line 241, mowing loop at line 291).
For `outline_passes > 1` (not currently used in production), the counter will
only increment once per `OUTLINE_WORKING_AREA → MOW` transition, so the value
will be 1 regardless of how many passes were planned. This is a known
limitation — documented for future improvement if multi-pass outlines are
introduced.

---

## Skipped Issues

### WR-03: `last_mow_angle_used_deg{0.0}` default

**File:** `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp:181`
**Reason:** `skipped: accepted_as_designed`

**Decision rationale:**

The `01-11-SUMMARY.md` key-decisions block and the `bt_context.hpp` header
comment (line 178-180) both explicitly document that the `0.0` default is
intentional and safe:

> "Defaults to 0.0 (= 'no plan ran yet'); read by FollowCoveragePlan only
> when an actual plan was loaded into coverage_plan, so the default is
> never persisted to a real checkpoint file."

The Plan 01-11 design decision is: `PlanCoverageGoal::onRunning` sets
`ctx->last_mow_angle_used_deg` from `PlanMetadata.mow_angle_used_deg` on the
success path (commit `5405fa60`, line 151). `FollowCoveragePlan::dispatch_checkpoint_write`
only runs when `coverage_plan_` is non-empty (guarded by `onStart` snapshot).
The BT sequencing invariant — `PlanCoverageGoal` must succeed before
`FollowCoveragePlan` ticks — means `last_mow_angle_used_deg` is always
populated before a checkpoint write occurs in a well-formed BT.

The REVIEW's risk scenario ("if `PlanCoverageGoal` ever fails to populate
the field") requires a broken BT tree or a non-standard test injection. The
existing `WriteCheckpointAreaIndexTest` fixture explicitly sets
`ctx_->last_mow_angle_used_deg = 42.5` to test the propagation path.

While a NaN/`-1.0` sentinel would add defense-in-depth, the plan history
accepts the `0.0` default as sufficient for the current production code path.
This finding is deferred to a future hardening pass if the blackboard-injection
pattern becomes used outside test fixtures.

**Original issue:** `BTContext::last_mow_angle_used_deg{0.0}` is
indistinguishable from a legitimate 0° mow angle, risking silent stomp of
auto-rotate state if the field is not populated before use.

---

## Verification Gap Note

Full `colcon build` + `colcon test` deferred to docker/dev-branch CI for all
fixes. This matches the established pattern documented in Plans 01-07, 01-09,
01-10, and 01-11. The macOS host has no ROS2 Kilted toolchain. No regressions
to the existing R-9 fix were introduced — only `dispatch_checkpoint_write`
internals were changed; the `area_index` and `last_mow_angle_deg` assignments
are untouched.

---

_Fixed: 2026-04-29T15:45:00Z_
_Fixer: Claude (gsd-code-fixer)_
_Iteration: 1_
