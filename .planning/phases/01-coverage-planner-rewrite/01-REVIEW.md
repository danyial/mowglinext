---
phase: 01-coverage-planner-rewrite
reviewed: 2026-04-29T12:00:00Z
depth: standard
files_reviewed: 11
files_reviewed_list:
  - gui/pkg/api/diagnostics.go
  - gui/pkg/api/mowglinext.go
  - gui/web/src/types/ros.ts
  - ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp
  - ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp
  - ros2/src/mowgli_behavior/src/coverage_nodes.cpp
  - ros2/src/mowgli_behavior/test/test_coverage_nodes.cpp
  - ros2/src/mowgli_coverage_planner/src/plan_builder/plan_builder.cpp
  - ros2/src/mowgli_coverage_planner/test/CMakeLists.txt
  - ros2/src/mowgli_coverage_planner/test/test_plan_builder_area_index.cpp
  - ros2/src/mowgli_interfaces/msg/CoverageWaypoint.msg
findings:
  critical: 0
  warning: 3
  info: 4
  total: 7
status: issues_found
---

# Phase 1 (gap-closure 01-10 + 01-11): Code Review Report

**Reviewed:** 2026-04-29
**Depth:** standard
**Scope:** ONLY the 11 hand-edited files in plans 01-10 + 01-11. The broader phase
was reviewed previously; this overwrite focuses solely on the gap-closure work.
**Status:** issues_found

## Summary

The gap-closure for CR-01 (canonical area_index) is correctly implemented at the
contract boundary the plans target:

- `CoverageWaypoint.msg` adds `area_index` with a documented `UINT32_MAX`
  sentinel for non-area segments.
- `PlanBuilder` stamps the field on every emit path; the `stamp_and_push`
  lambda is a clean mitigation of T-10-04 (forgetting to set the field on a new
  emit path) for per-area waypoints. Dock/undock/return/approach/start-pose
  TRANSIT all carry `kNoArea`.
- `FollowCoveragePlan::dispatch_checkpoint_write` now reads
  `last_wp.area_index` (not `sequence_id`), short-circuits on the sentinel,
  and propagates `ctx->last_mow_angle_used_deg` (not a hard-coded 0.0).
- The new in-process WriteCheckpoint stub-server test exercises the real
  rclcpp service codepath and gives us a regression guard for the exact pre-fix
  bug (`area_index = sequence_id`).

However, the BT side of the checkpoint write also stamps three OTHER `Checkpoint`
fields (`current_swath_index`, `last_completed_swath_index`, `next_open_swath_index`,
and conditionally `current_outline_index`) using a **plan-wide waypoint index**
(`completed_end_idx_exclusive - 1`). Those fields are documented in
`Checkpoint.msg` as **per-area** counters and `PlanBuilder` consumes them as
per-area counters on resume — see findings WR-01 / WR-02 below. This is the
follow-on half of CR-01 the plan didn't address: the area_index key is now
correct, but the value written under that key is still wrong. The new test
asserts `area_index` and `last_mow_angle_deg` only, so the test suite does NOT
catch this.

The Go/TS files are read-through: TS regenerates fine, the Go HTTP layer is
unrelated to the gap-closure. No real findings there.

## Warnings

### WR-01: Checkpoint swath/outline indices are stamped with a plan-wide index, not per-area

**File:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:319-338`
**Issue:** `dispatch_checkpoint_write` populates four per-area Checkpoint
fields with `completed_end_idx_exclusive - 1`, which is the global plan-wide
waypoint index (`coverage_plan_` is a flat list across all areas):

```cpp
req->checkpoint.current_swath_index =
    static_cast<uint32_t>(completed_end_idx_exclusive - 1);
req->checkpoint.last_completed_swath_index =
    static_cast<uint32_t>(completed_end_idx_exclusive - 1);
req->checkpoint.next_open_swath_index =
    static_cast<uint32_t>(completed_end_idx_exclusive);
// ...and for OUTLINE_*:
req->checkpoint.current_outline_index =
    static_cast<uint32_t>(completed_end_idx_exclusive - 1);
```

`Checkpoint.msg` documents these as per-area indices (`# Per-area resume
state. One Checkpoint per working area`). `PlanBuilder` reads them that way:

- `plan_builder.cpp:242` — `if (resume_ck->current_outline_index < ctx.robot.outline_passes)`
  (compared against `outline_passes`, typically 1u)
- `plan_builder.cpp:255-258` — `per_pass * resume_ck->current_outline_index`
  is used to *front-trim* the outline-waypoint list of one area
- `plan_builder.cpp:326` — `mow_count < 2u * resume_ck->current_swath_index`
  is used to *front-trim* the mowing waypoints of one area (each swath = 2 wps)

Concrete failure scenario for a plan like `[UNDOCK0, UNDOCK1, MOW2, MOW3,
DOCK_APPROACH4, DOCKING5]` (one swath in area 1):

1. After completing the MOW pair, `dispatch_checkpoint_write(4)` is called.
2. BT writes `current_swath_index = 3`, `next_open_swath_index = 4`, and
   `current_outline_index = 0` (false — the segment was MOWING, not OUTLINE,
   so the conditional override at `coverage_nodes.cpp:333-338` doesn't fire,
   so `current_outline_index = 0` is what's persisted).
3. Resume reads `coverage_<1>.kv` with `current_swath_index = 3`.
4. PlanBuilder generates the same area's swaths (n waypoints) and tries to
   skip `2 * 3 = 6` mowing waypoints — every swath in the area gets skipped.
5. Net effect: resume marks the entire area as already-mowed and walks straight
   to RETURN_TO_DOCK, which is the opposite of "continue at the open swath
   endpoint within ≤ 5 cm + 5° yaw" the architecture invariant promises.

Severity: this re-opens R-11 (resume correctness) for any plan whose mowing
segments don't start at plan index 0. That is every real plan: every plan
begins with at least one UNDOCK waypoint. So in practice no resume currently
lands on the right swath after a charge cycle.

**Fix:** Track per-area counters explicitly. Either:

(a) walk `coverage_plan_[start..completed_end_idx_exclusive)` and count how
many `MOWING_BOUSTROPHEDON` waypoints with `area_index == last_wp.area_index`
preceded the current group, divide by 2 for the swath count; OR

(b) precompute, in `onStart` after snapshotting the plan, a `per_area_swath_offset_`
map: for each waypoint index, the number of completed mowing pairs in the
SAME area before it. Then use that map in `dispatch_checkpoint_write`. Same
for outline counts. Sketch:

```cpp
// In onStart (once per plan):
std::vector<uint32_t> per_area_mow_pair_after_;  // sized = plan.size()+1
{
  std::map<uint32_t, uint32_t> mow_count_by_area;
  for (size_t i = 0; i < coverage_plan_.size(); ++i) {
    per_area_mow_pair_after_[i] = mow_count_by_area[coverage_plan_[i].area_index] / 2u;
    if (coverage_plan_[i].segment_type == CW::SEGMENT_MOWING_BOUSTROPHEDON) {
      ++mow_count_by_area[coverage_plan_[i].area_index];
    }
  }
  per_area_mow_pair_after_.back() = /* terminal value */;
}

// In dispatch_checkpoint_write:
const uint32_t completed_pairs =
    per_area_mow_pair_after_[completed_end_idx_exclusive];
req->checkpoint.current_swath_index = completed_pairs;
req->checkpoint.last_completed_swath_index =
    completed_pairs == 0 ? 0 : completed_pairs - 1;
req->checkpoint.next_open_swath_index = completed_pairs;
```

Add a regression test paralleling `WriteCheckpointAreaIndexTest::PersistsCanonicalAreaIndex`
that asserts `current_swath_index == 1` (not 3) for the synthetic plan in
`build_synthetic_plan`.

### WR-02: `current_outline_index` defaults to 0 unless the LAST waypoint is OUTLINE_*

**File:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:319, 333-338`
**Issue:** Initial assignment is `req->checkpoint.current_outline_index = 0;`
and is only overridden when `last_wp.segment_type` is `OUTLINE_WORKING_AREA`
or `OUTLINE_OBSTACLE`. So after the BT completes the outlines for area 1 AND
THEN the first mowing pair, the persisted `current_outline_index` snaps back
to 0 — meaning a charge-cycle resume re-walks all the outlines of that area
from scratch. This contradicts the gating logic at
`plan_builder.cpp:241-242`:

```cpp
if (!resume_ck.has_value() ||
    resume_ck->current_outline_index < ctx.robot.outline_passes)
```

With `outline_passes = 1u` (the YardForce default), once the first mowing
pair completes and writes `current_outline_index = 0 < 1`, the next plan emits
the full outline set again. Robot drives the lawn perimeter twice on every
charge cycle.

**Fix:** Same per-area counter approach as WR-01. Maintain a sticky
`outline_passes_completed_for_area_[area_idx]` and write it through every
checkpoint after the area's outlines complete — not just on the OUTLINE_*
waypoint dispatch. The simplest formulation: outline_passes is a single int
per area that monotonically increases; once
`outline_passes_completed_for_area_[a] == ctx.robot.outline_passes`, any
subsequent checkpoint for area `a` keeps it at that value.

Alternatively (cleaner): make outline / swath progress derived state computed
at write time from a scan over `coverage_plan_[0..completed_end_idx_exclusive)`
restricted to `area_index == last_wp.area_index`. Slower per-write but
avoids state-tracking bugs.

### WR-03: `last_mow_angle_used_deg` defaults to 0.0 — first checkpoint can't be distinguished from "explicit 0°"

**File:** `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp:181`
**File:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:330`
**Issue:** `BTContext::last_mow_angle_used_deg{0.0}` defaults to a value that
is *also* a valid mow angle (0° is a perfectly normal lawn orientation).
The header comment claims this default "is never persisted to a real
checkpoint file" because reads only happen after a successful plan, but the
guard isn't enforced — `dispatch_checkpoint_write` reads
`ctx->last_mow_angle_used_deg` unconditionally (line 330), and
`coverage_plan_` is populated independently via `setCoveragePlan` in tests
and via blackboard snapshot in production.

The risk: if `PlanCoverageGoal` ever fails to populate
`last_mow_angle_used_deg` (e.g., an action result missing the metadata
field, or a manual override path injecting a plan into the blackboard) but
`coverage_plan_` is non-empty, BT will persist `last_mow_angle_deg = 0.0`
and the next plan's `derive_mow_angle` will increment from 0° — potentially
stomping a real auto-rotate state mid-area.

**Fix:** Use `std::numeric_limits<double>::quiet_NaN()` (or a sentinel like
`-1.0` matching `mow_angle_offset_deg`'s convention) and have
`dispatch_checkpoint_write` guard:

```cpp
if (!std::isfinite(ctx->last_mow_angle_used_deg)) {
  RCLCPP_WARN(ctx->node->get_logger(),
              "FollowCoveragePlan: last_mow_angle_used_deg unset, "
              "skipping checkpoint to avoid persisting 0° default");
  return;
}
req->checkpoint.last_mow_angle_deg = ctx->last_mow_angle_used_deg;
```

The new `WriteCheckpointAreaIndexTest::PersistsCanonicalAreaIndex` test sets
`ctx_->last_mow_angle_used_deg = 42.5` so it does not exercise this path.

## Info

### IN-01: PlanBuilder outline-passes resume math is fragile

**File:** `ros2/src/mowgli_coverage_planner/src/plan_builder/plan_builder.cpp:251-259`
**Issue:** The resume-skip math on outlines is

```cpp
const std::size_t per_pass =
    outlines.waypoints.size() / ctx.robot.outline_passes;
skip_n = std::min<std::size_t>(
    outlines.waypoints.size(),
    per_pass * resume_ck->current_outline_index);
```

This only works when `outlines.waypoints.size() % outline_passes == 0`, i.e.
the OutlineGenerator emits exactly the same vertex count for every pass.
The outer guard `ctx.robot.outline_passes > 0u` (line 252) protects the
divide-by-zero today, but the math is brittle. A comment noting
"OutlineGenerator must emit a constant waypoints-per-pass count for this
front-trim to be exact" would help future readers. Or better: have
OutlineGenerator return a vector-of-vectors so the front-trim becomes
`std::vector<>(passes.begin() + n, passes.end())`.

**Fix:** Either add the comment, or refactor `OutlineGenerationResult` to
expose passes individually. Not a blocker for the gap-closure.

### IN-02: Test fixture computes mowing-pair counts that may be 0 — relies on `EXPECT_GT` only

**File:** `ros2/src/mowgli_coverage_planner/test/test_plan_builder_area_index.cpp:151-153`
**Issue:** The two assertions

```cpp
EXPECT_GT(mow_for_a0, 0u) << "no MOWING waypoints stamped for area 0";
EXPECT_GT(mow_for_a1, 0u) << "no MOWING waypoints stamped for area 1";
```

are correct as a smoke test, but they don't catch a swap (e.g., area 0's
sweep waypoints stamped with area_index=1 in some bug). A 5x5 m square at
`(0,0)` and a 5x5 m square at `(20,0)` produce the same swath count, so a
bug that swapped both areas' indices wholesale would leave both counters
non-zero and pass this test. Adding a positional sanity check ("waypoints
with `area_index=0` have `pose.position.x` near 0; waypoints with
`area_index=1` have x near 20") would catch a swap.

**Fix:** Add a per-area positional-bounding-box assertion alongside
`EXPECT_GT`:

```cpp
if (wp.area_index == 0u) {
  EXPECT_LT(std::abs(wp.pose.pose.position.x), 5.0)
      << "area-0 waypoint at x=" << wp.pose.pose.position.x
      << " seems to belong to area 1 (centred at x=20)";
} else if (wp.area_index == 1u) {
  EXPECT_GT(wp.pose.pose.position.x, 15.0)
      << "area-1 waypoint at x=" << wp.pose.pose.position.x
      << " seems to belong to area 0 (centred at x=0)";
}
```

### IN-03: `read_checkpoint_file` called twice per area on resume

**File:** `ros2/src/mowgli_coverage_planner/src/plan_builder/plan_builder.cpp:108, 228`
**Issue:** `derive_mow_angle` (line 108) and the per-area resume block (line
228) both call `read_checkpoint_file(areas_dir_, idx)` for the same area.
The function does file I/O. Not a correctness issue and unchanged by the
gap-closure plans — but worth a one-line note. A future refactor should
hoist the call once into `build()` and pass the optional through.

**Fix:** Cache. Out of scope for the gap-closure; flagging for future
cleanup.

### IN-04: `dispatch_checkpoint_write` future is discarded — completion errors are silently dropped

**File:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:343`
**Issue:** `checkpoint_client_->async_send_request(req);` discards the
returned future. The header (`coverage_nodes.hpp:204-206`) declares
`checkpoint_future_` and `checkpoint_in_flight_` for tracking — but neither
is used here. So a service handler that responds with `success=false`
(e.g., disk full, atomic-write failure) is invisible to the BT and the
operator. The "WARN on failure" promise on line 309 only fires for
`wait_for_service` timeouts, not for per-request failures.

The Q1 lock ("BT NEVER touches the filesystem; next successful checkpoint
is the recovery point") is what justifies fire-and-forget — and that's
fine — but at minimum the BT should consume the future and WARN on
`success=false` so a persistent disk-full condition produces logs.

**Fix:** Store the future in `checkpoint_future_` and poll it in
`onRunning` on a subsequent tick (during `IDLE`), or attach a callback via
the `async_send_request` overload that takes a callback. Sketch using the
callback-based send:

```cpp
checkpoint_client_->async_send_request(
    req,
    [logger = ctx->node->get_logger()](
        rclcpp::Client<mowgli_interfaces::srv::WriteCheckpoint>::SharedFuture fut) {
      auto resp = fut.get();
      if (!resp->success) {
        RCLCPP_WARN(logger,
                    "FollowCoveragePlan: WriteCheckpoint reported failure: %s",
                    resp->error_message.c_str());
      }
    });
```

This is one small step beyond the gap-closure scope — flagging because the
declared `checkpoint_in_flight_` flag suggests the original intent was to
track it, and the test suite never asserts the response was even consumed.

---

_Reviewed: 2026-04-29_
_Reviewer: Claude (gsd-code-reviewer)_
_Depth: standard_
_Scope: gap-closure plans 01-10 + 01-11 only_
