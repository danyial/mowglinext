---
phase: 01-coverage-planner-rewrite
verified: 2026-04-29T14:30:00Z
status: gaps_found
score: 12/13 must-haves verified
overrides_applied: 0
re_verification:
  previous_status: gaps_found
  previous_score: 11/13
  gaps_closed:
    - "R-9 root cause (CR-01): area_index is now canonical — CoverageWaypoint.msg adds uint32 area_index, PlanBuilder stamps it, dispatch_checkpoint_write uses last_wp.area_index not sequence_id, and last_mow_angle_deg propagates from PlanMetadata via BTContext"
  gaps_remaining:
    - "R-11 still fails: WR-01 (current_swath_index = plan-wide index, not per-area count) causes resume to skip the wrong number of swaths"
  regressions: []
gaps:
  - truth: "R-11 — resume_from_checkpoint=true continues at the open-swath endpoint within ≤ 5 cm + 5° of the persisted last_swath_endpoint"
    status: failed
    reason: |
      The area_index key is now correct (01-10/01-11 fix), so resume_ck is found by
      read_checkpoint_file. But the VALUE written under that key is still wrong.

      In dispatch_checkpoint_write (coverage_nodes.cpp:319-327), three Checkpoint fields
      are stamped with `completed_end_idx_exclusive - 1` — a GLOBAL PLAN-WIDE index, not
      a per-area swath counter:

        req->checkpoint.current_swath_index      = completed_end_idx_exclusive - 1;
        req->checkpoint.last_completed_swath_index = completed_end_idx_exclusive - 1;
        req->checkpoint.next_open_swath_index      = completed_end_idx_exclusive;

      PlanBuilder consumes current_swath_index as a per-area counter at line 326:
        mow_count < 2u * resume_ck->current_swath_index

      Concrete failure (synthetic plan [UNDOCK0, UNDOCK1, MOW2, MOW3, DOCK4, DOCK5]):
      - After completing the mowing pair, completed_end_idx_exclusive = 4.
      - BT writes current_swath_index = 3, next_open_swath_index = 4.
      - On resume, PlanBuilder skips `2 * 3 = 6` mowing waypoints.
      - The area has only 1 swath (2 waypoints) so both are skipped: area marked done.
      - Robot drives straight to RETURN_TO_DOCK. R-11's 5 cm / 5° resume guarantee
        is violated by construction for every plan that has any UNDOCK prefix — i.e.
        every real plan.

      For a larger area (say 5 swaths = 10 mowing waypoints, starting at global index 2):
      - After swath 1, completed_end_idx_exclusive = 4, current_swath_index = 3.
      - Resume skips 2*3=6 waypoints → skips swaths 1,2,3 (3 swaths) even though
        only swath 1 was mowed. Robot starts at swath 4, not swath 2.

      Additionally (WR-02): current_outline_index is set to 0 at line 319 and only
      overridden when last_wp.segment_type is OUTLINE_*. After the outline completes
      and the first mowing pair is dispatched, the checkpoint snaps current_outline_index
      back to 0 (< outline_passes=1), causing PlanBuilder to re-emit the full outline set
      on every charge cycle. Robot re-mows the lawn perimeter unnecessarily.

      The new WriteCheckpointAreaIndexTest tests assert only area_index and
      last_mow_angle_deg. They do NOT catch the swath-index or outline-index bugs.
    artifacts:
      - path: "ros2/src/mowgli_behavior/src/coverage_nodes.cpp"
        issue: "lines 319-327: current_swath_index, last_completed_swath_index, next_open_swath_index stamped with plan-wide waypoint index (completed_end_idx_exclusive-1), not per-area swath count. current_outline_index defaults to 0 and snaps back to 0 after first mowing pair completes (WR-02)."
      - path: "ros2/src/mowgli_behavior/test/test_coverage_nodes.cpp"
        issue: "WriteCheckpointAreaIndexTest::PersistsCanonicalAreaIndex asserts area_index=1 and last_mow_angle_deg=42.5 only. Does not assert current_swath_index == 1 (per-area count for a plan where the mowing pair starts at global index 2)."
    missing:
      - "Replace the three swath-index assignments with per-area counts: walk coverage_plan_[0..completed_end_idx_exclusive) restricted to area_index == last_wp.area_index and count MOWING_BOUSTROPHEDON pairs. Alternatively precompute a per_area_mow_pair_before_[i] lookup in onStart. See WR-01 fix sketch in 01-REVIEW.md."
      - "Fix current_outline_index: maintain a sticky per-area outline-passes-completed counter (or compute it by scanning coverage_plan_[0..completed_end_idx_exclusive) for OUTLINE_* waypoints in the same area). current_outline_index must NOT snap back to 0 after mowing begins."
      - "Add a regression test in test_coverage_nodes.cpp that asserts current_swath_index == 1 (not 3) for the build_synthetic_plan() fixture after dispatching completed_end_idx_exclusive=4."
deferred: []
human_verification:
  - test: "Hardware smoke test on Pi5 in Eichenau garden — plan generated, all 10 validation points pass, robot mows at least 1 complete strip without #61-class (drive-through obstacle outline) or #64-class (10 cm BackUp abort) failures"
    expected: |
      (1) Robot deployed via auto-deploy to Pi5; (2) operator places robot on dock and sends
      COMMAND_START via GUI; (3) coverage_planner_node accepts the goal and emits a non-empty
      sparse plan <= 500 waypoints; (4) all 10 validation points pass (visible in planner logs);
      (5) UNDOCK segment executes via Nav2 BackUp + costmap clear; (6) first MOWING_BOUSTROPHEDON
      strip completes end-to-end without LETHAL-cell BackUp abort and without any virtual-obstacle
      drive-through; (7) mow_session_monitor JSONL shows healthy cross-checks (RTK cov drop <=
      300 ms, fusion<->gps consistent, wheel<->gyro yaw drift bounded).
    why_human: |
      SPEC AC-13 requires physical Pi5 + RTK base + grass; cannot be simulated faithfully.
      Documented in 01-VALIDATION.md as the single pending row (01-09-T3) and intentionally
      operator-gated per 01-09-SUMMARY.md. The hardware test should be re-run AFTER R-11 is
      fixed (the swath-index WR-01 bug). Running hardware now: AC-13 (first strip mows
      correctly on a fresh dock-start) will likely pass, but a charge-cycle resume will silently
      resume at the wrong swath index.
---

# Phase 1: Coverage Planner Rewrite — Re-Verification Report

**Phase Goal:** Replace the existing pull-based strip planner with a new `coverage_planner_node` that emits a complete deterministic sequential `PoseStamped` waypoint plan via `PlanCoverage.action`, with metadata, YAML checkpoints, and pre-flight geometric validation. Plan includes Undock/Approach/Dock segments. BT follows Plan sequentially.

**Verified:** 2026-04-29T14:30:00Z
**Status:** gaps_found
**Re-verification:** Yes — after gap-closure plans 01-10 + 01-11 (commits c329a81b through 8d51b96a)

## Re-Verification Summary

Prior score: **11/13** (R-9 and R-11 both failed via CR-01 root cause).

Gap-closure work (01-10 + 01-11) **partially fixed** the root cause:

- **Fully resolved:** The canonical area_index key problem. `CoverageWaypoint.msg` now carries `uint32 area_index`; `PlanBuilder::stamp_and_push` stamps every per-area waypoint with the loop counter `idx`; `dispatch_checkpoint_write` reads `last_wp.area_index` (not `sequence_id`); `last_mow_angle_deg` is now `ctx->last_mow_angle_used_deg` (not hardcoded 0.0). The `.kv` file is written under the correct name and angle is correct.

- **Newly resolved: R-9** — `derive_mow_angle` now finds the file it wrote (`coverage_<real_area_index>.kv`) and increments correctly. The auto-rotate chain is end-to-end correct.

- **Still failing: R-11** — The swath-index VALUE written in the checkpoint is a plan-wide global index, not a per-area swath counter. PlanBuilder uses it as a per-area count to skip already-mowed swaths. Any plan with an UNDOCK prefix (every real plan) persists an inflated swath index that causes resume to skip the wrong number of swaths. See WR-01 in 01-REVIEW.md for the full analysis and fix sketch.

New score: **12/13**.

## Goal Achievement

### Observable Truths

| #   | Truth                                                                                              | Status     | Evidence                                                                                                                                                                                                                                                                                                                                 |
| --- | -------------------------------------------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| R-1 | A new `coverage_planner_node` package exists, builds, and exposes `/coverage_planner_node/plan_coverage` | ✓ VERIFIED | Package, launch wiring, and action server unchanged from prior verification. No regression. |
| R-2 | `PlanCoverage.action` accepts (start_pose, dock_pose, mow_angle_offset_deg, resume_from_checkpoint) and returns sparse `CoverageWaypoint[]` + `PlanMetadata` + `PlanError` | ✓ VERIFIED | Unchanged from prior verification. No regression. |
| R-3 | Sparse plan output: 50–500 waypoints for a 500 m² area; one pose per outline vertex / swath endpoint | ✓ VERIFIED | Unchanged from prior verification. No regression. |
| R-4 | Every waypoint carries `segment_type` ∈ {UNDOCK,TRANSIT,OUTLINE_WORKING_AREA,OUTLINE_OBSTACLE,MOWING_BOUSTROPHEDON,RETURN_TO_DOCK,DOCK_APPROACH,DOCKING}; blade rules respected | ✓ VERIFIED | Unchanged from prior verification. No regression. |
| R-5 | `PlanMetadata` includes mow_angle_used_deg, outline_passes_used, path_spacing_used, processed/skipped area indices, skip reasons, warnings, checkpoint_seed | ✓ VERIFIED | Unchanged from prior verification. Additionally confirmed `mow_angle_used_deg` is now read back by `PlanCoverageGoal::onRunning:151` into `ctx->last_mow_angle_used_deg`. |
| R-6 | Footprint-aware geometry uses 4-point rectangular footprint with drive_axis_offset; rejects plans where chassis overhangs into obstacle | ✓ VERIFIED | Unchanged from prior verification. No regression. |
| R-7 | Working-area outlines outside-in, obstacle outlines inside-out, both at outline_offset_robot + robot_width/2 | ✓ VERIFIED | Unchanged from prior verification. No regression. |
| R-8 | Boustrophedon AABB sweep with per-scan-line obstacle clipping; alternating directions; skip < robot_length segments via narrow-area strategy | ✓ VERIFIED | Unchanged from prior verification. No regression. |
| R-9 | When mow_angle_offset_deg = -1, planner derives new_angle = (last_completed_angle + angle_increment) mod 180° | ✓ VERIFIED | **Now fixed end-to-end by 01-10 + 01-11.** (1) `CoverageWaypoint.msg` has `uint32 area_index` stamped by PlanBuilder with the canonical loop index. (2) `dispatch_checkpoint_write` uses `last_wp.area_index` (not `sequence_id`) — confirmed at `coverage_nodes.cpp:318`. (3) `last_mow_angle_deg` propagates from `ctx->last_mow_angle_used_deg` which is populated from `PlanMetadata.mow_angle_used_deg` in `PlanCoverageGoal::onRunning:151`. (4) `derive_mow_angle` reads `coverage_<real_area_index>.kv` and now finds the file the BT wrote. Auto-rotate chain is end-to-end correct. Resolved by commits c329a81b (msg field), bd6fda58 (PlanBuilder stamp), 354066e1 (dispatch fix). |
| R-10| Per-area YAML/.kv checkpoints written atomically (temp + fsync + rename + dirfsync) | ✓ VERIFIED | Unchanged from prior verification. No regression. |
| R-11| Resume after charging: re-planning with resume_from_checkpoint=true continues at open swath within ≤ 5 cm + 5° of the persisted last_swath_endpoint | ✗ FAILED   | **Partially fixed: the area_index key is now correct** so `read_checkpoint_file` finds the file. But the swath index VALUE is still wrong. `dispatch_checkpoint_write` sets `current_swath_index = completed_end_idx_exclusive - 1` (a global plan index, e.g. 3 for waypoints [UNDOCK0,UNDOCK1,MOW2,MOW3]). `PlanBuilder:326` uses it as a per-area counter: `mow_count < 2u * resume_ck->current_swath_index` = `mow_count < 6`. For a 1-swath area this skips all 2 waypoints (area marked done). For a 5-swath area after swath 1 (global idx 2-3), resume skips 3 swaths instead of 1. The 5 cm / 5° resume guarantee cannot hold. (WR-01 in 01-REVIEW.md). Additionally `current_outline_index` snaps back to 0 after mowing begins, causing outlines to re-run on every charge cycle (WR-02). The new `WriteCheckpointAreaIndexTest` only validates `area_index` and `last_mow_angle_deg` — it does NOT catch either index bug. |
| R-12| Pre-flight validation against 10 SPEC points; structured `PlanError` on failure with all 7 user-facing error codes triggerable by tests | ✓ VERIFIED | Unchanged from prior verification. No regression. |
| R-13| Narrow-area handling: 3 strategies operator-selectable per area via `MapArea.narrow_area_strategy` and GUI dropdown | ✓ VERIFIED | Unchanged from prior verification. No regression. |

**Score:** 12/13 truths verified. R-11 still fails on the swath-index value (WR-01) and outline-index reset (WR-02).

### Required Artifacts

| Artifact                                                                         | Expected                                   | Status     | Details                                                                                                                                           |
| -------------------------------------------------------------------------------- | ------------------------------------------ | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ros2/src/mowgli_interfaces/msg/CoverageWaypoint.msg`                            | `uint32 area_index` field added            | ✓ VERIFIED | Field added at line 4 with UINT32_MAX sentinel documented for non-area waypoints. Commit c329a81b.                                                |
| `ros2/src/mowgli_coverage_planner/src/plan_builder/plan_builder.cpp`            | Stamps area_index on every per-area emit   | ✓ VERIFIED | `stamp_and_push` lambda at line 234-238 sets `wp.area_index = idx` (the loop counter). Dock/undock/return segments carry `kNoArea`. Commit bd6fda58. |
| `ros2/src/mowgli_behavior/src/coverage_nodes.cpp`                               | `dispatch_checkpoint_write` uses area_index| ✓ PARTIAL  | Line 318: `req->checkpoint.area_index = last_wp.area_index` — correct. Lines 319-327: `current_swath_index` = plan-wide index — still wrong (WR-01). Line 330: `last_mow_angle_deg = ctx->last_mow_angle_used_deg` — correct. |
| `ros2/src/mowgli_behavior/test/test_coverage_nodes.cpp`                         | Integration tests for checkpoint stamping  | ✓ PARTIAL  | `WriteCheckpointAreaIndexTest` (3 tests) correctly exercises the rclcpp service path and catches `area_index` and `last_mow_angle_deg` bugs. Does NOT assert `current_swath_index` — the WR-01 regression guard is missing. |
| `ros2/src/mowgli_coverage_planner/test/test_plan_builder_area_index.cpp`       | area_index stamped on all emit paths       | ✓ VERIFIED | Two-area fixture asserts `EXPECT_GT(mow_for_a0, 0u)` and `EXPECT_GT(mow_for_a1, 0u)`. Confirms no emit path forgot the field. Commit 36bbbb30 (RED) + bd6fda58 (GREEN). |

### Key Link Verification

| From | To | Via | Status | Details |
| ---- | -- | --- | ------ | ------- |
| `PlanCoverageGoal::onRunning` | `BTContext::last_mow_angle_used_deg` | `ctx->last_mow_angle_used_deg = wrapped.result->metadata.mow_angle_used_deg` | ✓ WIRED | `coverage_nodes.cpp:151`. New in 01-11, commit 5405fa60. |
| `dispatch_checkpoint_write` | `Checkpoint.area_index` | `last_wp.area_index` (from CoverageWaypoint stamped by PlanBuilder) | ✓ WIRED | `coverage_nodes.cpp:318`. Correct. |
| `dispatch_checkpoint_write` | `Checkpoint.current_swath_index` | `completed_end_idx_exclusive - 1` | ✗ WRONG-VALUE | Global plan index, not per-area swath count. `plan_builder.cpp:326` consumes as per-area. |
| `dispatch_checkpoint_write` | `Checkpoint.current_outline_index` | Conditional on last_wp.segment_type | ✗ WRONG-VALUE | Snaps to 0 after mowing begins (WR-02). `plan_builder.cpp:242` uses it as an outline-passes-done count. |
| `dispatch_checkpoint_write` | `Checkpoint.last_mow_angle_deg` | `ctx->last_mow_angle_used_deg` | ✓ WIRED | `coverage_nodes.cpp:330`. Correct. |
| `derive_mow_angle` | checkpoint file `coverage_<area_index>.kv` | `read_checkpoint_file(ctx.areas_dir, area_index)` | ✓ WIRED | `plan_builder.cpp:108`. Now finds the file the BT wrote (area_index key is correct). R-9 end-to-end path is complete. |
| `PlanBuilder resume` | checkpoint file `coverage_<idx>.kv` | `read_checkpoint_file(areas_dir_, idx)` | ✓ FOUND-BUT-WRONG-VALUE | File is found (area_index key correct). But `current_swath_index` in the file is a global plan index, not a per-area count. Resume skips wrong number of swaths. |

### Data-Flow Trace (Level 4) — Changed Paths Only

| Artifact | Data Variable | Source | Produces Real Data | Status |
| -------- | ------------- | ------ | ------------------ | ------ |
| Checkpoint file `coverage_<N>.kv` | `area_index` filename key | `last_wp.area_index` stamped by PlanBuilder's loop counter | ✓ CORRECT | Key now matches what `derive_mow_angle` and `PlanBuilder::build` read at `read_checkpoint_file(_, idx)`. |
| Checkpoint file `coverage_<N>.kv` | `last_mow_angle_deg` | `ctx->last_mow_angle_used_deg` ← `PlanMetadata.mow_angle_used_deg` | ✓ FLOWING | Auto-rotate chain is end-to-end correct. |
| Checkpoint file `coverage_<N>.kv` | `current_swath_index` | `completed_end_idx_exclusive - 1` (global plan index) | ✗ WRONG-VALUE | PlanBuilder reads as per-area count; produces wrong resume point for any plan with UNDOCK prefix. |
| Checkpoint file `coverage_<N>.kv` | `current_outline_index` | 0 (default) unless last_wp is OUTLINE_* | ✗ WRONG-VALUE | Snaps to 0 after mowing pair; causes full outline re-run on every charge cycle. |

### Behavioral Spot-Checks

Skipped — the verifier runs on macOS host outside the ROS2 devcontainer. All automated test evidence is from CI/colcon per plan SUMMARY files.

### Requirements Coverage

| Requirement | Source Plan(s) | Description | Status | Evidence |
| ----------- | -------------- | ----------- | ------ | -------- |
| R-9  | 01-07, 01-10, 01-11 | Mow-angle auto-rotation | ✓ SATISFIED | area_index key correct, last_mow_angle_deg correct, derive_mow_angle finds the file. |
| R-11 | 01-07, 01-10, 01-11 | Resume after charging | ✗ BLOCKED | area_index key correct, but swath/outline indices are plan-wide offsets, not per-area counts. resume skips wrong swaths. |

All other requirements (R-1 through R-8, R-10, R-12, R-13) remain satisfied — no regressions detected.

### Anti-Patterns Found (Delta from Prior Report)

The following anti-patterns from the prior report are **resolved:**

| Was | Resolution |
| --- | ---------- |
| `area_index = last_wp.sequence_id` (line 307, prior) | Fixed: now `last_wp.area_index` (line 318) |
| `last_mow_angle_deg = 0.0` (line 314, prior) | Fixed: now `ctx->last_mow_angle_used_deg` (line 330) |

The following anti-patterns are **newly confirmed** as blockers:

| File | Lines | Pattern | Severity | Impact |
| ---- | ----- | ------- | -------- | ------ |
| `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` | 319-327 | `current_swath_index = completed_end_idx_exclusive - 1` (plan-wide global index used as per-area swath count) | Blocker | Every real-plan resume skips the wrong number of swaths — R-11 violated by construction for all plans with UNDOCK prefix (WR-01). |
| `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` | 319, 333-338 | `current_outline_index` defaults to 0 and snaps back to 0 after first mowing pair completes | Blocker | Resume re-runs all outlines on every charge cycle — incorrect behavior observable on hardware (WR-02). |

The following anti-patterns from the prior report remain unchanged (warnings/info, not yet fixed):

| File | Pattern | Severity |
| ---- | ------- | -------- |
| `coverage_planner_node.cpp:251-381` | Cancellation not checked during plan build | Warning (WR-01 in prior report, now relabeled to avoid confusion with WR-01 in REVIEW) |
| `boustrophedon_sweeper.cpp:178,186` | `outline_passes - 1u` no unsigned underflow clamp | Warning |
| `coverage_nodes.cpp:75` | `resume_from_checkpoint = true` unconditional | Warning |
| `validator_pipeline.cpp:443-492` | PathSpacingValidator body empty | Warning |
| `validator_pipeline.cpp:180-219` | ObstacleCoverageValidator over-approximates | Warning |
| `coverage_planner_node.cpp:184` | `std::thread{}.detach()` captures `this` | Info |
| Various | See prior VERIFICATION.md anti-patterns table for full list | Info |

Additionally, a new warning from 01-REVIEW.md:

| File | Lines | Pattern | Severity | Impact |
| ---- | ----- | ------- | -------- | ------ |
| `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp:181` | `last_mow_angle_used_deg{0.0}` default | Warning (WR-03 in REVIEW) | 0.0 is a valid mow angle; if PlanCoverageGoal fails to populate the field before FollowCoveragePlan reads it, a real auto-rotate state is silently stomped. Recommend NaN or -1.0 sentinel. |

### Human Verification Required

1. **Hardware smoke test on Pi5 in Eichenau garden** (SPEC AC-13)

   **Test:** Deploy `coverage_planner_node` + `FollowCoveragePlan` BT to Pi5 via existing auto-deploy. Place robot on dock; send `COMMAND_START` via GUI. Run `mow_session_monitor` in parallel (per CLAUDE.md).

   **Expected:**
   - Plan generated; pre-flight validation passes (10/10).
   - UNDOCK segment executes via Nav2 BackUp + costmap clear.
   - First MOWING_BOUSTROPHEDON strip completes end-to-end without LETHAL-cell BackUp abort and without virtual-obstacle drive-through.
   - JSONL log shows healthy cross-checks (RTK cov drop ≤ 300 ms, fusion↔gps consistent, wheel↔gyro yaw drift bounded).

   **Why human:** Requires physical Pi5 + RTK base + grass; cannot be simulated. Documented in `01-VALIDATION.md` as the single pending row (01-09-T3).

   **Recommendation:** Run AFTER R-11 is fixed (WR-01 swath-index bug). A fresh dock-start without a prior charge cycle will succeed for AC-13 (first strip mows correctly). A charge-cycle resume will silently start at the wrong swath.

### Gaps Summary

**R-9 is now fully resolved.** The end-to-end chain — PlanBuilder stamps `area_index = loop_counter`, BT reads `last_wp.area_index` as the checkpoint key, `last_mow_angle_deg` flows from `PlanMetadata` via `BTContext` — is correct. `derive_mow_angle` now finds the file it looks for, and auto-rotate increments correctly across plans. Commits c329a81b + bd6fda58 + 5405fa60 + 354066e1 together close this gap cleanly.

**R-11 is still open.** The key repair from 01-10/01-11 (area_index key) means the planner now reads the checkpoint file the BT wrote. But the value stored in that file for `current_swath_index` is a plan-wide global waypoint index (`completed_end_idx_exclusive - 1`), not a per-area swath-pair count. `PlanBuilder::build:326` consumes it as a per-area count: `mow_count < 2u * resume_ck->current_swath_index`. For any plan with an UNDOCK prefix (every real plan starts with at least 2 UNDOCK waypoints), the persisted count is offset by the UNDOCK prefix length — causing resume to skip `UNDOCK_prefix / 2` more swaths than were actually mowed. In the degenerate case (single-swath area), the entire area is skipped and the robot returns to dock without mowing. In the general case, the robot resumes at the wrong swath endpoint, violating R-11's ≤ 5 cm + 5° guarantee.

The fix is to count per-area `MOWING_BOUSTROPHEDON` pairs in `coverage_plan_[0..completed_end_idx_exclusive)` restricted to `area_index == last_wp.area_index`, and use that count instead of the global plan index. The test regression guard (adding `EXPECT_EQ(captured_[0].current_swath_index, 1u)` to `PersistsCanonicalAreaIndex`) would have caught this before the plans were committed.

The hardware smoke test (AC-13) should be deferred until after the WR-01 fix.

---

_Verified: 2026-04-29T14:30:00Z_
_Verifier: Claude (gsd-verifier)_
_Re-verification after gap-closure plans 01-10 + 01-11_
