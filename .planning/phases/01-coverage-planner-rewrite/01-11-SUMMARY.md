---
phase: 01-coverage-planner-rewrite
plan: 11
subsystem: bt-coverage
tags: [ros2, mowgli_behavior, bt, checkpoint, cr-01, tdd, area_index, r-9, r-11]

# Dependency graph
requires:
  - 01-08-bt-coverage-nodes (FollowCoveragePlan + dispatch_checkpoint_write skeleton)
  - 01-10-area-index (CoverageWaypoint.area_index field + PlanBuilder stamping)
provides:
  - "BTContext::last_mow_angle_used_deg field (propagated from PlanCoverageGoal)"
  - "dispatch_checkpoint_write uses last_wp.area_index (canonical key — CR-01 BT-side fix)"
  - "dispatch_checkpoint_write early-returns on UINT32_MAX sentinel (T-11-03 mitigation)"
  - "dispatch_checkpoint_write populates Checkpoint.last_mow_angle_deg from BTContext (R-9/R-11)"
  - "3 WriteCheckpointAreaIndexTest cases: PersistsCanonicalAreaIndex, SkipsSentinelAreaIndex, DoesNotEchoSequenceIdAsAreaIndex"
affects:
  - coverage_planner_node write_checkpoint handler (receives correct area_index now)
  - derive_mow_angle (R-9 read-side sees correct last_mow_angle_deg on next plan)
  - PlanBuilder resume-snap (R-11 resume reads correct area .kv file)

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "kNoArea = UINT32_MAX early-return in dispatch_checkpoint_write: eliminates spurious coverage_4294967295.kv writes for dock/undock segments"
    - "SingleThreadedExecutor worker-thread + same-node client+server pattern: avoids same-process Cyclone DDS cross-node flakiness documented in 01-08 Deviation 4"
    - "BTContext scalar field propagation: PlanCoverageGoal writes ctx->last_mow_angle_used_deg immediately after ctx->coverage_plan = ... on the success path"

key-files:
  modified:
    - "ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp — double last_mow_angle_used_deg{0.0} added to coverage plan state block"
    - "ros2/src/mowgli_behavior/src/coverage_nodes.cpp — PlanCoverageGoal::onRunning propagates metadata.mow_angle_used_deg; dispatch_checkpoint_write rewritten (kNoArea guard + area_index + mow_angle)"
    - "ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp — coverage_plan_ and dispatch_checkpoint_write lifted from private to protected for test injection"
    - "ros2/src/mowgli_behavior/test/test_coverage_nodes.cpp — 3 new TEST_F cases + FollowCoveragePlanIntegrationTest subclass + WriteCheckpointAreaIndexTest fixture"

key-decisions:
  - "private->protected lift for coverage_plan_ and dispatch_checkpoint_write: protected not public — same T-11-06 mitigation rationale as setBladeEnabled in Plan 01-08"
  - "Same-node client+server (bt_node_ owns both): bypasses the Cyclone DDS intra-process shm flakiness that affected Plan 01-08's original service-round-trip attempt; the fixture exercises the full rclcpp client path without crossing node boundaries"
  - "kNoArea early-return placed BEFORE client creation: avoids creating the checkpoint_client_ lazy singleton for dock/undock segments — no DDS participant churn on non-area waypoints"
  - "current_swath_index / last_completed_swath_index use completed_end_idx_exclusive-1 (BT local index) instead of last_wp.sequence_id: monotonically increasing per dispatch, closer approximation for resume-snap skip logic than the global sequence_id was"

# Metrics
duration: 4min
completed: 2026-04-29
---

# Phase 1 Plan 11: dispatch_checkpoint_write BT-side fix (CR-01 half 2) Summary

**Fixed the three buggy lines in dispatch_checkpoint_write: area_index now comes from last_wp.area_index (canonical PlanBuilder key), mow_angle from BTContext::last_mow_angle_used_deg (propagated from PlanMetadata), and dock/undock segments are silently skipped via a UINT32_MAX sentinel early-return. Three in-process WriteCheckpoint integration tests cement all three contracts (canonical key, sentinel skip, no-sequence-id-echo). R-9 + R-11 are now end-to-end correct on the BT -> planner -> filesystem path.**

## Performance

- **Duration:** 4 min
- **Started:** 2026-04-29T10:14:19Z
- **Completed:** 2026-04-29T10:18:35Z
- **Tasks:** 2 (Task 1: BTContext field + PlanCoverageGoal propagation; Task 2: TDD RED + GREEN)
- **Commits:** 3 (Task 1 feat, Task 2 RED test, Task 2 GREEN impl)

## Accomplishments

### What was fixed

The original `dispatch_checkpoint_write` had three bugs:

1. `req->checkpoint.area_index = last_wp.sequence_id` — used the **global** sequence counter as the per-area .kv filename key. On a two-area plan the planner wrote `coverage_3.kv` instead of `coverage_1.kv`, so `derive_mow_angle` and the resume-snap could never find the file on the next plan goal (R-9 + R-11 failure).

2. `req->checkpoint.last_mow_angle_deg = 0.0` — hard-coded 0.0 regardless of what angle the planner actually used. `derive_mow_angle` (R-9) would always read 0.0 back and rotate to 90° even when the last plan had already optimized the angle.

3. No sentinel check — a dock/undock waypoint (area_index=UINT32_MAX) would trigger a service call to write `coverage_4294967295.kv`, which is both wasteful and a latent T-11-03 DoS vector.

### What was added

- `BTContext::last_mow_angle_used_deg{0.0}` — new scalar field in the coverage plan state block. Single-threaded BT executor, no mutex needed (same convention as `coverage_plan`).
- `PlanCoverageGoal::onRunning` success path: `ctx->last_mow_angle_used_deg = wrapped.result->metadata.mow_angle_used_deg` immediately after `ctx->coverage_plan = wrapped.result->plan`. Log line updated to report the stored value.
- `dispatch_checkpoint_write` rewrite:
  - `kNoArea = std::numeric_limits<std::uint32_t>::max()` sentinel defined locally (consistent with `plan_builder.cpp` from Plan 01-10)
  - Early-return before any lazy-client creation when `last_wp.area_index == kNoArea`
  - `req->checkpoint.area_index = last_wp.area_index` (canonical key)
  - `req->checkpoint.last_mow_angle_deg = ctx->last_mow_angle_used_deg` (real angle)
  - `current_swath_index` / `last_completed_swath_index` now use `completed_end_idx_exclusive - 1` (BT local index) instead of `last_wp.sequence_id`
- `<limits>` include added to `coverage_nodes.cpp` for `std::numeric_limits`.

### Test pattern

The `WriteCheckpointAreaIndexTest` fixture creates ONE `rclcpp::Node` that owns both the stub `WriteCheckpoint` service server AND the lazy checkpoint client that `dispatch_checkpoint_write` creates on first call. This is the key difference from the Plan 01-08 `setBladeEnabled` override pattern:

| 01-08 pattern | 01-11 pattern |
|---|---|
| Override at C++ level (`setBladeEnabled` virtual) | Exercise the full rclcpp client path |
| DDS edge NOT exercised (no service server) | DDS edge IS exercised (same-node server) |
| Tests the software contract inside BT | Tests the BT → service payload contract |
| Used for blade on/off (fire-and-forget MowerControl) | Used for WriteCheckpoint payload correctness |

The same-node pattern avoids the Cyclone DDS inter-node flakiness documented in 01-08 Deviation 4, because the transport layer is not crossed — Cyclone uses the intra-process shm fast path when client and server live on the same node.

The executor spins on a `std::thread` worker; `wait_for_captured()` polls with a 25 ms interval up to a 2 s deadline. `SkipsSentinelAreaIndex` uses a fixed 500 ms sleep (no service call expected).

## Task Commits

1. **Task 1: BTContext field + PlanCoverageGoal propagation** — `5405fa60` (feat)
2. **Task 2 RED: 3 failing WriteCheckpoint integration tests** — `d1361536` (test)
3. **Task 2 GREEN: dispatch_checkpoint_write canonical fix** — `354066e1` (feat)

## Deviations from Plan

None — plan executed exactly as written. All three acceptance criteria groups passed first time on macOS host-side verification.

## Verification Status

### Host-side (completed this plan)

- `grep -c 'last_mow_angle_used_deg' bt_context.hpp` → 1 PASS
- `grep -E 'double[[:space:]]+last_mow_angle_used_deg[[:space:]]*\{0\.0\}'` → match PASS
- `grep -F 'wrapped.result->metadata.mow_angle_used_deg'` → match PASS
- `grep -c 'TEST_F(WriteCheckpointAreaIndexTest'` → 3 PASS
- `grep -F 'rclcpp::Service<mowgli_interfaces::srv::WriteCheckpoint>'` → match PASS
- `grep -c 'blade_off_calls_'` → 2 PASS
- Regression guard: 0 occurrences of `last_wp.sequence_id` inside `dispatch_checkpoint_write` body PASS
- `last_wp.area_index` present in dispatch function body PASS
- `ctx->last_mow_angle_used_deg` present in dispatch function body PASS
- `last_wp.area_index == kNoArea` sentinel guard present PASS
- `coverage_plan_` and `dispatch_checkpoint_write` under `protected:` block in header PASS (2 matches)
- RED commit `d1361536` present PASS
- GREEN commit `354066e1` present PASS

### Deferred to docker/dev-branch CI

- `colcon build --packages-select mowgli_interfaces mowgli_behavior`
- `colcon test --packages-select mowgli_behavior --ctest-args -R test_coverage_nodes` — expects 7 tests (4 pre-existing + 3 new), 0 failures

Run command:
```bash
docker exec mowgli-ros2 bash -c '
  source /opt/ros/kilted/setup.bash && cd /ros2_ws &&
  colcon build --packages-select mowgli_interfaces mowgli_behavior \
    --event-handlers console_cohesion+ &&
  colcon test --packages-select mowgli_behavior \
    --ctest-args -R test_coverage_nodes \
    --event-handlers console_cohesion+'
```

### Post-plan recommendation

Re-run `/gsd-verify-phase 1` so `01-VERIFICATION.md` regenerates with R-9 and R-11 flipped from FAILED to VERIFIED. The hardware Pi5 smoke test (SPEC AC-13) is now safe to attempt — the broken auto-rotate and resume that the verification report warned about are fixed by Plans 01-10 + 01-11 together.

## Safety Notes

No changes affect physical blade behavior or emergency-stop paths. `dispatch_checkpoint_write` is a fire-and-forget RPC to the planner's filesystem helper — it has no effect on motor, blade, or navigation commands. The UINT32_MAX early-return reduces unnecessary DDS traffic on dock/undock segments (defense in depth for T-11-03).

## Threat Flags

None — all STRIDE threats in the plan's threat register are mitigated by the implementation:
- T-11-01 (Tampering — wrong area_index key): mitigated by `last_wp.area_index` + regression tests
- T-11-03 (DoS — UINT32_MAX filename): mitigated by kNoArea early-return + `SkipsSentinelAreaIndex` test
- T-11-06 (EoP — private→protected): mitigated by `protected` (not `public`) scope; no new public API surface

## Self-Check: PASSED

- `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp` — FOUND (last_mow_angle_used_deg)
- `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` — FOUND (kNoArea guard + area_index + mow_angle + propagation)
- `ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp` — FOUND (protected block with coverage_plan_ + dispatch_checkpoint_write)
- `ros2/src/mowgli_behavior/test/test_coverage_nodes.cpp` — FOUND (3 TEST_F + WriteCheckpointAreaIndexTest + FollowCoveragePlanIntegrationTest)
- Commit `5405fa60` (Task 1) — FOUND
- Commit `d1361536` (RED) — FOUND
- Commit `354066e1` (GREEN) — FOUND

---

*Phase: 01-coverage-planner-rewrite*
*Plan: 11*
*Completed: 2026-04-29*
