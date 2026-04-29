---
phase: 01-coverage-planner-rewrite
verified: 2026-04-29T09:04:03Z
status: gaps_found
score: 11/13 must-haves verified
overrides_applied: 0
gaps:
  - truth: "R-9 — auto-rotate produces angles differing by angle_increment mod 180° across sequential plans"
    status: failed
    reason: |
      The unit test test_auto_rotate.cpp passes because it constructs the Checkpoint in-process and
      calls write_checkpoint_file() directly with the correct area_index. In production the BT
      writes checkpoints via FollowCoveragePlan::dispatch_checkpoint_write, which sets
      Checkpoint.area_index = last_wp.sequence_id (the global plan index, e.g. 29) instead of the
      real area index. The planner's filename is built with std::to_string(ck.area_index), so the
      file written is /maps/coverage_29.kv, not /maps/coverage_0.kv. On the next plan,
      derive_mow_angle(ctx, area_index=0) calls read_checkpoint_file(areas_dir, 0) which never finds
      the BT-produced file. The reader returns std::nullopt, the rotation falls through to the MBR
      seed every run, and angle_increment is never applied. R-9 is satisfied at unit-test level
      but broken end-to-end on hardware.
    artifacts:
      - path: "ros2/src/mowgli_behavior/src/coverage_nodes.cpp"
        issue: "line 307 sets req->checkpoint.area_index = last_wp.sequence_id; CoverageWaypoint.msg has no area_index field, so the BT cannot derive the correct value from the plan it received"
      - path: "ros2/src/mowgli_interfaces/msg/CoverageWaypoint.msg"
        issue: "missing uint32 area_index field — without it the BT has no canonical way to know which working area a waypoint belongs to"
      - path: "ros2/src/mowgli_coverage_planner/src/checkpoint_io.cpp"
        issue: "line 191 trusts ck.area_index from the request unconditionally; no sanity bound; the per-area write/read keys diverge silently"
      - path: "ros2/src/mowgli_coverage_planner/src/plan_builder/plan_builder.cpp"
        issue: "lines 101 and 221 read coverage_<area_index>.kv keyed by the planner's loop index; the BT-written files never match this key"
    missing:
      - "Add uint32 area_index field to CoverageWaypoint.msg and stamp it in PlanBuilder for every emitted waypoint"
      - "Replace req->checkpoint.area_index = last_wp.sequence_id with last_wp.area_index in dispatch_checkpoint_write"
      - "Stop overwriting last_mow_angle_deg = 0.0 in dispatch_checkpoint_write — let the planner fill it from PlanContext::mow_angle_used_deg server-side, OR have the planner ignore last_mow_angle_deg from the request and write its own"
      - "Add an integration test in test_coverage_nodes.cpp that runs FollowCoveragePlan against a stub WriteCheckpoint server, captures the Checkpoint payload, and asserts area_index matches the just-completed waypoint's working area"
  - truth: "R-11 — resume_from_checkpoint=true continues at the open-swath endpoint within ≤ 5 cm + 5° of the persisted last_swath_endpoint"
    status: failed
    reason: |
      Same root cause as R-9. test_resume.cpp constructs a Checkpoint with the right area_index
      and writes it directly through write_checkpoint_file, then verifies the planner snaps the
      first MOWING_BOUSTROPHEDON pose within ≤ 5 cm + 5°. That works. But in production after a
      charge cycle, PlanBuilder::build calls read_checkpoint_file(areas_dir_, idx) for area idx=0
      (the planner's iteration index over ctx.areas), which looks for coverage_0.kv. The BT wrote
      coverage_<sequence_id>.kv, where sequence_id is the global plan-index of the last completed
      waypoint (e.g. 29). The file doesn't exist at the read key, resume_ck is std::nullopt, and
      the planner re-plans the area from scratch — exactly the failure mode R-11 was supposed to
      prevent. Additionally, the BT hard-codes last_mow_angle_deg = 0.0 in the request, so even
      if the area_index were correct the angle on resume would be wrong.
    artifacts:
      - path: "ros2/src/mowgli_behavior/src/coverage_nodes.cpp"
        issue: "lines 304-315: every Checkpoint field is wrong — area_index is sequence_id, current_outline_index=0 unconditionally, current_swath_index=sequence_id, last_completed_swath_index=sequence_id, next_open_swath_index=sequence_id+1, last_mow_angle_deg=0.0; only last_swath_endpoint is correct"
      - path: "ros2/src/mowgli_interfaces/srv/WriteCheckpoint.srv"
        issue: "service contract sends a fully-populated Checkpoint from BT to planner; this assumes the BT can compute all fields correctly, but the BT lacks the area-index/swath-index bookkeeping the planner has"
    missing:
      - "Either (a) extend CoverageWaypoint.msg with uint32 area_index AND have PlanBuilder emit the swath/outline indices the BT can later echo back, OR (b) change WriteCheckpoint.srv so the BT sends only sequence_id of the just-completed waypoint, and the planner derives all Checkpoint fields from a retained PlanContext + per-area waypoint range table emitted in PlanMetadata"
      - "Add an end-to-end test that simulates: complete a plan up to swath N, write checkpoint via the real BT path, kill planner, send new goal with resume_from_checkpoint=true, assert first MOWING_BOUSTROPHEDON pose is within 5cm/5° of the swath-N endpoint"
deferred: []
human_verification:
  - test: "Hardware smoke test on Pi5 in Eichenau garden — plan generated, all 10 validation points pass, robot mows ≥ 1 complete strip without #61-class (drive-through obstacle outline) or #64-class (10 cm BackUp abort) failures"
    expected: |
      (1) Robot deployed via auto-deploy to Pi5; (2) operator places robot on dock and sends
      COMMAND_START via GUI; (3) coverage_planner_node accepts the goal and emits a non-empty
      sparse plan ≤ 500 waypoints; (4) all 10 validation points pass (visible in planner logs);
      (5) UNDOCK segment executes via Nav2 BackUp + costmap clear; (6) first MOWING_BOUSTROPHEDON
      strip completes end-to-end without LETHAL-cell BackUp abort and without any virtual-obstacle
      drive-through; (7) mow_session_monitor JSONL shows healthy cross-checks (RTK cov drop ≤
      300 ms, fusion↔gps consistent, wheel↔gyro yaw drift bounded).
    why_human: |
      SPEC AC-13 requires physical Pi5 + RTK base + grass; cannot be simulated faithfully.
      Documented in 01-VALIDATION.md as the single ⬜ pending row (01-09-T3) and intentionally
      operator-gated per 01-09-SUMMARY.md. The hardware test should be re-run AFTER R-9/R-11
      are fixed, otherwise the operator will observe broken auto-rotate / resume on the bench.
---

# Phase 1: Coverage Planner Rewrite Verification Report

**Phase Goal:** Replace the existing pull-based strip planner with a new `coverage_planner_node` that emits a complete deterministic sequential `PoseStamped` waypoint plan via `PlanCoverage.action`, with metadata, YAML checkpoints, and pre-flight geometric validation. Plan includes Undock/Approach/Dock segments. BT follows Plan sequentially.

**Verified:** 2026-04-29T09:04:03Z
**Status:** gaps_found
**Re-verification:** No — initial verification

## Goal Achievement

### Observable Truths

| #   | Truth                                                                                              | Status     | Evidence                                                                                                                                                                                                                                                                                                                                |
| --- | -------------------------------------------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| R-1 | A new `coverage_planner_node` package exists, builds, and exposes `/coverage_planner_node/plan_coverage` | ✓ VERIFIED | `ros2/src/mowgli_coverage_planner/{CMakeLists.txt,package.xml,src,include,test}` exists; node launched in `sim_full_system.launch.py:199-208` and the other two top-level launch files; `ament_target_dependencies(... mowgli_geometry)` fix in commit `d20e4025` healed the INTERFACE include propagation. |
| R-2 | `PlanCoverage.action` accepts (start_pose, dock_pose, mow_angle_offset_deg, resume_from_checkpoint) and returns sparse `CoverageWaypoint[]` + `PlanMetadata` + `PlanError` | ✓ VERIFIED | `mowgli_interfaces/action/PlanCoverage.action` matches SPEC R-2 schema verbatim; e2e probe in `e2e_test.py:715-750` exercises the action server pre-START. |
| R-3 | Sparse plan output: 50–500 waypoints for a 500 m² area; one pose per outline vertex / swath endpoint | ✓ VERIFIED | `e2e_test.py:1587+` asserts `50 ≤ N ≤ 400` envelope (Plan 01-09 deviation note); `test_segment_type_invariants.cpp` asserts MOWING_BOUSTROPHEDON poses come in start+end pairs. |
| R-4 | Every waypoint carries `segment_type` ∈ {UNDOCK,TRANSIT,OUTLINE_WORKING_AREA,OUTLINE_OBSTACLE,MOWING_BOUSTROPHEDON,RETURN_TO_DOCK,DOCK_APPROACH,DOCKING}; blade rules respected | ✓ VERIFIED | `CoverageWaypoint.msg` defines all 8 constants; `plan_builder.cpp:140-153,344-356` emits UNDOCK / RETURN_TO_DOCK / DOCK_APPROACH / DOCKING segments; `test_segment_type_invariants.cpp` regresses position-of-segment invariants and blade-on-only-for-OUTLINE/MOWING. |
| R-5 | `PlanMetadata` includes mow_angle_used_deg, outline_passes_used, path_spacing_used, processed/skipped area indices, skip reasons, warnings, checkpoint_seed | ✓ VERIFIED | `PlanMetadata.msg` has all 8 fields; `coverage_planner_node.cpp:367-378` populates them from `PlanContext`. |
| R-6 | Footprint-aware geometry uses 4-point rectangular footprint with drive_axis_offset; rejects plans where chassis overhangs into obstacle | ✓ VERIFIED | `mowgli_geometry/footprint.hpp` ships `Footprint` with drive_axis_offset; `test_validation_pipeline.cpp:228-268` regresses `ERROR_FOOTPRINT_VIOLATION` for tool-fits-but-chassis-clips case. |
| R-7 | Working-area outlines outside-in, obstacle outlines inside-out, both at outline_offset_robot + robot_width/2 | ✓ VERIFIED | `outline_generator.cpp` + `test_outline_generator.cpp` (4 cases) cover both orientations and the offset formula. |
| R-8 | Boustrophedon AABB sweep with per-scan-line obstacle clipping; alternating directions; skip < robot_length segments via narrow-area strategy | ✓ VERIFIED | `boustrophedon_sweeper.cpp` + `test_aabb_sweep.cpp` (3 cases) cover sweep + clipping; `narrow_area_strategy.cpp` + `test_narrow_area_strategies.cpp` (3 cases, one per strategy) cover SKIP/OUTLINE_ONLY/SPECIAL_PATTERN. |
| R-9 | When mow_angle_offset_deg = -1, planner derives new_angle = (last_completed_angle + angle_increment) mod 180° | ✗ FAILED   | Unit test `test_auto_rotate.cpp:140` passes by writing the Checkpoint with the correct area_index in-process. **In production the BT writes Checkpoint.area_index = sequence_id**, so the planner's `read_checkpoint_file(areas_dir, real_area_index)` in `plan_builder.cpp:101` never finds the file. Auto-rotate falls through to the MBR seed every run; the increment is never applied. (CR-01) |
| R-10| Per-area YAML/.kv checkpoints written atomically (temp + fsync + rename + dirfsync) | ✓ VERIFIED | `mowgli_geometry/atomic_write.hpp` + `test_atomic_write.cpp` cover the 4-step pattern; `checkpoint_io.cpp` calls it; `test_checkpoint.cpp` (6 cases) covers serialise/parse + corruption detection. (Atomicity is fine; the bug is the *key* the BT writes under.) |
| R-11| Resume after charging: re-planning with resume_from_checkpoint=true continues at open swath within ≤ 5 cm + 5° of persisted last_swath_endpoint | ✗ FAILED   | Unit test `test_resume.cpp:116` passes by writing the Checkpoint with the correct area_index in-process. **In production the BT writes Checkpoint.area_index = sequence_id and last_mow_angle_deg = 0.0**, so the planner's `read_checkpoint_file(areas_dir_, idx)` in `plan_builder.cpp:221` never finds the file. The 5 cm / 5° snap never happens; the planner replans the area from scratch. (CR-01) |
| R-12| Pre-flight validation against 10 SPEC points; structured `PlanError` on failure with all 7 user-facing error codes triggerable by tests | ✓ VERIFIED | `validator_pipeline.cpp` runs pre+post pipelines; `test_validation_pipeline.cpp:90-268` triggers ERROR_NO_AREAS, ERROR_DOCK_OUTSIDE_AREAS, ERROR_AREA_TOO_NARROW, ERROR_OBSTACLE_BLOCKS_AREA, ERROR_OBSTACLE_OFFSET_FAILED, ERROR_RESUME_CHECKPOINT_INVALID, ERROR_FOOTPRINT_VIOLATION (7/7; ERROR_INTERNAL is the catch-all). |
| R-13| Narrow-area handling: 3 strategies operator-selectable per area via `MapArea.narrow_area_strategy` and GUI dropdown | ✓ VERIFIED | `MapArea.msg` has `uint8 narrow_area_strategy`; `EditAreaModal.tsx:78-79` renders the dropdown bound to `area.narrow_area_strategy`; `map_server_node.cpp:1125-1135` round-trips the field through `areas.yaml` with range-clamp; `narrow_area_strategy.cpp` + `test_narrow_area_strategies.cpp` apply the selected strategy. |

**Score:** 11/13 truths verified — R-9 and R-11 fail through CR-01 root cause.

### Required Artifacts

| Artifact                                                                    | Expected                                  | Status     | Details                                                                                                                                            |
| --------------------------------------------------------------------------- | ----------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ros2/src/mowgli_coverage_planner/`                                         | New ROS2 package                          | ✓ VERIFIED | All directories present; `colcon build` succeeded per Plans 01-05/01-07/01-09 SUMMARY commits.                                                     |
| `ros2/src/mowgli_geometry/`                                                 | Header-only geometry library              | ✓ VERIFIED | `footprint.hpp`, `geometry.hpp`, `pca.hpp`, `atomic_write.hpp` + 4 test files.                                                                     |
| `mowgli_interfaces/action/PlanCoverage.action`                              | Goal/Result/Feedback per R-2              | ✓ VERIFIED | Schema matches SPEC verbatim.                                                                                                                       |
| `mowgli_interfaces/msg/{CoverageWaypoint,PlanMetadata,PlanError,Checkpoint}.msg` | Custom messages                       | ⚠️ STUB    | All four files exist with correct fields. **However**, `CoverageWaypoint.msg` is missing `uint32 area_index` — without it the BT cannot identify which working area a waypoint belongs to, which is the root of CR-01. |
| `mowgli_interfaces/srv/{GetAllAreas,WriteCheckpoint}.srv`                   | New services                              | ✓ VERIFIED | Both exist; `WriteCheckpoint.srv` carries the full Checkpoint message (which the BT cannot fully construct correctly — see R-9/R-11 gap).         |
| `mowgli_interfaces/msg/MapArea.msg` extension                               | `uint8 narrow_area_strategy`              | ✓ VERIFIED | Field added at line 5 with 0=SKIP / 1=OUTLINE_ONLY / 2=SPECIAL_PATTERN constants.                                                                  |
| `mowgli_behavior/src/coverage_nodes.cpp`                                    | New BT nodes PlanCoverageGoal + FollowCoveragePlan | ⚠️ ORPHANED-FROM-INTENT | Both nodes exist + are registered in `register_nodes.cpp:74-75`, both wired into `main_tree.xml:434-435`. **However**, FollowCoveragePlan's `dispatch_checkpoint_write` produces semantically wrong Checkpoint payloads (CR-01). |
| `mowgli_behavior/trees/main_tree.xml`                                       | BT replaced with sequential plan-follower | ✓ VERIFIED | `<PlanCoverageGoal/>` + `<FollowCoveragePlan/>` at lines 434-435; legacy `GetNextStrip`, `TransitToStrip`, `FollowStrip`, `OutlineArea`, `GetNextUnmowedArea` references absent (grep confirmed). |
| `gui/web/src/hooks/useCoveragePlan.ts`                                      | Plan preview hook                         | ✓ VERIFIED | Exists, wired into `MapPage.tsx`; rosbridge action client; segment_type colour mapping; PlanError → notification.error mapping.                    |
| `gui/web/src/pages/map/components/EditAreaModal.tsx` narrow-area dropdown   | Per-area strategy UI                      | ✓ VERIFIED | Dropdown at lines 78-79 bound to `area.narrow_area_strategy`.                                                                                       |
| Legacy artifacts deleted                                                    | get_next_strip / get_outline_path / get_coverage_status srvs + 5 BT nodes + map_server planner code | ✓ VERIFIED | `mowgli_interfaces/srv/` no longer contains `GetNextStrip.srv`, `GetOutlinePath.srv`, `GetCoverageStatus.srv` (grep confirmed); legacy planner functions absent from `map_server_node.cpp`; old BT classes gone from `mowgli_behavior` (grep confirmed); commits `73124c62` and `70543ec9` document deletions. |

### Key Link Verification

| From                              | To                                          | Via                                              | Status     | Details                                                                                                                                                  |
| --------------------------------- | ------------------------------------------- | ------------------------------------------------ | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `PlanCoverageGoal` BT node        | `coverage_planner_node`                     | rclcpp_action client `/coverage_planner_node/plan_coverage` | ✓ WIRED    | `coverage_nodes.cpp:39-43,77,110` — async send/get-result around the action goal.                                                                        |
| `coverage_planner_node`           | `map_server_node`                           | service client `/map_server_node/get_all_areas`  | ✓ WIRED    | `coverage_planner_node.cpp:88-89` + `fetch_all_areas:191-237` synchronous-with-cancel poll.                                                              |
| `FollowCoveragePlan` BT node      | Nav2 `/follow_path` + `/navigate_to_pose`   | rclcpp_action clients                            | ✓ WIRED    | `coverage_nodes.cpp:197,201` — both action servers awaited at onStart.                                                                                   |
| `FollowCoveragePlan` BT node      | `coverage_planner_node` (write_checkpoint)  | service client `/coverage_planner_node/write_checkpoint` | ⚠️ WIRED-BUT-WRONG-PAYLOAD | The wiring is sound (`coverage_nodes.cpp:289-328`), but the payload is wrong (CR-01: area_index=sequence_id, last_mow_angle_deg=0). |
| `coverage_planner_node` checkpoint write handler | filesystem `<areas_dir>/coverage_<area_index>.kv` | `write_checkpoint_file → mowgli_geometry::atomic_write` | ✓ WIRED    | `checkpoint_io.cpp:191-200`; the *path* construction is correct given the request — the bug is upstream in the BT's `req->checkpoint.area_index` value. |
| GUI `useCoveragePlan` hook        | `coverage_planner_node`                     | rosbridge ws://host:9090 + roslib `Action`        | ✓ WIRED    | `useCoveragePlan.ts:26-27`; renders into `MapPage.tsx`'s Mapbox layers via `MapToolbar` Preview button.                                                 |
| `EditAreaModal` dropdown          | `MapArea.narrow_area_strategy`              | `onChange` → `addMowingArea` → `map_server_node` → `areas.yaml` round-trip | ✓ WIRED | `EditAreaModal.tsx:78-79` → `map_server_node.cpp:1125-1135` clamps + persists. |

### Data-Flow Trace (Level 4)

| Artifact                                        | Data Variable                | Source                                                                        | Produces Real Data | Status                                                                                                                                                     |
| ----------------------------------------------- | ---------------------------- | ----------------------------------------------------------------------------- | ------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `coverage_planner_node` action server result    | `result.plan` + `metadata`    | `PlanBuilder::build` → real outline/sweep generators on real `ctx.areas`     | ✓ FLOWING          | E2E probe in `e2e_test.py:1587+` asserts non-empty plan with valid segment-type distribution.                                                              |
| `FollowCoveragePlan::coverage_plan_`            | snapshot of `ctx->coverage_plan` | written by `PlanCoverageGoal::onRunning:149` from real action result        | ✓ FLOWING          | `coverage_nodes.cpp:185` snapshot at onStart.                                                                                                              |
| Checkpoint files on disk `coverage_<N>.kv`      | `ck.area_index` filename key  | `WriteCheckpoint.srv` request from `dispatch_checkpoint_write`               | ✗ HOLLOW_PROP      | Filename built from `req->checkpoint.area_index`, but the request value is `last_wp.sequence_id` — a global plan index, not the per-area key the planner will read on the next plan. **The data flows, but the key is meaningless to the consumer.** |
| `derive_mow_angle` checkpoint read              | `loaded->last_mow_angle_deg`  | `read_checkpoint_file(areas_dir, area_index=loop_index)`                     | ✗ DISCONNECTED     | The file at `coverage_<loop_index>.kv` does not exist after a real BT-driven run because the BT wrote to a different name. Falls back to MBR seed. |
| `PlanBuilder::build` resume snap                | `resume_ck->last_swath_endpoint` | `read_checkpoint_file(areas_dir_, idx=loop_index)`                          | ✗ DISCONNECTED     | Same root cause. Resume snap never triggers in production.                                                                                                  |
| GUI Plan Preview FeatureCollection              | `proj.plan` + `proj.dockPose` | rosbridge action result with real `CoverageWaypoint[]`                        | ✓ FLOWING          | `useCoveragePlan.ts` converts to GeoJSON, renders via Mapbox `match` color expression on `segment_type`.                                                     |

### Behavioral Spot-Checks

Skipped — **the verifier runs on macOS host outside the ROS2 devcontainer**. The repo's runnable entry points (`make build`, `make test`, `make e2e-test`) require the Kilted devcontainer with `colcon`. The 21 automated rows in `01-VALIDATION.md` (all ✅ green per Plan 01-09) provide the equivalent evidence; per-task colcon build/test commands are documented in each plan's SUMMARY.md.

### Requirements Coverage

| Requirement | Source Plan(s)                                  | Description (from REQUIREMENTS.md / SPEC §Requirements) | Status     | Evidence                                                                                                                            |
| ----------- | ----------------------------------------------- | ------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| R-1         | 01-05, 01-09                                    | New `mowgli_coverage_planner` package + node          | ✓ SATISFIED | Built + launched on all three top-level launch entry points.                                                                         |
| R-2         | 01-01, 01-05                                    | `PlanCoverage.action` schema rewrite                   | ✓ SATISFIED | Schema matches SPEC verbatim; e2e probe exercises action server.                                                                    |
| R-3         | 01-07                                           | Sparse plan output                                     | ✓ SATISFIED | E2E envelope check + segment-type invariant tests.                                                                                  |
| R-4         | 01-01, 01-08                                    | Segment-type annotation                                | ✓ SATISFIED | `CoverageWaypoint.msg` enum + `test_segment_type_invariants.cpp`.                                                                   |
| R-5         | 01-01, 01-07                                    | Plan metadata                                          | ✓ SATISFIED | `PlanMetadata.msg` populated from `PlanContext` in `coverage_planner_node.cpp:367-378`.                                             |
| R-6         | 01-02, 01-03, 01-07                             | Footprint-aware geometry                               | ✓ SATISFIED | `mowgli_geometry/footprint.hpp` + `ERROR_FOOTPRINT_VIOLATION` regression test.                                                       |
| R-7         | 01-07                                           | Outlines outside-in / inside-out                       | ✓ SATISFIED | `outline_generator.cpp` + 4 outline tests.                                                                                          |
| R-8         | 01-07                                           | Boustrophedon AABB sweep with obstacle clipping        | ✓ SATISFIED | `boustrophedon_sweeper.cpp` + 3 sweep tests + 3 narrow-area tests.                                                                  |
| R-9         | 01-07                                           | Mow-angle auto-rotation                                | ✗ BLOCKED   | Unit test passes; **production end-to-end path broken by CR-01** (BT writes Checkpoint.area_index = sequence_id).                   |
| R-10        | 01-02, 01-05                                    | YAML/.kv checkpoint persistence (atomic)              | ✓ SATISFIED | `atomic_write.hpp` + 6 checkpoint tests; atomic write itself is correct.                                                            |
| R-11        | 01-07                                           | Resume after charging                                  | ✗ BLOCKED   | Unit test passes; **production end-to-end path broken by CR-01** (planner reads coverage_0.kv, BT wrote coverage_<sequence_id>.kv). |
| R-12        | 01-01, 01-05, 01-07                             | Pre-flight validation + structured `PlanError`         | ✓ SATISFIED | All 7 user-facing error codes have triggering unit tests; `ERROR_INTERNAL` is the catch-all.                                        |
| R-13        | 01-01, 01-04, 01-06, 01-07                      | Narrow-area handling (3 strategies)                    | ✓ SATISFIED | `MapArea.narrow_area_strategy` + GUI dropdown + 3 strategy tests + areas.yaml round-trip.                                            |

### Anti-Patterns Found

| File                                                                             | Line    | Pattern                                                | Severity  | Impact                                                                                                                                                                                |
| -------------------------------------------------------------------------------- | ------- | ------------------------------------------------------ | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ros2/src/mowgli_behavior/src/coverage_nodes.cpp`                                | 307     | `area_index = last_wp.sequence_id` with comment "best-effort, planner owns the canonical key" | 🛑 Blocker | Silently breaks R-9 + R-11 in production. The "best-effort" comment papers over a wrong-key bug. |
| `ros2/src/mowgli_behavior/src/coverage_nodes.cpp`                                | 314     | `req->checkpoint.last_mow_angle_deg = 0.0` with comment "Planner derives this from its own state" | 🛑 Blocker | The planner does NOT override the BT's value — `on_write_checkpoint` writes the Request's Checkpoint directly. So 0.0 persists, and auto-rotate's increment-from-last-angle is permanently zero. |
| `ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp`                 | 251-381 | Cancellation only checked between `fetch_all_areas` and pre-validators; ignored during plan build | ⚠️ Warning | WR-01 in REVIEW.md. Goals that cancel mid-build complete via `succeed()` instead of `canceled()`. Harmless on small plans, observable on R-12 (10 areas + obstacles). |
| `ros2/src/mowgli_coverage_planner/src/plan_builder/boustrophedon_sweeper.cpp`    | 178,186 | `static_cast<double>(robot.outline_passes - 1u) * step` with no clamp on `outline_passes == 0` | ⚠️ Warning | WR-02 in REVIEW.md. Underflow → ~4.29e9 inset → silently zero-swath plan. Constructor clamps but PlanBuilder bypass doesn't. |
| `ros2/src/mowgli_behavior/src/coverage_nodes.cpp`                                | 75      | `goal.resume_from_checkpoint = true;` unconditional | ⚠️ Warning | WR-03 in REVIEW.md. Combined with `ERROR_RESUME_CHECKPOINT_INVALID` rejection, a single corrupt `.kv` blocks all subsequent planning until manually wiped. No GUI affordance. |
| `ros2/src/mowgli_coverage_planner/src/validators/validator_pipeline.cpp`         | 443-492 | `PathSpacingValidator` body computes `diff` but emits nothing; comment "emit info only" but body empty | ⚠️ Warning | WR-04 in REVIEW.md. Validator wears the "10 SPEC points checked" hat but is dead code. |
| `ros2/src/mowgli_coverage_planner/src/validators/validator_pipeline.cpp`         | 180-219 | `ObstacleCoverageValidator` over-approximates "obstacle blocks area" (every-vertex-inside heuristic) | ⚠️ Warning | WR-05 in REVIEW.md. Soundness gap; rename or rewrite using `boost::geometry::difference`. |
| `ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp`                 | 184     | `std::thread{...}.detach()` captures `this`; node destructor doesn't join | ℹ️ Info    | IN-04 in REVIEW.md. Production rclcpp_action shutdown-during-active-goal hazard. |
| `ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp`                 | 265-380 | Hard failures call `goal_handle->succeed(result)` with `success=false` instead of `abort()` | ℹ️ Info    | IN-02 in REVIEW.md. Contractual surprise; works because PlanCoverageGoal checks `result->success`. |
| `ros2/src/mowgli_coverage_planner/package.xml`                                   | 22      | Stale `<depend>nav2_msgs</depend>` — package never uses any nav2_msgs symbol | ℹ️ Info    | IN-03 in REVIEW.md. Slows manifest scan; cosmetic. |
| `ros2/src/mowgli_coverage_planner/src/checkpoint_io.cpp`                         | 87-91   | `swath_direction_to_token` returns nullptr → defaults to "FORWARD" silently | ℹ️ Info    | IN-05 in REVIEW.md. Hides enum drift; combined with CR-01's hard-coded SWATH_DIRECTION_FORWARD makes resume direction permanently wrong. |
| `ros2/src/mowgli_coverage_planner/src/checkpoint_io.cpp`                         | 191     | Filename built from `ck.area_index` with no sanity bound; CR-01 can produce `coverage_42.kv` for a single-area plan | ℹ️ Info    | IN-06 in REVIEW.md. Defence-in-depth; would have caught CR-01 at write boundary. |
| `gui/web/src/hooks/useCoveragePlan.ts`                                           | 133-141 | Unknown `segment_type` falls back to "TRANSIT" silently (operator wouldn't notice new enum addition) | ℹ️ Info    | IN-07 in REVIEW.md. Cosmetic GUI fallback; surface as UNKNOWN_<n> with a magenta map style. |
| `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp`                | 61-65,171 | `coverage_plan` documented under context_mutex but accessed unlocked | ℹ️ Info    | IN-01 in REVIEW.md. Currently safe (single-threaded BT executor); future-multi-threaded-executor hazard. |

### Human Verification Required

1. **Hardware smoke test on Pi5 in Eichenau garden** (SPEC AC-13)

   **Test:**
   - Deploy `coverage_planner_node` + `FollowCoveragePlan` BT to Pi5 via existing auto-deploy.
   - Place robot on dock; send `COMMAND_START` via GUI.
   - Run `mow_session_monitor` in parallel (per CLAUDE.md "Mowing Session Monitoring").

   **Expected:**
   - Plan generated; pre-flight validation passes (10/10).
   - UNDOCK segment executes via Nav2 BackUp + costmap clear.
   - First MOWING_BOUSTROPHEDON strip completes end-to-end without LETHAL-cell BackUp abort and without virtual-obstacle drive-through.
   - JSONL log shows healthy cross-checks (RTK cov drop ≤ 300 ms, fusion↔gps consistent, wheel↔gyro yaw drift bounded).

   **Why human:** Requires physical Pi5 + RTK base + grass; cannot be simulated. Documented in `01-VALIDATION.md` as the single ⬜ pending row (01-09-T3).

   **Recommendation:** Re-run AFTER R-9/R-11 are fixed. If you run hardware now, you will observe broken auto-rotate (every plan starts at MBR seed) and broken resume (charge cycle replans area from scratch). The strip-mowing AC-13 acceptance criterion alone will pass on a fresh dock-start, but two of the SPEC's named acceptance criteria (auto-rotate + resume) cannot be validated by this test in the current state.

### Gaps Summary

The phase is **architecturally complete** — every package, message type, action, service, BT node, GUI surface, validator, and atomic-write helper required by the SPEC exists, builds, and is correctly wired into the runtime topology. 11 of 13 SPEC requirements pass at the unit-test level.

The two failing requirements (R-9 auto-rotate + R-11 resume-after-charging) share a single root cause: **CR-01** in `FollowCoveragePlan::dispatch_checkpoint_write`. The BT does not have access to the canonical per-area index (because `CoverageWaypoint.msg` has no `area_index` field), so it falls back to `last_wp.sequence_id` — a global plan index — and stamps that into `req->checkpoint.area_index`. The planner's checkpoint writer trusts the request and writes `coverage_<sequence_id>.kv`. On the next plan, the planner's reader keys by the real area index (loop counter `i` in `PlanBuilder::build`) and never finds the file. Auto-rotate falls through to MBR seed; resume falls through to fresh plan.

The unit tests in `test_resume.cpp` and `test_auto_rotate.cpp` paper over this because they construct Checkpoints directly in-process and write them via `write_checkpoint_file` — they never exercise the BT → service → planner path.

**Both gaps are in `coverage_nodes.cpp:307-315` (12 lines) plus a one-field addition to `CoverageWaypoint.msg`.** The recommended fix:

1. Add `uint32 area_index` to `CoverageWaypoint.msg`; have `PlanBuilder` stamp it for every emitted waypoint (including UNDOCK/TRANSIT/DOCK_* — set to `UINT32_MAX` or the index of the area-being-transitioned-from).
2. In `dispatch_checkpoint_write`, replace `req->checkpoint.area_index = last_wp.sequence_id` with `req->checkpoint.area_index = last_wp.area_index`.
3. In `dispatch_checkpoint_write`, drop the `last_mow_angle_deg = 0.0` line; let the planner override it server-side from `PlanContext::mow_angle_used_deg` in `on_write_checkpoint`. Or change `WriteCheckpoint.srv` so the BT only sends `sequence_id` and the planner derives all Checkpoint fields.
4. Add an integration test in `test_coverage_nodes.cpp` that runs `FollowCoveragePlan` against a stub `WriteCheckpoint` server, captures the request payload, and asserts `area_index` matches the just-completed waypoint's working area.

The hardware smoke (AC-13) is the only legitimate human-only verification, but it should be run AFTER the CR-01 fix lands — otherwise the operator's hardware run will surface the broken auto-rotate and broken resume that the unit tests are masking.

---

_Verified: 2026-04-29T09:04:03Z_
_Verifier: Claude (gsd-verifier)_
