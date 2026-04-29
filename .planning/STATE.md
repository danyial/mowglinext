---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: hardware-smoke-pending
stopped_at: 01-09 automatable scope complete; SPEC AC-13 Pi5 garden smoke pending
last_updated: "2026-04-29T09:30:00.000Z"
progress:
  total_phases: 1
  completed_phases: 0
  total_plans: 9
  completed_plans: 8
  percent: 95
---

# Project state

## Current phase

1 — Coverage Planner Rewrite (Waves 1-4 complete + Wave 5 automatable scope complete; SPEC AC-13 Pi5 hardware smoke pending operator verification)

## Current Plan

09 — E2E sim + Pi5 hardware smoke (AUTOMATABLE SCOPE COMPLETE — committed `d20e4025`, `08ae7808`, `5e0683b6`; SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-09-SUMMARY.md`. Hardware checkpoint Task 3 is operator-gated and remains ⬜ pending.)

## Total Plans

9

## Resume point

- **Last completed step:** Plan 01-09 automatable scope (T0 precondition fix + T1 launch+e2e + T2 VALIDATION populate). SUMMARY committed. Build precondition healed (mowgli_geometry INTERFACE export propagation fixed in mowgli_coverage_planner consumer).
- **Next step:** SPEC AC-13 hardware smoke on Pi5 in Eichenau garden (operator-gated — see `.planning/phases/01-coverage-planner-rewrite/01-09-SUMMARY.md` § "Hardware Checkpoint Procedure").
- **Auto-chain flag persisted:** yes (`workflow._auto_chain_active=true` in `.planning/config.json`)
- **Wave 1 plans:** 01-01 ✅ COMPLETE, 01-03 ✅ COMPLETE
- **Wave 2 plans:** 01-02 ✅ COMPLETE, 01-04 ✅ COMPLETE
- **Wave 3 plans:** 01-05 ✅ COMPLETE, 01-06 ✅ COMPLETE
- **Wave 4 plans:** 01-07 ✅ COMPLETE, 01-08 ✅ COMPLETE
- **Wave 5 plans:** 01-09 🟡 AUTOMATABLE COMPLETE (T0+T1+T2 ✅; T3 Pi5 hardware smoke ⬜ pending operator verification)

## Last session

- **Last session:** 2026-04-29T09:30:00.000Z
- **Stopped at:** Completed 01-09 automatable scope (T0+T1+T2); SPEC AC-13 Pi5 garden smoke pending
- **Resume file:** `.planning/phases/01-coverage-planner-rewrite/01-09-SUMMARY.md` § "Hardware Checkpoint Procedure"
- **Blockers:** SPEC AC-13 — operator must execute the Pi5 Eichenau garden smoke (procedure documented in 01-09-SUMMARY.md). Until then, Phase 1 remains in "automatable complete, hardware-verified pending" state.

## Performance Metrics

| Phase | Plan | Duration | Tasks | Files |
|-------|------|----------|-------|-------|
| 01    | 01   | 12min    | 3     | 18    |
| 01    | 03   | 3min     | 2     | 2     |
| 01    | 02   | 9min     | 2     | 11    |
| 01    | 04   | 15min    | 2     | 7     |
| 01    | 05   | 8min     | 2     | 11    |
| 01    | 06   | 25min    | 2     | 10    |
| 01    | 07   | 20min    | 2     | 22    |
| 01    | 08   | 34min    | 2     | 13    |
| 01    | 09   | 25min    | 3 (T0+T1+T2; T3 pending) | 7 |

## Active branch

`migrate/upstream-localization` (HEAD `c778e40c` at .planning bootstrap time)

## Locked architectural decisions

These were locked in chat on 2026-04-28 before `/gsd-spec-phase` started — the spec-phase interview must NOT re-litigate them, only refine details below them.

1. **New separate ROS2 node:** `coverage_planner_node` lives in a new package (or in `mowgli_map`), parallel to `map_server_node`. The legacy strip planner inside `map_server_node` keeps running until the new planner is hardware-verified.
2. **Action-based delivery:** `PlanCoverage.action` (rclcpp_action) — supports feedback during long planning, supports cancellation, supports resume.
3. **Sequential plan execution by BT:** the BT consumes the plan as a sequential `PoseStamped` waypoint list. Lokale dynamische Hindernisse werden via `collision_monitor` + Nav2 lokalem Planer umfahren — KEIN globales Replan.
4. **Iteration 1 = simple AABB sweep with obstacle clipping** (footprint-aware). No BCD in this phase.
5. **Robot footprint = rectangular 4-point polygon with `drive_axis_offset`** as the rotation centre. All collision checks use the footprint, not the tool plate.
6. **Checkpoint persistence = atomic YAML file** next to `mow_progress.png`, written at the end of each completed swath.

## Open milestones (parking lot)

- Localization migration Phase 3 (mag fusion, blocked by AltIMU-10v6 hardware) — separate from this rewrite.
- BCD upgrade for complex polygons — future phase, after Iteration 1 lands.
- LiDAR scan-match smart undock (#43 local tracker) — separate from this rewrite.

## Decision log

| Date | Decision | Reason |
|---|---|---|
| 2026-04-28 | Bootstrap `.planning/` directly with this single phase, skip `/gsd-new-project` | Project context is already documented in CLAUDE.md + memory; full discovery would be overhead |
| 2026-04-28 | Six architectural decisions locked pre-spec (see above) | User explicitly answered all six in chat before invoking `/gsd-spec-phase` |
| 2026-04-28 | SPEC.md committed — 13 requirements locked, ambiguity 0.17 | `/gsd-spec-phase 1` end-to-end |
| 2026-04-28 | CONTEXT.md committed — 12 implementation decisions (D-01..D-12) + Claude's discretion items | `/gsd-discuss-phase 1 --chain` end-to-end; `--chain` triggers auto-advance to plan-phase |
| 2026-04-28 | D-05 supersedes SPEC R-10 wording on file format: key=value text instead of YAML | yaml-cpp deliberately avoided in stack (`hardware_bridge_node.cpp:99`); other R-10 acceptance bits (atomic, sidecar, full field set) preserved |
| 2026-04-29 | Plan 01-01: WriteCheckpoint.srv locks BT-delegates-IO-to-planner pattern | Resolves RESEARCH §10 Q1; planner owns filesystem I/O so atomic-write helper lives in one place |
| 2026-04-29 | Plan 01-01: PlanCoverage.action old BCD-style schema discarded entirely | No clients of old schema on dev branch yet; clean rewrite is safer than versioned shim |
| 2026-04-29 | Plan 01-01: Fixed 3 latent codegen bugs (firmware parser inline-comment + Go/TS missing mowgli_interfaces case branches) | Bugs were silently corrupting Emergency.h fields and would have blocked all downstream waves; in-scope per Rule 1 |
| 2026-04-29 | Plan 01-03: Invariant #15 wording references PolygonSlow polygon + dormant coverage_server.robot_width, not the fictitious collision_monitor.robot_width parameter | The plan-suggested referent does not exist in nav2_params.yaml — the live collision_monitor block uses an explicit polygon. Honest documentation > fictitious referent (Rule 1 fix). |
| 2026-04-29 | Plan 01-02: footprint_inside_polygon uses bg::covered_by, not bg::within | Coverage planning needs the footprint to be allowed to graze the working-area boundary on outline passes; bg::within would reject those poses. T-02-03 GIGO mitigation lives in Plan 05 ValidatorPipeline. |
| 2026-04-29 | Plan 01-02: PCA degenerate-rank fallback uses the polygon's longest edge, not the SelfAdjointEigenSolver eigenvector | Robust against numerical noise on rank-1 covariances and matches SPECIAL_PATTERN intent (centerline along the dominant geometric span). |
| 2026-04-29 | Plan 01-02: atomic_write returns false when fsync(parent_dir) fails | The file is in place at that point but durability across power loss is at risk. Failing loudly surfaces SD/ext4 health issues per RESEARCH §9.4 instead of silently degrading the checkpoint guarantee. |
| 2026-04-29 | Plan 01-04: roslib@^2.x adopted as the browser-side rosbridge client (v1.x ships only ROS1 actionlib via /goal /cancel /feedback /result topics — incompatible with rosbridge_v2 + ROS2 Kilted) | v2's `Action` class uses the `send_action_goal` / `cancel_action_goal` rosbridge_v2 protocol ops, which is what the ROS2 stack speaks. v2 bundles its own types in `dist/RosLib.d.ts`; the v1-only `@types/roslib` package was removed. |
| 2026-04-29 | Plan 01-04: Browser → rosbridge direct on `ws://<host>:9090`, not through the Go API server | The Go API has no generic action-relay endpoint, and adding one was outside this plan's GUI-only scope. Direct browser→rosbridge matches CLAUDE.md's documented rosbridge_server placement (docker/README.md:305). Operators must expose port 9090 from the mowgli-ros2 container. |
| 2026-04-29 | Plan 01-04: Cross-segment-type bridge segments emit a TRANSIT line | Keeps the polyline visually continuous across segment_type boundaries; the D-11 match expression renders bridges grey, matching operator intuition that the robot physically transits between segments even when both endpoints share a non-TRANSIT label. |
| 2026-04-29 | Plan 01-04: Removed the legacy coverageLineWidth zoom-interpolated tool_width memo | Tightly coupled to the deleted plan-preview-coverage layer; the new coverage-plan-line uses a fixed 2.5 px width per UI-SPEC. A tool_width-tracking band can be reintroduced in a future plan as a separate non-segment_type-coloured layer if operators want it back. |
| 2026-04-29 | Plan 01-05: PLAN-07-PLACEHOLDER marker block in `coverage_planner_node.cpp::execute()` | Plan 01-07 will literally `grep -n PLAN-07-PLACEHOLDER` to find the extension point. Action plumbing is end-to-end live for the empty-areas + invalid-geometry paths today; the placeholder short-circuits with ERROR_INTERNAL until Plan 01-07 lands the validator pipeline + plan builder. |
| 2026-04-29 | Plan 01-05: Checkpoint .kv body does NOT carry `area_index` — encoded only in the filename via `std::to_string` | Mitigates T-05-01 (path traversal) at the type-system level: `uint32 -> std::to_string` produces digit-only output, no `..` or slash escape possible. `read_checkpoint_file` fills `area_index` from the caller's argument. |
| 2026-04-29 | Plan 01-05: T-05-02 NaN guard lives in `write_checkpoint_file`, not `serialize_checkpoint` | `serialize` is reused by the round-trip test where we want it to faithfully echo whatever it gets. `write_checkpoint_file` is the single trust boundary on the BT->planner path; `std::isfinite` rejects NaN endpoint x/y + mow_angle_deg before serialising. |
| 2026-04-29 | Plan 01-05: Yaw serialised as a single scalar (`last_swath_endpoint_yaw=`) extracted via `tf2::getYaw` | Six decimals is enough resolution for the SPEC R-11 5cm/5deg tolerance, and the .kv stays human-inspectable in the field. Parser reconstructs the quaternion via `tf2::Quaternion::setRPY(0, 0, yaw)` so the Pose is fully populated when read back. |
| 2026-04-29 | Plan 01-05: Tests link against the STATIC `mowgli_coverage_planner_lib`, not the executable | Mirrors `mowgli_map`'s gtest pattern. `test_coverage_planner_skeleton` spins the node on a SingleThreadedExecutor in a worker thread and queries the action endpoint from a fresh client node — exercises the action plumbing end-to-end without re-running `main.cpp`. |
| 2026-04-29 | Plan 01-06: `MapServerNode::point_in_polygon` kept as a one-line forwarder during Task 1, deleted in Task 2 alongside the strip-planner code that called it | Bridge keeps Task 1 a self-contained, build-clean commit; Task 2 removes the wrapper + every legacy callsite together. Mirrors how Plan 01-02's `inline` exports replaced the static member without breaking external consumers. |
| 2026-04-29 | Plan 01-06: areas.yaml `narrow_area_strategy` field is OPTIONAL on read for forward-compat with legacy on-disk files | Missing key -> default 0 (Skip), the safe behaviour those files implicitly already have. Out-of-range value (T-06-01) -> WARN + clamp to 0. Keeps the on-disk schema bump truly additive — no migration script required. |
| 2026-04-29 | Plan 01-06: mowgli_behavior intentionally LEFT BROKEN until Plan 01-08 lands the BT rewrite | CONTEXT.md "single source of truth (Q1.1=a) over parallel-keep" — pull-path is deleted, not deprecated. Plan 01-06's verify scope is `--packages-select mowgli_interfaces mowgli_map mowgli_coverage_planner` (deliberately excluding mowgli_behavior). Until 01-08 lands, on-Pi5 deployments must NOT pick up this branch. |
| 2026-04-29 | Plan 01-07: PLAN-07-PLACEHOLDER block in coverage_planner_node.cpp REPLACED with the full SPEC R-12 pipeline (pre-geometry validators -> PlanBuilder -> post-geometry validators -> result.metadata) | Marker is gone from every source file. execute() now end-to-end: empty-input/invalid-geometry -> structured PlanError; valid input -> sparse CoverageWaypoint[] plan. |
| 2026-04-29 | Plan 01-07: Single-angle-per-plan policy locked (Iteration 1 simplification) | The FIRST working area's derived angle becomes ctx.mow_angle_used_deg for the whole plan; subsequent areas reuse it. Multi-angle support intentionally deferred to avoid abrupt mid-plan rotations and keep PlanBuilder logic in a single screen. Can be lifted in a future phase. |
| 2026-04-29 | Plan 01-07: Resume snap — PlanBuilder rewrites the first MOWING_BOUSTROPHEDON pose to the persisted last_swath_endpoint | SPEC R-11's 5cm/5° tolerance holds by construction (delta = 0). Alternative (re-derive endpoint geometrically + assert tolerance) rejected because sweep math is sensitive to FP rounding and the persisted endpoint is the ground truth. |
| 2026-04-29 | Plan 01-07: AC-3 plan-size ceiling raised 200 -> 400 (Rule 1 deviation) | SPEC AC-3 is mathematically inconsistent: 22.36 m / 0.13 m = 172 swaths × 2 endpoints = ~344 waypoints, exceeds the 200 ceiling regardless of implementation. Test now guards the sparse-plan invariant (>=50, <=400) which still catches dense pixel-densification regressions. |
| 2026-04-29 | Plan 01-07: Validator order in pre-geometry pipeline is locked (InputSanity must come first) | InputSanityValidator catches out-of-range mow_angle_offset_deg before NoAreas mis-routes ERROR_INTERNAL to ERROR_NO_AREAS. Tests rely on this order. |
| 2026-04-29 | Plan 01-07: OutlineGenerator detects flipped polygons via shoelace winding sign | offset_polygon_inward returns 4 points even when inset overshoots and produces an inverted polygon; sign-flip path treats the result as collapsed (warning for working-area, ERROR_OBSTACLE_OFFSET_FAILED for obstacles). Prevents emitting outline waypoints on a self-intersecting path. |
| 2026-04-29 | Plan 01-08: 5 legacy BT coverage nodes deleted, replaced with PlanCoverageGoal + FollowCoveragePlan (D-03 monolithic dispatch realised) | Single-shot plan generation at AUTONOMOUS branch entry; per-segment dispatch table embedded in FollowCoveragePlan; checkpoint writes delegated to /coverage_planner_node/write_checkpoint (Q1 lock). |
| 2026-04-29 | Plan 01-08: setBladeEnabled changed from private to virtual+protected for unit-test override | Same-process Cyclone DDS service round-trips proved flaky in single-process gtest setups; the in-process software contract (onHalted invokes setBladeEnabled(false)) is what the test asserts; the DDS edge is exercised by Plan 01-09 hardware/sim test. |
| 2026-04-29 | Plan 01-08: PreFlightCheck migrated from /map_server_node/get_coverage_status (deleted by Plan 01-06) to /map_server_node/get_all_areas | Treats any non-navigation polygon as a valid mowing area. Cleaner than the area_index=0 hack the deleted service required. |
| 2026-04-29 | Plan 01-08: IncrementSkippedSwaths node + BTContext::skipped_swaths field deleted entirely | Only consumer was the deleted SkipStrip subtree of the legacy AreaLoop; HighLevelStatus's swath-progress fields zeroed until plan-level progress is wired via PlanCoverage feedback in a future plan. |
| 2026-04-29 | Plan 01-09: T0 precondition fix moves mowgli_geometry from target_link_libraries to ament_target_dependencies in mowgli_coverage_planner | Root cause is consumer-side namespace mismatch — exported target is `mowgli_geometry::mowgli_geometry`, consumer used bare `mowgli_geometry`. ament_target_dependencies reliably reads `${mowgli_geometry_INCLUDE_DIRS}` and matches mowgli_map's working pattern. |
| 2026-04-29 | Plan 01-09: coverage_planner_node added to all three top-level launch entry points (full_system, sim_full_system, sim_small_garden), not "mowgli_bringup.launch.py" | The plan-instructed file does not exist; the de-facto bringup entry points are these three launch files (one per environment). All three launch coverage_planner_node alongside map_server_node sharing the same robot_config (D-07 lockstep). |
| 2026-04-29 | Plan 01-09: e2e_test.py PlanCoverage probe runs BEFORE COMMAND_START | Independent rclpy ActionClient probe gives the test a non-flaky pre-START gate that doesn't depend on BT cooperation. Asserts plan-shape invariants (50<=N<=400, OUTLINE_WORKING_AREA present, blade-rule SPEC R-4) regardless of whether the BT later reaches MOWING. |
| 2026-04-29 | Plan 01-09: SUMMARY.md is intentionally non-final | SPEC AC-13 hardware smoke is operator-gated (autonomous: false). T0+T1+T2 are ✅ green; T3 (Pi5 Eichenau garden) is ⬜ pending. Phase 1 is in "automatable scope complete, hardware-verified pending" until the operator returns the smoke verdict. |
