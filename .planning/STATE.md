---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: unknown
stopped_at: Plan 02-08 automatable scope complete; Pi5 hardware UAT pending
last_updated: "2026-04-30T08:00:00.000Z"
progress:
  total_phases: 2
  completed_phases: 1
  total_plans: 19
  completed_plans: 18
  percent: 95
---

# Project state

## Current phase

2 — LiDAR Dock Pose Estimation (Wave 1-3 complete + Wave 4 Plan 02-07 complete; Plan 02-08 e2e + Pi5 hardware smoke remaining)

## Current Plan

02-08 — Sim publisher + e2e + monitor + Pi5 checklist (🟡 AUTOMATABLE COMPLETE; T1+T2 ✅ commits `ba9647de`, `9d0fd50c`, `5544c763`; SUMMARY at `.planning/phases/02-lidar-dock-pose-estimation/02-08-SUMMARY.md`. T3 ⬜ Pi5 5-of-5 hardware UAT pending operator verification per `02-08-PI5-CHECKLIST.md`. D-14 sim publisher + sim_lidar_docking.launch.py + e2e_test `_run_fine_dock_phase` + D-15 mow_session_monitor `dock_match.*` + `lateral_error_at_contact` fields + 02-VERIFICATION.md acceptance matrix. Phase 2 closure blocked on operator running PI5-CHECKLIST + committing 02-08-PI5-RESULTS.md.)

## Total Plans

19 (Phase 1: 11 plans, Phase 2: 8 plans)

## Resume point

- **Last completed step:** Plan 02-08 Tasks 1+2 — automatable scope. D-14 synthetic_scan_kicp_publisher + sim_lidar_docking.launch.py + pre-baked sim_dock_scan.pcd fixture + e2e_test `_run_fine_dock_phase` + D-15 mow_session_monitor `/dock_match/{pose,confidence}` subscribers + per-sample `dock_match.*` fields + rising-edge `lateral_error_at_contact_m` + summary `dock_match_summary` block + 02-VERIFICATION.md (29 acceptance rows) + 02-08-PI5-CHECKLIST.md (operator runbook).
- **Next step:** Phase-end podman colcon build to clear DEFERRED-TO-PHASE-END-BUILD steps from Plans 02-01 through 02-08; THEN operator runs `02-08-PI5-CHECKLIST.md` on Pi5 for the 5-of-5 hardware UAT (R-7/R-9/R-11 hardware gates) + commits `02-08-PI5-RESULTS.md`; THEN `/gsd-verify-phase 2` flips hardware rows in 02-VERIFICATION.md. Phase 2 closes only after that sequence completes.
- **Auto-chain flag persisted:** yes (`workflow._auto_chain_active=true` in `.planning/config.json`)
- **Wave 1 plans:** 01-01 ✅ COMPLETE, 01-03 ✅ COMPLETE
- **Wave 2 plans:** 01-02 ✅ COMPLETE, 01-04 ✅ COMPLETE
- **Wave 3 plans:** 01-05 ✅ COMPLETE, 01-06 ✅ COMPLETE
- **Wave 4 plans:** 01-07 ✅ COMPLETE, 01-08 ✅ COMPLETE
- **Wave 5 plans:** 01-09 🟡 AUTOMATABLE COMPLETE (T0+T1+T2 ✅; T3 Pi5 hardware smoke ⬜ pending operator verification)
- **Gap-closure plans:** 01-10 ✅ COMPLETE (CoverageWaypoint.area_index + PlanBuilder stamping), 01-11 ✅ COMPLETE (dispatch_checkpoint_write BT-side fix, CR-01 closed)

## Last session

- **Last session:** 2026-04-30T08:00:00.000Z
- **Stopped at:** Plan 02-08 automatable scope complete; Pi5 hardware UAT pending
- **Resume file:** None
- **Blockers:**
  - SPEC AC-13 (Phase 1) — operator must still execute the Pi5 Eichenau garden 5-of-5 smoke per 01-09-SUMMARY.md.
  - Phase 2 — operator must execute the new Pi5 5-of-5 hardware UAT per `02-08-PI5-CHECKLIST.md` (R-7 ≤ 2 cm lateral / ≤ 1° yaw at contact, R-9 manual E-Stop test in cycle 3, R-11 ≥7-day auto-refresh in one cycle). Operator commits `02-08-PI5-RESULTS.md` afterwards; `/gsd-verify-phase 2` flips R-7/R-9/R-11 hardware rows in `02-VERIFICATION.md`.
  - Phase 2 phase-end podman build (host = macOS, no colcon) — clears DEFERRED-TO-PHASE-END-BUILD steps from Plans 02-01 through 02-08 before the operator's 5-of-5 begins.

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
| Phase 01 P10 | 6min | 2 tasks | 9 files |
| 01    | 11   | 4min     | 4     | 4     |
| Phase 02 P01 | 31min | 4 tasks | 14 files |
| Phase 02 P02 | 32 | 2 tasks | 17 files |
| Phase 02 P03 | 38 | 2 tasks | 7 files |
| Phase 02 P04 | 35 | 2 tasks | 11 files |
| Phase 02 P05 | 25 | 1 tasks | 3 files |
| Phase 02 P06 | 9 | 2 tasks | 8 files |
| Phase 02 P07 | 18 | 2 tasks | 8 files |
| Phase 02 P08 | 25 | 3 tasks (T1+T2 ✅; T3 Pi5 5-of-5 pending operator) | 11 files |

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
| 2026-04-29 | Plan 01-11: kNoArea early-return placed before checkpoint_client_ lazy creation | Avoids creating the DDS client singleton for dock/undock segments — no unnecessary DDS participant churn on non-area waypoints. T-11-03 mitigation. |
| 2026-04-29 | Plan 01-11: Same-node client+server pattern in WriteCheckpointAreaIndexTest | Bypasses Cyclone DDS inter-node flakiness (01-08 Deviation 4) while still exercising the full rclcpp client path. Protected (not public) lift of coverage_plan_ + dispatch_checkpoint_write. |
| 2026-04-29 | Plan 02-02: kiss_icp::VoxelHashMap production code path on both PROBE.md A1+A2 verdicts; no fallback path needed | A1 (AddPoints) + A2 (GetClosestNeighbor) both confirmed public via struct default access in kiss_icp v1.2.0 |
| 2026-04-29 | Plan 02-02: GetClosestNeighbor returns std::tuple<Eigen::Vector3d, double>; confidence_metrics.cpp uses returned squared distance directly | Plan-snippet showed `Eigen::Vector3d nn = voxel_map.GetClosestNeighbor(p)` which would have failed to compile; PROBE.md verbatim declaration is the source of truth |
| 2026-04-29 | Plan 02-02: mowgli_lidar_docking CMakeLists uses ament_target_dependencies(... mowgli_geometry kinematic_icp ...) instead of explicit namespaced IMPORTED targets | Mirrors mowgli_coverage_planner Plan 01-05 pattern (lines 65-74); safe across kinematic_icp upstream releases that may rename per-component CMake targets |
| 2026-04-29 | Plan 02-02: PCD atomic write renders canonical PCD v0.7 ASCII to a std::string and routes through mowgli_geometry::atomic_write | PCL savePCDFile owns its own fd with no public hook to redirect to a writable string buffer in the Kilted apt build of PCL — hand-rolled ASCII renderer is the simpler path |
| 2026-04-29 | Plan 02-02: dock_scan_meta_age_exceeds uses timegm-based real day arithmetic | Lex compare on raw ISO strings (RESEARCH §A5) is the documented failure mode at year boundaries / clock skew; T-02-06 mitigation requires real day arithmetic |
| 2026-04-29 | Plan 02-03: dock_scan_capture is a rclpy library (not a node) imported by calibrate_imu_yaw_node | Library shape lets Plan 02-08's synthetic_scan_kicp_publisher reuse the same code path in sim without standing up calibrate_imu_yaw_node's full state machine. Function returns; no long-lived subscriptions. |
| 2026-04-29 | Plan 02-03: setup.py-route to install dock_scan_capture rejected; routed through CMakeLists.txt install(PROGRAMS ...) | Package is ament_cmake (no setup.py exists); existing convention for the 8 sibling Python scripts is install(PROGRAMS ...). Plan-vs-codebase mismatch fixed under Rule 3. |
| 2026-04-29 | Plan 02-03: capture-call gate WILL trip on most installs (robot ~0.8 m off dock at insertion point) | Pitfall 2 stationarity gate is the canonical safeguard; failure path is non-fatal and documented; Plan 02-07 GUI Recapture button is the primary path for dock_scan.pcd to land. |
| 2026-04-29 | Plan 02-03: tf_transformations import is lazy with inline math fallback | Avoids adding a hard runtime dependency on ros-kilted-tf-transformations just for a single quaternion-to-yaw conversion in the dock_scan extrinsic helper. |
| 2026-04-29 | Plan 02-03: dock_scan_capture import wrapped in try/except with None sentinel | Calibration node must never crash on legacy installs that have not yet rebuilt mowgli_localization; degrade dock_scan add-on, keep IMU-yaw calibration intact. |
| 2026-04-29 | Plan 02-03: CLAUDE.md AI #1 says calibrate_imu_yaw_node is rclcpp; disk reality is rclpy | A2 revision (2026-04-27) reverted Decision A back to upstream Python; CLAUDE.md is stale. Followed disk reality; flagged in 02-03-SUMMARY.md Coordination Risks for the human maintainer. |
| 2026-04-29 | Plan 02-04: KinematicIcpDockMatcher seeds via kicp_.SetPose(anchor) followed by kicp_.VoxelMap().AddPoints(dock_points) — PROBE.md A1 production path | Order is load-bearing: SetPose() internally calls local_map_.Clear() (KinematicICP.hpp:88), so AddPoints MUST come after. Reversing it silently drops the dock points. |
| 2026-04-29 | Plan 02-04: Reload() destroys + reconstructs the kinematic_icp pipeline via std::optional reset + emplace under std::mutex | Cheaper than per-member reset and immune to stale adaptive-threshold history poisoning a fresh capture. Same mutex protects Match() so a concurrent Reload during Match cannot mid-flight corrupt the voxel map (T-04-07 mitigation). |
| 2026-04-29 | Plan 02-04: dock_scan_match uses default single-threaded executor (rclcpp::spin) on Pi5 | TF distance gate (gate_distance_m default 5 m) skips RegisterFrame when robot is far from dock — duty cycle stays low enough that MultiThreadedExecutor is unnecessary. Switch only if Plan 02-08 hardware smoke shows scheduling lag. |
| 2026-04-29 | Plan 02-04: /dock_match/pose published ONLY when trusted=true; /dock_match/confidence published every tick (including degraded + TF-far + scan-missing paths) | Pitfall 6 mitigation: consumers see a clean baseline at t=0 before any scan arrives. Pose absence is the "not yet trusted" signal; conf liveness is the "alive" heartbeat. Both topics are needed for a clean state machine in FineDock onRunning. |
| 2026-04-29 | Plan 02-04: 180° flip detector (Pitfall 1) uses atan2(R(1,0), R(0,0)) on the rotation matrix + hand-rolled shortest_angular_distance | Keeps the matcher library free of tf2 / angles dependencies. The trust gate downstream collapses (valid && trusted_metric) into a single bool, so a flipped pose surfaces as trusted=false even if the confidence math alone would have let it through. |
| 2026-04-29 | Plan 02-04: Test ctor injects an IDockMatcher to bypass kiss_icp + LaserProjection + a live tf_buffer in the gtest harness | mtime watcher's dynamic_cast<KinematicIcpDockMatcher*> returns nullptr for the MockMatcher path so the watcher quietly skips Reload — the production Reload path is exercised by ReloadSwapsDockScan in test_kinematic_icp_dock_matcher. End-to-end /scan_kicp -> /dock_match/pose smoke is DEFERRED-TO-PLAN-02-08-SIM. |
| 2026-04-30 | Plan 02-07: 6 DockRobot sites migrated to LidarDock\* Sequence subtrees (R-10); UndockSequence extended with Pre/Post + RecordDockApproachPose (R-5, R-11, R-12, R-13) | Each migrated site gets a unique LidarDock\* name (Critical/Rain/Battery/FailedCoverage/MowingComplete/Home) so BT-log diffs surface which dock-trigger path fired. |
| 2026-04-30 | Plan 02-07: R-9 verdict — no new XML guard; existing IsCommand+ClearCommand pattern + EmergencyHandler-does-not-restore-command together prevent FineDock auto-restart after E-Stop reset | Verified at plan-revision time against main_tree.xml lines 165, 170-180, 318, 510, 538. Plan 02-06 unit test FineDockHaltsOnEmergency pins the in-node onHalted publish_zero contract. |
| 2026-04-30 | Plan 02-07: GUI dock-card extension uses Go-relay topicMap (D-12 lock); DockMatchCard surfaced via antd Popover trigger in MowerStatus topbar; Recapture button reuses /api/calibration/imu-yaw | No new browser-side rosbridge dependency. Compact topbar Popover keeps MowerStatus visually consistent while D-09 placement (next to Charging indicator) is honoured. |
| 2026-04-30 | Plan 02-07: DockMatchConfidence type redeclared inline in useDockMatch.ts (NOT imported from ros.generated.ts or hand-edited into ros.ts) | ros.generated.ts has the type but is broken at HEAD (MapAreaConstants enum self-reference, lines 281-283 — pre-existing Plan 02-01 codegen bug). ros.ts is hand-curated and not yet resynced. Inline redeclaration avoids hand-editing either; Phase-2-out-of-scope per deferred-items.md. |
| 2026-04-30 | Plan 02-07: yarn build BLOCKED by pre-existing ros.generated.ts MapAreaConstants enum bug; documented in deferred-items.md, NOT auto-fixed | SCOPE BOUNDARY (deviation rules): pre-existing failure in unrelated file. cd gui && go build ./... exits 0; tsc with ros.generated.ts excluded passes — Plan 02-07 files compile cleanly in isolation. |
| 2026-04-30 | Plan 02-08: pre-baked sim_dock_scan.pcd over launch-time sim calibration drive | Determinism + speed > realism. PCD is generated from the same V-funnel synthesizer with `noise_sigma=0`; live sim publisher applies σ=5 mm noise during the run. CI repeatability + faster sim startup. |
| 2026-04-30 | Plan 02-08: Sim FineDock phase asserts `is_charging` engaged + matcher trusted=true at least once, NOT lateral_error ≤ 2 cm | Sim noise floor (synthetic σ=5 mm + Gazebo wheel-odom drift) differs from real LD19 noise floor; the 2-cm SPEC R-7 gate belongs to the hardware bench. Sim acts as smoke test only. |
| 2026-04-30 | Plan 02-08: lateral_error_at_contact_m fires only on rising edge of is_charging | One value per cycle is exactly what the 5-of-5 acceptance grep needs. Sampling every tick during dock contact would dilute the metric; tail outliers would be hidden inside the distribution. |
| 2026-04-30 | Plan 02-08: synthetic_scan_kicp_publisher reads TF in `map -> base_footprint_wheels` not `map -> base_footprint`; static fallback robot pose for early-CI ticks | Matches the parallel-tree convention per CLAUDE.md Architecture Invariant #1; consumer behavior identical between sim and Pi5. Static fallback covers the case where Gazebo's clock hasn't started yet. |
| 2026-04-30 | Plan 02-08: DockMatchConfidence subscriber in mow_session_monitor is lazy-imported | Pre-Phase-2 builds don't ship the message; monitor still runs and emits null dock_match.* fields rather than crashing on import. Mirrors the existing AbsolutePose/Status lazy-import pattern. |
