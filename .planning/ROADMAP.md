# Roadmap

## Active milestone: Coverage Planner v2

The pull-based, ad-hoc strip planner inside `map_server_node` is being replaced by a deterministic full-plan coverage planner with action-based delivery, footprint-aware geometry, dock segments inline, and YAML checkpoint resume.

## Phases

### Phase 1 — Coverage Planner Rewrite

**Status:** all 11 plans complete (automatable scope); SPEC AC-13 Pi5 hardware smoke operator-gated. R-9 + R-11 end-to-end correct — re-run `/gsd-verify-phase 1` to flip verification status.
**Plans:** 11 plans (10 fully complete + 1 automatable-complete-pending-hardware)
**Goal:** Replace the existing pull-based strip planner with a new `coverage_planner_node` that emits a complete deterministic sequential `PoseStamped` waypoint plan via `PlanCoverage.action`, with metadata, YAML checkpoints, and pre-flight geometric validation. Plan inkludiert Undock/Approach/Dock-Segmente. BT folgt Plan sequenziell.

**Canonical refs:**
- User-Spec: `.planning/REQUIREMENTS.md` (the 10-validation-point spec submitted 2026-04-28)
- Architectural decisions locked in `STATE.md` (separate node, action-based, sequential, simple AABB sweep, rectangular footprint w/ drive_axis_offset, YAML checkpoint)
- CLAUDE.md invariants 7+8 (cell-based coverage, FTCController for swaths)
- Existing planner code: `ros2/src/mowgli_map/src/map_server_node.cpp` (`compute_outline_path`, `ensure_strip_layout`, `find_next_unmowed_strip`)
- Existing action skeleton: `ros2/src/mowgli_interfaces/action/PlanCoverage.action` (likely needs revision — current shape predates this spec)

**Success criteria:**
- New `coverage_planner_node` package builds, action server exposes `PlanCoverage.action`
- Plan output is a sequential PoseStamped list with `segment_type` annotation and metadata (mow angle, outline count, path spacing, areas processed, skipped regions, warnings)
- All 10 validation points from the user spec pass before plan emission; failure produces a structured error instead of a plan
- Resume after pause: BT can continue from the exact open swath using the YAML checkpoint
- BT follows the plan sequentially; lokale dynamische Hindernisse via collision_monitor + Nav2 lokalem Planer ohne globale Replanung
- E2E: simulated working area + obstacle + navigation area produces a deterministic plan that satisfies all validation points

**Out of scope (this phase):**
- Boustrophedon Cell Decomposition (BCD) — Iteration 1 uses simple AABB sweep with obstacle clipping
- Bezier/Spline smoothing — straight segments + in-place yaw rotations only

Plans:
- [x] 01-01-PLAN.md — mowgli_interfaces extensions (4 new msgs + GetAllAreas.srv + WriteCheckpoint.srv + PlanCoverage.action rewrite + MapArea.narrow_area_strategy + firmware/Go/TS regen) → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-01-SUMMARY.md` (commits `27cae866`, `dd19d5c9`, `d39255c4`)
- [x] 01-02-PLAN.md — mowgli_geometry header-only library (4 promoted helpers + footprint/PCA/atomic_write + 4 unit tests) → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-02-SUMMARY.md` (commits `ae551459`, `63933306`, `351f4139`)
- [x] 01-03-PLAN.md — mowgli_robot.yaml robot_geometry: section + CLAUDE.md Architecture Invariants #7 rewrite + #15 (manual sync rule) → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-03-SUMMARY.md` (commits `8bf52a71`, `fb7020c8`)
- [x] 01-04-PLAN.md — GUI: useCoveragePlan hook + delete plan-preview-* layers + add coverage-plan-* layers + EditAreaModal narrow_area_strategy dropdown + MapToolbar Preview Plan button → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-04-SUMMARY.md` (commits `66ffe815`, `504f490c`)
- [x] 01-05-PLAN.md — mowgli_coverage_planner skeleton: action server + GetAllAreas client + WriteCheckpoint service + Checkpoint .kv I/O + 10 unit tests → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-05-SUMMARY.md` (commits `4bce4424`, `72a4d925`, `14e51df4`)
- [x] 01-06-PLAN.md — map_server_node cleanup: GetAllAreas server + delete 4 pull-path .srv files + delete 8+ strip-planner functions + areas.yaml narrow_area_strategy round-trip → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-06-SUMMARY.md` (commits `d5634dc0`, `73124c62`). NOTE: mowgli_behavior temporarily breaks; Plan 01-08 repairs it.
- [x] 01-07-PLAN.md — mowgli_coverage_planner core: ValidatorPipeline (12 validators / 8 error codes) + OutlineGenerator + BoustrophedonSweeper + NarrowAreaStrategy + auto-rotate + resume snap + 7 unit tests → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-07-SUMMARY.md` (commits `48447625`, `bc42d57f`, `cc818f3e`, `78ac2d66`). PLAN-07-PLACEHOLDER block in coverage_planner_node.cpp REPLACED with the full SPEC R-12 pipeline.
- [x] 01-08-PLAN.md — BT integration: delete 5 legacy nodes + add PlanCoverageGoal + FollowCoveragePlan + main_tree.xml subtree + bt_context.hpp + safety unit tests → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-08-SUMMARY.md` (commits `70543ec9`, `4b213d3a`, `698cbbd5`). mowgli_behavior build break (Plan 01-06) healed; T-08-01 + T-08-03 HIGH-severity safety threats regression-tested.
- [~] 01-09-PLAN.md — E2E + Pi5 hardware smoke: launch wiring + e2e_test.py update + VALIDATION.md populate + Pi5 Eichenau garden checkpoint (SPEC AC-13). **Automatable scope COMPLETE** (T0 precondition fix + T1 launch+e2e + T2 VALIDATION populate; commits `d20e4025`, `08ae7808`, `5e0683b6`; SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-09-SUMMARY.md`). **T3 hardware smoke ⬜ pending operator verification** — see SUMMARY.md § "Hardware Checkpoint Procedure".
- [x] 01-10-PLAN.md — **Gap closure (R-9/R-11 root cause):** Add `uint32 area_index` to `CoverageWaypoint.msg`; regenerate firmware rosserial + Go + TypeScript bindings; refactor `PlanBuilder` to stamp `area_index` on every emitted waypoint via a single `stamp_and_push` lambda (UNDOCK/dock segments → UINT32_MAX sentinel; outline/sweep waypoints → loop index). 3 new gtest cases pin the contract. → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-10-SUMMARY.md` (commits `c329a81b`, `36bbbb30`, `bd6fda58`)
- [x] 01-11-PLAN.md — **Gap closure (R-9/R-11 production fix):** Replace `req->checkpoint.area_index = last_wp.sequence_id` with `last_wp.area_index` in `dispatch_checkpoint_write`; add `BTContext::last_mow_angle_used_deg` propagated by PlanCoverageGoal from `PlanMetadata.mow_angle_used_deg`; early-return on UINT32_MAX sentinel for dock/undock segments. 3 new TEST_F cases run an in-process WriteCheckpoint stub server and capture the request payload to assert the canonical key. → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-11-SUMMARY.md` (commits `5405fa60`, `d1361536`, `354066e1`)


### Phase 2 — LiDAR-based dock pose estimation (undock + closed-loop docking)

**Status:** ⬜ NEXT (operator-blocking — promoted from backlog 2026-04-29 after end-to-end mow showed RTK-only docking misses the V-funnel even with continuous RTK-Fixed)
**GH issues:** [#43](https://github.com/danyial/mowglinext/issues/43) (smart undock: LiDAR free-space probe + RTK validation), [#75](https://github.com/danyial/mowglinext/issues/75) (closed-loop LiDAR docking final approach)
**Goal:** Eliminate the systematic "robot misses the dock V-funnel" failure mode by switching from RTK-only docking to a hybrid RTK-coarse + LiDAR-fine approach. The same LiDAR scan-match infrastructure is shared with the smart-undock work — capture once, use both directions.

**Architecture:** the dock geometry is captured once (Variante γ — operator-confirmed first capture, automatic 7-day refresh on high-confidence undock matches) into `dock_scan.pcd`. At undock end, the robot's exact post-undock pose is recorded as `dock_approach.yaml`. At dock approach, RTK navigates the robot to that exact point with yaw aimed straight at the dock; from there the final 1.5 m runs under continuous ICP correction (lateral + yaw P-controller) crawling forward at ~5 cm/s until `is_charging`. By construction the approach point IS the same point the robot ended up at after undock — eliminating the cumulative drift between `dock_calibration.yaml` and reality (measured 10 cm Y-offset on the 2026-04-29 mow).

**Components:**
1. `dock_scan_capture` — captures `/scan_kicp` snapshot at GPS-RTK dock yaw finalisation, writes `dock_scan.pcd` + metadata.
2. `RecordDockApproachPose` BT node — runs at end of UndockSequence, writes `dock_approach.yaml` from current LiDAR-match pose.
3. `dock_scan_match` — continuous ICP `/scan_kicp` ↔ `dock_scan.pcd`, publishes `/dock_match/pose` + `/dock_match/confidence`.
4. `ApproachDock` BT node — Nav2 NavigateToPose to `dock_approach.yaml` (RTK-coarse, replaces current opennav_docking RPP approach).
5. `FineDock` BT node — replaces opennav_docking SimpleChargingDock for last 1.5 m. Subscribes `/dock_match/pose`, P-controller on lateral_y + yaw, `cmd_vel.x = +0.05 m/s`, stops on `is_charging`, aborts on confidence drop.
6. GUI Dock-card extension — show `dock_match` confidence, `dock_scan.pcd` age, last fine-dock distance/lateral error.

**Why next:** Smooth outlines (#70) are operator-cosmetic; closed-loop docking is operator-blocking. Both undock and dock failures stop end-to-end mowing — without #43+#75, the robot can't reliably restart the next session. Smooth outlines are deferred to Phase 3.

**Out of scope (this phase):** Replacing opennav_docking entirely (FineDock is a parallel BT node, opennav_docking remains the framework). LiDAR feature-detection of the dock structure itself (we use raw scan-vs-snapshot ICP, not parametric dock detection).

**Acceptance:** End-to-end COMMAND_START → undock → mow → return → autodock with `is_charging` engaged on first attempt at lateral error < 2 cm and yaw error < 1°.

**Plans:** 8 plans

Plans:
- [x] 02-01-PLAN.md — Bootstrap: kinematic_icp submodule init + Dockerfile deps (laser_geometry/PCL/sophus) + DockMatchConfidence.msg + 4-pipeline codegen + mowgli_geometry::key_value_parser shared promotion + kiss_icp VoxelHashMap public-API probe (PROBE.md) → SUMMARY at `.planning/phases/02-lidar-dock-pose-estimation/02-01-SUMMARY.md` (commits `026fa024`, `e41e8534`, `ce00312f`)
- [ ] 02-02-PLAN.md — mowgli_lidar_docking package skeleton: IDockMatcher contract + dock_scan_io (PCL ASCII + atomic) + confidence_metrics (production+brute-force) + dock_approach_loader + dock_scan_meta_loader + 5 gtest binaries (~15 cases)
- [ ] 02-03-PLAN.md — dock_scan_capture rclpy library (5-frame voxel-merge, atomic write) + calibrate_imu_yaw_node extension (R-1, no extra operator step) + 7 pytests
- [ ] 02-04-PLAN.md — dock_scan_match node + KinematicIcpDockMatcher production impl + mtime watcher + TF distance gate + degraded mode + navigation.launch.py wiring + 11 gtests (R-2, R-3)
- [ ] 02-05-PLAN.md — dock_yaw_to_set_pose.py cascade extension (lidar > file > heading) + 7 cascade pytests + AUTONOMOUS-state regression (R-4)
- [ ] 02-06-PLAN.md — 5 new BT nodes (RecordDockApproachPose / ApproachDock / FineDock / PreUndockClearanceCheck / PostUndockRtkValidation) + BTContext extension + factory registration + 12 gtests (R-5..R-9, R-11..R-13)
- [ ] 02-07-PLAN.md — main_tree.xml migration (6 DockRobot sites + UndockSequence extension) + GUI Dock-card extension (useDockMatch hook + DockMatchCard component + topicMap relay per D-12) (R-9, R-10)
- [ ] 02-08-PLAN.md — Sim infrastructure (synthetic_scan_kicp_publisher + sim_lidar_docking.launch.py + e2e_test extension) + mow_session_monitor extension (D-15) + 02-VERIFICATION.md + Pi5 5-of-5 hardware acceptance checkpoint (operator-gated, autonomous: false)

### Phase 3 — Smooth outline-pass transitions (≤30° tangent change)

**Status:** ⬜ scheduled after Phase 2 (operator-cosmetic; operator-blocked while #43+#75 not done)
**GH issue:** [#70](https://github.com/danyial/mowglinext/issues/70)
**Goal:** Every transition between consecutive plan segments (outline-pass-N → N+1, last-outline → first-strip, strip → strip U-turns, last-strip → RETURN_TO_DOCK) has ≤ 30° tangent-angle change. The planner chooses the start vertex of each outline pass so its yaw smoothly continues the previous pass's exit; emits intermediate join waypoints if no vertex meets the threshold; applies the same heuristic to outline→strip and strip→strip turns.

**Why scheduled here:** Phase 1 verified the planner end-to-end. First hardware mow run (2026-04-29 hexagon test) exposed visible operator-side ugliness AND physical impact: FTC PRE_ROTATE phases of 2.3 s per transition burning pose-drift budget. Pre-existing — not introduced by Phase 1 — but only became visible once the validator stopped rejecting plans wholesale. Demoted from Phase 2 because Phase 2 (#43+#75 LiDAR docking) is the operator-blocking issue; smooth outlines are quality-of-life.

**Out of scope (this phase):** Full Bezier/spline smoothing (separate future enhancement). Multi-pass obstacle outlines (stays at 1 pass).

### Phase 4 — Live coverage visualisation in the GUI

**Status:** ⬜ scheduled after Phase 3 (operator-UX; quality-of-life)
**GH issues:** [#71](https://github.com/danyial/mowglinext/issues/71) (auto-show plan on Start), [#72](https://github.com/danyial/mowglinext/issues/72) (mowed-area overlay)
**Goal:** Operator sees the planned coverage path the moment they click Start Mowing (no manual Preview Plan click) AND a live, semi-transparent overlay of which cells the blade has already covered during the session.

Two parallel work streams:
1. **#71 — Auto-plan-on-Start**: BT publishes its current coverage plan to a latched topic (`/behavior_tree_node/active_coverage_plan`). GUI subscribes and reuses the existing coverage-plan layer in `MapPage.tsx`. Plus: disable Preview Plan button when `state == AUTONOMOUS` (closes a known race condition where mid-mow Preview Plan triggers a BT re-plan and breaks the run).
2. **#72 — Mowed-area overlay**: small server-side node (or extension to `mowgli_monitoring/diagnostics_node`) maintains a `nav_msgs/OccupancyGrid` of cells covered by the blade footprint while `mow_enabled=true`. Published on `/coverage_progress` (latched, transient_local). GUI consumes via the existing `coverageCells` subscription wiring. Persists per-session to `/ros2_ws/maps/`.

**Why this phase**: Operator can't currently see whether the robot has covered everything or repeatedly missed a corner. Plus: the manual-Preview-Plan workaround actively broke a run on 2026-04-29 (mid-mow click triggered BT re-plan).

**Out of scope (this phase):** Heatmap of coverage *count* per cell (multi-pass density). 3D visualisation. Historical playback of past sessions.

## Backlog (999.x — not scheduled, not blocking active milestone)

Phases promoted out of `999.x` get renumbered into the active milestone via `/gsd-review-backlog`.

### Phase 999.1 — Single-source robot footprint publisher

**Status:** ⬜ not started
**GH issue:** [#67](https://github.com/danyial/mowglinext/issues/67)
**Goal:** Eliminate the three-way manual sync of chassis dimensions across `mowgli_robot.yaml:chassis_*`, `mowgli_robot.yaml:robot_geometry:*`, and `nav2_params.yaml:collision_monitor:PolygonSlow.points` (Architecture Invariant #15) by introducing a `mowgli_footprint_publisher` Python node that derives all three from a single source and publishes the slow-zone polygon on `/robot_footprint_slow`. Switch `nav2_collision_monitor` to `type: polygon_topic`. Delete the duplicated `robot_geometry:` block. Retire Architecture Invariant #15.

**Why backlog and not now:** the three-way manual sync works as long as we co-commit, and 2026-04-29 we just paid the cost (chassis re-measure on the bench → 0.57 × 0.43 × 0.18 propagated to all three files). Half-day of work, but not blocking any current goal — promotable when next chassis-geometry change hits, or when we onboard a non-YF500 platform.

**Out of scope:** PolygonStop re-introduction (separate decision — see commit `cc2a4c4` for why it was dropped).
