---
phase: 2
slug: lidar-dock-pose-estimation
status: pending-execution
nyquist_compliant: true
wave_0_complete: false
created: 2026-04-29
populated: 2026-04-29
---

# Phase 2 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution. Materialised as a planning artifact at plan-time (per Nyquist Check 8e: VALIDATION.md must exist before plans pass verification). Source content extracted from `02-RESEARCH.md` §Validation Architecture (lines 1169-1227, committed `98d8f470`). Drift-check is the responsibility of Plan 02-01 Task 4 during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | `ament_cmake_gtest` (C++ unit/integration), Python launch_testing (E2E sim), pytest for `e2e_test*.py` + `dock_yaw_to_set_pose` cascade test |
| **Config files** | `ros2/src/mowgli_lidar_docking/test/CMakeLists.txt` (Wave 1 creates), `ros2/src/mowgli_behavior/test/CMakeLists.txt` (Wave 3 extends), `ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py` (Wave 2 creates), `ros2/src/mowgli_localization/test/test_calibrate_imu_yaw_capture.py` (Wave 1 creates) |
| **Quick run command** | `cd ros2 && colcon test --packages-select mowgli_lidar_docking mowgli_behavior mowgli_localization --event-handlers console_cohesion+ --ctest-args --output-on-failure` |
| **Full suite command** | `cd ros2 && make build && make test && make e2e-test` |
| **Estimated runtime** | quick: ~45 s; full (with sim): ~6–8 min |

---

## Sampling Rate

- **After every task commit:** Run `colcon test --packages-select mowgli_lidar_docking mowgli_behavior mowgli_localization` (~45 s)
- **After every plan wave:** Run `make test` for the full ROS2 test suite (~3 min) plus `make sim` + `make e2e-test` (~5 min headless sim)
- **Phase gate (before Pi5 hardware acceptance):** Full suite green; sim e2e_test passes the new FineDock phase via `_run_fine_dock_phase`; operator runs Pi5 5-of-5 acceptance per `02-08-PI5-CHECKLIST.md`
- **Max feedback latency:** 45 s (quick), 600 s (full + sim)

---

## Per-Requirement Verification Map

> Direct extract from `02-RESEARCH.md` §Validation Architecture > "Phase Requirements → Test Map". Every R-XX requirement from `02-SPEC.md` MUST appear in at least one row. Status column updated by per-plan SUMMARY.md after execution.

| Req ID | Behavior | Test Type | Automated Command | File | Status |
|--------|----------|-----------|-------------------|------|--------|
| R-1 | dock_scan.pcd written by calibrate_imu_yaw_node extension | unit (Python) | `pytest ros2/src/mowgli_localization/test/test_calibrate_imu_yaw_capture.py -k dock_scan` | Wave 1 (Plan 02-03) | ⬜ pending |
| R-1 | dock_scan_meta.yaml round-trips through shared key=value parser | unit (C++) | `colcon test --packages-select mowgli_lidar_docking --ctest-args -R DockScanMetaRoundTrip` | Wave 1 (Plan 02-02) | ⬜ pending |
| R-1 | PCD file readable by `pcl_viewer` | manual / acceptance | (operator opens pcl_viewer on Pi5) | manual | ⬜ pending |
| R-2 | `/dock_match/pose` published at ≥ 5 Hz (D-07 = 10 Hz default) | unit (rate test) | gtest `DockScanMatchNodeRateTest` (in-process node + mock matcher) | Wave 2 (Plan 02-04) | ⬜ pending |
| R-2 | Stationary on-dock match within 5 cm | hardware | `mow_session_monitor.py` extension; operator checklist | manual | ⬜ pending |
| R-3 | Confidence trips false within 1 s of corrupt scan | unit | gtest `ConfidenceMetricsRejectsNoise` (synthetic noisy points fed to confidence wrapper) | Wave 1 (Plan 02-02) | ⬜ pending |
| R-3 | trust threshold respects ROS params (`min_inlier_ratio`, `max_rmse_m`) | unit | gtest `ConfidenceMetricsParamOverrides` | Wave 2 (Plan 02-04) | ⬜ pending |
| R-4 | Cascade priority `dock_match.pose` > `dock_calibration.yaml` > `/gnss/heading` | unit (Python) | `pytest ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py` | Wave 2 (Plan 02-05) | ⬜ pending |
| R-5 | RecordDockApproachPose writes correct `dock_approach.yaml` (yaw_to_dock = atan2(dock-here)) | unit (C++) | gtest `RecordDockApproachPoseGeometry` (mocked TF + dock_match) | Wave 3 (Plan 02-06) | ⬜ pending |
| R-5 | RecordDockApproachPose falls back to TF when dock_match untrusted | unit | gtest `RecordDockApproachPoseTfFallback` | Wave 3 (Plan 02-06) | ⬜ pending |
| R-6 | ApproachDock returns SUCCESS within 60 s under healthy conditions | unit (mocked NavigateToPose) | gtest `ApproachDockMockClient` | Wave 3 (Plan 02-06) | ⬜ pending |
| R-6 | ApproachDock returns FAILURE immediately on missing yaml | unit | gtest `ApproachDockNoYaml` | Wave 3 (Plan 02-06) | ⬜ pending |
| R-7 | FineDock control loop converges in sim (synthetic_scan_kicp_publisher) | sim (e2e) | new e2e_test phase `_run_fine_dock_phase` | Wave 5 (Plan 02-08) | ⬜ pending |
| R-7 | FineDock achieves ≤ 2 cm lateral + ≤ 1° yaw at first contact | hardware (5-of-5) | `mow_session_monitor.py` + `02-08-PI5-CHECKLIST.md` operator-gated | manual | ⬜ pending (operator) |
| R-8 | FineDock issues `cmd_vel=0` on confidence loss within 1 s | unit | gtest `FineDockAbortsOnConfidenceLoss` (mock /dock_match publisher) | Wave 3 (Plan 02-06) | ⬜ pending |
| R-8 | FineDock degraded-mode (no pose received) bails after 5 s | unit | gtest `FineDockBailsAfter5sNoPose` (added per Iteration-1 Warning 9 fix) | Wave 3 (Plan 02-06) | ⬜ pending |
| R-9 | E-Stop during FineDock stops motion + no auto-restart | unit (gtest) + hardware (manual) | gtest `FineDockHaltsOnEmergency`; operator presses estop during sim | unit Wave 3 / hw manual | ⬜ pending |
| R-10 | All `<DockRobot ` literal tags removed from main_tree.xml | smoke / lint | `grep -c "<DockRobot " ros2/src/mowgli_behavior/trees/main_tree.xml` returns 0 (note `<` and trailing space — historical comments at lines 84/123/463/481/503/515/518/520 PRESERVED, see Iteration-1 Blocker-1 fix) | Wave 4 (Plan 02-07) | ⬜ pending |
| R-11 | Auto-refresh on 8-day-old + inlier ≥ 0.95 → PCD updated | unit | gtest `AutoRefreshTriggersOnHighConfidence` (mock filesystem timestamps) | Wave 3 (Plan 02-06) | ⬜ pending |
| R-11 | Auto-refresh skipped when inlier < 0.95 (kept stale, log WARN) | unit | gtest `AutoRefreshSkipsOnLowConfidence` | Wave 3 (Plan 02-06) | ⬜ pending |
| R-11 | dock_scan_match hot-reloads PCD on mtime change (file watcher) | unit | gtest `DockScanMatchReloadsOnMtimeChange` | Wave 2 (Plan 02-04) | ⬜ pending |
| R-12 | PreUndockClearanceCheck FAILS on rear obstacle within 1.7 m | unit | gtest `PreUndockClearanceFailsOnRearObstacle` (synthetic LaserScan with 0.5 m return at 180°) | Wave 3 (Plan 02-06) | ⬜ pending |
| R-12 | PreUndockClearanceCheck SUCCEEDS when rear clear | unit | gtest `PreUndockClearanceSucceedsWhenClear` | Wave 3 (Plan 02-06) | ⬜ pending |
| R-13 | PostUndockRtkValidation logs WARN at 0.5–1.5 m discrepancy | unit | gtest `PostUndockRtkValidationWarnsAtMidGap` | Wave 3 (Plan 02-06) | ⬜ pending |
| R-13 | PostUndockRtkValidation FAILS at > 1.5 m discrepancy | unit | gtest `PostUndockRtkValidationFailsAtLargeGap` | Wave 3 (Plan 02-06) | ⬜ pending |
| End-to-end | COMMAND_START → undock (with PreUndockClearanceCheck + PostUndockRtkValidation) → mow → autodock first attempt | sim + hardware | extended `e2e_test.py` `_run_fine_dock_phase` (sim); 5-of-5 operator runs (hw) | Wave 5 (Plan 02-08) | ⬜ pending |
| Drift verification | 7-day window auto-refresh fires at least once with inlier ≥ 0.95 | hardware (long-running) | `mow_session_monitor.py` JSONL inspection across days; not automatable in CI | manual | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements (test infrastructure)

> Wave 0 (Plan 02-01) must establish all test infrastructure + framework deps before Wave 1 implementation begins. Initial seed list extracted from RESEARCH §Validation Architecture > Wave 0 Gaps. Plans 02-02 + 02-03 expand it.

- [ ] Apt deps: `ros-kilted-laser-geometry`, `ros-kilted-pcl-conversions`, `ros-kilted-pcl-ros`, `libpcl-dev`, `libsophus-dev` (also added to `ros2/Dockerfile` base stage)
- [ ] Submodule init: `git submodule update --init --recursive ros2/src/kinematic_icp ros2/src/kiss_icp` + verify headers `kinematic_icp/pipeline/KinematicICP.hpp` + `kiss_icp/core/VoxelHashMap.hpp` exist
- [ ] kiss_icp public-API probe (assumptions A1, A2 from RESEARCH): confirm `VoxelHashMap::AddPoints` and `VoxelHashMap::GetClosestNeighbor` are public methods
- [ ] `ros2/src/mowgli_interfaces/msg/DockMatchConfidence.msg` defined + 3-pipeline codegen run (firmware/sync_ros_lib.py + gui/generate_go_msgs.sh + gui/generate_ts_types.sh)
- [ ] `ros2/src/mowgli_geometry/include/mowgli_geometry/key_value_parser.hpp` shared parser promoted (extends Phase 1 atomic_write helper; consumed by `dock_calibration_loader`, `dock_approach_loader`, `dock_scan_meta_loader`)
- [ ] `ros2/src/mowgli_lidar_docking/test/CMakeLists.txt` — gtest pattern (mirrors Phase 1 `mowgli_coverage_planner`)
- [ ] `ros2/src/mowgli_lidar_docking/test/test_dock_scan_io.cpp` — PCD round-trip
- [ ] `ros2/src/mowgli_lidar_docking/test/test_confidence_metrics.cpp` — inlier_ratio + rmse against synthetic scans
- [ ] `ros2/src/mowgli_lidar_docking/test/test_dock_approach_loader.cpp` — yaml parser corruption rejection
- [ ] `ros2/src/mowgli_lidar_docking/test/test_dock_scan_meta_loader.cpp` — yaml parser round-trip
- [ ] `ros2/src/mowgli_lidar_docking/test/test_idock_matcher_mock.cpp` — D-16 wrapper interface contract
- [ ] `ros2/src/mowgli_behavior/test/test_docking_nodes.cpp` — extends existing test/ for new BT nodes (12+ test cases)
- [ ] `ros2/src/mowgli_simulation/scripts/synthetic_scan_kicp_publisher.py` — D-14 sim publisher
- [ ] `ros2/src/mowgli_simulation/test_data/sim_dock_scan.pcd` — pre-baked binary for sim acceptance
- [ ] `ros2/src/mowgli_simulation/test_data/sim_dock_scan_meta.yaml` — companion metadata
- [ ] `ros2/src/e2e_test.py` extension: `_run_fine_dock_phase` Python method + sim-dock placement
- [ ] `ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py` — Python pytest for R-4
- [ ] `ros2/src/mowgli_localization/test/test_calibrate_imu_yaw_capture.py` — Python pytest for R-1 (synthetic /scan_kicp)
- [ ] `ros2/src/mowgli_simulation/test/test_synthetic_scan_publisher.py` — sim publisher contract test

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Pi5 hardware acceptance: 5 of 5 consecutive autodock cycles with `is_charging` engaged on first attempt | SPEC R-7 (acceptance gate) + End-to-end | Requires physical Pi5 + dock + RTK base; cannot be sim-faithful (real LiDAR returns + multipath + grass terrain) | (1) Operator deploys image to Pi5 via existing auto-deploy. (2) Robot starts docked + charging. (3) Operator runs `mow_session_monitor.py --session 2026-XX-XX-phase2-acceptance-N` for each cycle (1..5). (4) Send COMMAND_START. (5) After mow + autodock, confirm: `is_charging == true`, lateral_error_at_contact ≤ 2 cm, yaw_error ≤ 1° (from JSONL `cross_checks.dock_match.lateral_error_at_contact_m`). (6) Repeat cycles 2–5. (7) Commit JSONLs. (8) Fill `02-08-PI5-RESULTS.md` with pass/fail per cycle. (9) Phase 2 verification flips R-7 to ✅ only when all 5 pass. |
| 7-day drift verification: dock_scan.pcd auto-refresh fires at least once with inlier_ratio ≥ 0.95 logged | SPEC R-11 (drift gate) | Requires multi-day live operation; not automatable in CI | (1) After Pi5 image deployment, set initial `dock_scan_meta.yaml` timestamp ≥ 7 days back. (2) Run normal mow cycles for ≥ 7 days. (3) Monitor `dock_scan.pcd` mtime + log lines for "auto-refresh triggered, new inlier_ratio: X". (4) Operator confirms refresh occurred AND no false positives (refresh skipped when confidence < 0.95). |
| GUI Dock-card LiDAR-confidence display renders correctly | SPEC implicit (D-09, D-10, D-11) | Operator browser inspection | (1) Open GUI in browser. (2) Verify Dock-card has new section with green/yellow/red status badge + "Inlier X% / RMSE Y cm" pair. (3) Verify "Recapture dock scan" button opens confirm modal. (4) Verify modal "Cancel" + "Capture" buttons work. (5) Verify confidence value updates at ~1 Hz. |

---

## Validation Sign-Off

- [ ] All R-1..R-13 requirements have at least one automated test row (verified by grep over the table)
- [ ] All test commands are colcon-test or pytest invocations runnable from `cd ros2`
- [ ] Wave 0 list aligns with Plan 02-01 + 02-02 + 02-03 task lists
- [ ] Manual-only verifications are explicitly operator-gated (`autonomous: false` in plan frontmatter)
- [ ] Phase 2 cannot close until 5-of-5 hardware acceptance is committed to `02-08-PI5-RESULTS.md`

---

## Drift Check (executed by Plan 02-01 Task 4)

Plan 02-01 Task 4 verifies that this VALIDATION.md still matches `02-RESEARCH.md` §Validation Architecture (no silent drift between research and execution). The check is a simple grep:

```bash
grep -c "^| R-" .planning/phases/02-lidar-dock-pose-estimation/02-VALIDATION.md
# Expected: ≥ 13 (one row per R-XX requirement, several have multiple test rows)
```

If new R-XX entries are added to RESEARCH §Validation Architecture mid-execution, this VALIDATION.md must be updated and re-committed.
