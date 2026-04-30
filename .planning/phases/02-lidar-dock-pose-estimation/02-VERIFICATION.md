---
phase: 02-lidar-dock-pose-estimation
status: pending-pi5-hardware
created: 2026-04-30
last_updated: 2026-04-30
score: pending (automatable scope COMPLETE; Pi5 5-of-5 hardware UAT pending)
---

# Phase 2 — LiDAR Dock Pose Estimation: Verification Report

> Per-requirement acceptance matrix for Phase 2. Mirrors the Phase 1
> 01-VERIFICATION.md style (table per requirement, status + evidence).
> Source-of-truth for "what's left to prove before Phase 2 closes".
>
> Initial state: every row is `⬜ pending` until either the corresponding
> automated test runs green in CI (automatable scope) OR the operator
> commits 02-08-PI5-RESULTS.md (hardware scope). The verifier
> (`/gsd-verify-phase 2`) flips rows to `✓ VERIFIED` with commit hashes
> as evidence as those gates close.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| Framework | `ament_cmake_gtest` (C++ unit/integration), pytest (Python rclpy + smoke), launch_testing (Python E2E sim) |
| Quick run | `cd ros2 && colcon test --packages-select mowgli_lidar_docking mowgli_behavior mowgli_localization mowgli_simulation --event-handlers console_cohesion+ --ctest-args --output-on-failure` |
| Full sim | `cd ros2 && make build && make test && make e2e-test` (the e2e_test phase `_run_fine_dock_phase` exercises Plan 02-08's sim composition) |
| Hardware bench | Pi5 @ `pi@10.10.40.68`, RTK base in Eichenau garden, operator-driven 5-of-5 mow cycles per `02-08-PI5-CHECKLIST.md` |
| Estimated runtime | quick: ~60 s; full + sim: ~6–8 min; hardware: ~30–45 min for 5 cycles |

---

## Per-Requirement Acceptance Matrix

> Status legend: ⬜ pending · ✓ VERIFIED · ✗ FAILED · ⚠️ FLAKY · 🔧 hardware-only
>
> Multi-row requirements: a single row per (requirement, test) pair so
> partial passes are visible. R-7, R-9, R-11 each have one automated row
> AND one hardware row — the hardware row is the actual SPEC AC gate.

| Req  | Description                                                   | Test                                                                              | Source Plan | Status                | Evidence |
|------|---------------------------------------------------------------|-----------------------------------------------------------------------------------|-------------|-----------------------|----------|
| R-1  | dock_scan.pcd captured by calibrate_imu_yaw_node extension    | `pytest test_calibrate_imu_yaw_capture.py -k dock_scan`                           | 02-03       | ⬜ pending             | —        |
| R-1  | dock_scan_meta.yaml round-trips through key=value parser      | gtest `DockScanMetaRoundTrip`                                                     | 02-02       | ⬜ pending             | —        |
| R-1  | PCD readable by `pcl_viewer`                                  | manual / hardware checklist (PI5-CHECKLIST §pre-flight)                           | 02-08       | 🔧 pending: hardware  | —        |
| R-2  | `/dock_match/pose` published at ≥ 5 Hz                        | gtest `DockScanMatchNodeRateTest` (in-process node + mock matcher)                | 02-04       | ⬜ pending             | —        |
| R-2  | Stationary on-dock match within 5 cm                          | hardware (mow_session_monitor JSONL grep `dock_match.pose.{x,y}` while charging)  | 02-08       | 🔧 pending: hardware  | —        |
| R-3  | Confidence trips false within 1 s of corrupt scan             | gtest `ConfidenceMetricsRejectsNoise`                                             | 02-02       | ⬜ pending             | —        |
| R-3  | Trust threshold respects `min_inlier_ratio` / `max_rmse_m`    | gtest `ConfidenceMetricsParamOverrides`                                           | 02-04       | ⬜ pending             | —        |
| R-4  | Cascade priority dock_match > dock_calibration > /gnss/heading | `pytest test_dock_yaw_seeder_cascade.py` (7 cases)                                | 02-05       | ⬜ pending             | —        |
| R-5  | RecordDockApproachPose writes correct dock_approach.yaml      | gtest `RecordDockApproachPoseGeometry` (mocked TF + dock_match)                   | 02-06       | ⬜ pending             | —        |
| R-5  | RecordDockApproachPose TF fallback when dock_match untrusted  | gtest `RecordDockApproachPoseTfFallback`                                          | 02-06       | ⬜ pending             | —        |
| R-6  | ApproachDock SUCCESS within 60 s (healthy)                    | gtest `ApproachDockMockClient`                                                    | 02-06       | ⬜ pending             | —        |
| R-6  | ApproachDock FAILURE on missing yaml                          | gtest `ApproachDockNoYaml`                                                        | 02-06       | ⬜ pending             | —        |
| R-7  | FineDock control loop converges in sim                        | e2e_test `_run_fine_dock_phase` (synthetic publisher + dock_scan_match in sim)    | 02-08       | ⬜ pending             | —        |
| R-7  | FineDock ≤ 2 cm lateral + ≤ 1° yaw at first contact (hardware)| Pi5 5-of-5 acceptance per 02-08-PI5-CHECKLIST.md                                  | 02-08       | 🔧 pending: hardware  | —        |
| R-8  | FineDock issues `cmd_vel=0` on confidence loss within 1 s     | gtest `FineDockAbortsOnConfidenceLoss`                                            | 02-06       | ⬜ pending             | —        |
| R-8  | FineDock degraded-mode (no pose) bails after 5 s              | gtest `FineDockBailsAfter5sNoPose`                                                | 02-06       | ⬜ pending             | —        |
| R-9  | E-Stop during FineDock halts motion                           | gtest `FineDockHaltsOnEmergency`                                                  | 02-06       | ⬜ pending             | —        |
| R-9  | XML guard: ClearCommand + EmergencyHandler-no-restore         | code review on `main_tree.xml` (Plan 02-07 lines 165, 170-180, 318, 510, 538)     | 02-07       | ⬜ pending             | —        |
| R-9  | E-Stop during sim FineDock + no auto-restart after reset      | hardware (operator manual injection, PI5-CHECKLIST §cycle-3)                      | 02-08       | 🔧 pending: hardware  | —        |
| R-10 | All `<DockRobot ` literal tags removed from main_tree.xml     | smoke `grep -c "<DockRobot " main_tree.xml` returns 0 (literal `<` + space)       | 02-07       | ⬜ pending             | —        |
| R-11 | Auto-refresh on 8-day-old + inlier ≥ 0.95 → PCD updated       | gtest `AutoRefreshTriggersOnHighConfidence` (mocked filesystem timestamps)        | 02-06       | ⬜ pending             | —        |
| R-11 | Auto-refresh skipped when inlier < 0.95 (kept stale, log WARN)| gtest `AutoRefreshSkipsOnLowConfidence`                                           | 02-06       | ⬜ pending             | —        |
| R-11 | dock_scan_match hot-reloads PCD on mtime change               | gtest `DockScanMatchReloadsOnMtimeChange`                                         | 02-04       | ⬜ pending             | —        |
| R-11 | Auto-refresh fires on hardware (one cycle in 5)               | hardware (PI5-CHECKLIST §acceptance ⓒ)                                           | 02-08       | 🔧 pending: hardware  | —        |
| R-12 | PreUndockClearanceCheck FAILS on rear obstacle ≤ 1.7 m        | gtest `PreUndockClearanceFailsOnRearObstacle`                                     | 02-06       | ⬜ pending             | —        |
| R-12 | PreUndockClearanceCheck SUCCEEDS when rear clear              | gtest `PreUndockClearanceSucceedsWhenClear`                                       | 02-06       | ⬜ pending             | —        |
| R-13 | PostUndockRtkValidation logs WARN at 0.5–1.5 m discrepancy    | gtest `PostUndockRtkValidationWarnsAtMidGap`                                      | 02-06       | ⬜ pending             | —        |
| R-13 | PostUndockRtkValidation FAILS at > 1.5 m discrepancy          | gtest `PostUndockRtkValidationFailsAtLargeGap`                                    | 02-06       | ⬜ pending             | —        |
| E2E sim   | full FineDock convergence in headless Gazebo            | e2e_test `_run_fine_dock_phase`                                                   | 02-08       | ⬜ pending             | —        |
| Pi5 5-of-5 | Hardware acceptance (5 consecutive first-attempt docks) | operator runs PI5-CHECKLIST.md, commits 02-08-PI5-RESULTS.md                      | 02-08       | 🔧 pending: hardware  | —        |
| Drift      | 7-day window auto-refresh fires once with inlier ≥ 0.95 | hardware (multi-day mow_session_monitor JSONL inspection)                         | 02-08       | 🔧 pending: hardware  | —        |

---

## Plan-Level Summary Cross-Reference

| Plan  | Subsystem                                              | SUMMARY                          | Score (this report)        |
|-------|--------------------------------------------------------|----------------------------------|----------------------------|
| 02-01 | Test infrastructure + DockMatchConfidence msg + 3-pipeline codegen | `02-01-SUMMARY.md`               | covered by R-* gtest rows  |
| 02-02 | mowgli_lidar_docking package skeleton + dock_scan_io   | `02-02-SUMMARY.md`               | covered by R-1, R-3 rows   |
| 02-03 | dock_scan_capture (rclpy library)                      | `02-03-SUMMARY.md`               | covered by R-1 rows        |
| 02-04 | dock_scan_match node + KinematicIcpDockMatcher         | `02-04-SUMMARY.md`               | covered by R-2, R-3, R-11 rows |
| 02-05 | dock_yaw_to_set_pose cascade                           | `02-05-SUMMARY.md`               | covered by R-4 row         |
| 02-06 | 8 BT nodes (Approach/FineDock/RecordDockApproach/Pre*/Post*) | `02-06-SUMMARY.md`         | covered by R-5..R-9, R-11..R-13 rows |
| 02-07 | main_tree.xml wiring (R-10) + GUI dock-card extension  | `02-07-SUMMARY.md`               | covered by R-10 row        |
| 02-08 | Sim publisher + e2e + monitor + PI5 checklist (this plan) | (in progress)                | covered by E2E + Pi5 rows  |

---

## Manual / Hardware-Only Verifications

| Behavior | Requirement | Why Manual | Procedure |
|----------|-------------|------------|-----------|
| Pi5 5-of-5 acceptance — 5 consecutive autodock cycles, is_charging engaged on first attempt, ≤ 2 cm lateral / ≤ 1° yaw at contact | SPEC R-7 (acceptance gate) | Requires physical Pi5 + RTK + Eichenau garden; sim noise floor differs from real LD19 | `02-08-PI5-CHECKLIST.md` |
| 7-day drift verification — dock_scan.pcd auto-refresh fires once with inlier ≥ 0.95 logged | SPEC R-11 | Requires multi-day live operation; not automatable in CI | Set initial dock_scan_meta.yaml.captured_at ≥ 7 days back; run normal mow cycles for ≥ 7 days; grep refresh log + diff dock_scan.pcd mtime |
| GUI Dock-card LiDAR-confidence rendering | SPEC implicit (D-09, D-10, D-11) | Operator browser inspection | Open GUI; verify Dock-card has status badge + "Inlier X% / RMSE Y cm" + Recapture button + confirm modal |
| R-9 manual E-Stop during FineDock + no auto-restart after ResetEmergency | SPEC R-9 | Requires physical E-Stop + dock approach in progress | PI5-CHECKLIST §cycle-3 |

---

## Validation Sign-Off

- [ ] All R-1..R-13 requirements have at least one automated test row in this matrix (verified by grep over the table)
- [ ] All test commands are colcon-test or pytest invocations runnable from `cd ros2`
- [ ] Manual / hardware verifications are explicitly operator-gated (each row tagged 🔧)
- [ ] Pi5 5-of-5 result file (02-08-PI5-RESULTS.md) exists with operator-reported verdicts
- [ ] Phase 2 cannot close until 02-08-PI5-RESULTS.md is committed AND `/gsd-verify-phase 2` flips R-7 / R-9 / R-11 hardware rows to ✓ VERIFIED

---

## Status as of file creation

- **Automatable scope (Plans 02-01 through 02-08 Tasks 1+2)**: COMPLETE on disk; deferred to phase-end podman build for actual colcon-test execution. STATE.md "Deferred verify steps" tracks every DEFERRED-TO-PHASE-END-BUILD entry.
- **Plan 02-08 Task 3 (Pi5 5-of-5 hardware)**: pending operator verification on the Eichenau test bench.
- **Phase 2 closure**: blocked on Task 3 + the phase-end podman build.

This report is intentionally a `pending`-heavy matrix. The verifier
(`/gsd-verify-phase 2`) is responsible for flipping individual rows to
`✓ VERIFIED` with commit-hash evidence as the underlying gates close.
