# Phase 2: LiDAR-based dock pose estimation — Specification

**Created:** 2026-04-29
**Ambiguity score:** 0.105 (gate: ≤ 0.20)
**Requirements:** 11 locked

## Goal

Replace the open-loop GPS-only docking final approach with a closed-loop LiDAR-fine approach that achieves ≤ 2 cm lateral and ≤ 1° yaw error at first contact, by capturing a one-time `dock_scan.pcd` snapshot, recording the exact post-undock pose as `dock_approach.yaml`, and crawling the final 1.5 m under continuous `/scan_kicp` ICP correction. Same scan-match infrastructure also feeds `#43` Smart Undock's pre-undock free-space check + post-undock RTK validation.

## Background

End-to-end mow on 2026-04-29 confirmed the operator-blocking failure mode: even with continuous RTK-Fixed conditions, the robot misses the dock V-funnel by ~10 cm. Measured GPS at "on dock" position (3.876, 2.745) vs `dock_calibration.yaml` (3.885, 2.644) — a 10 cm Y-offset that compounds with the open-loop final approach of `opennav_docking::SimpleChargingDock`.

**What exists today (relevant to this phase):**
- `kinematic_icp_scan_frame_relay.py` already publishes `/scan_kicp` at the correct sensor frame on the parallel TF tree (`lidar_link_wheels`). Substrate for our scan-match is in place.
- `dock_yaw_to_set_pose.py` is the existing single-source EKF seeder with a priority cascade (file → /gnss/heading). Phase 2 extends the cascade with `dock_match.pose` at the top.
- `dock_calibration.yaml` is the existing source of truth for dock pose. Read by 4 consumers (`navigation.launch.py`, `hardware_bridge`, `behavior_tree_node`, `dock_yaw_to_set_pose`). Persists across container restarts since #74.
- `RecordUndockStart` / `CalibrateHeadingFromUndock` BT nodes (just refactored to use `odom→base_footprint` TF in #73) — natural extension point for `RecordDockApproachPose`.
- 4 `DockRobot` BT calls in `main_tree.xml` (CriticalBatteryDock, RainDockAndResume, BatteryDockAndResume, FailedCoverageDock) — all 4 will be migrated to FineDock.
- `calibrate_imu_yaw_node.py` runs the operator-confirmed dock-yaw calibration (existing GUI button). Extension point for the initial `dock_scan.pcd` capture.
- Architecture Invariant #10: opennav_docking's `UndockRobot` is deprecated; Nav2 `BackUp` behavior used instead. Phase 2 does NOT reintroduce UndockRobot — `FineDock` is a new BT node, not a Nav2 dock plugin.

**What does NOT exist yet:**
- No `dock_scan.pcd` capture node
- No `dock_scan_match` ICP runner
- No `dock_approach.yaml` writer or reader
- No `ApproachDock` or `FineDock` BT nodes
- No GUI Dock-card LiDAR-confidence display
- No `RecordDockApproachPose` BT extension
- No 7-day auto-refresh of `dock_scan.pcd`

## Requirements

1. **dock_scan.pcd capture extends calibrate_imu_yaw_node**: The first `dock_scan.pcd` is written when the operator runs the existing dock-yaw calibration, with no extra operator-side step.
   - Current: `calibrate_imu_yaw_node.py` writes `dock_calibration.yaml` only
   - Target: After successful calibration, the node also captures a `/scan_kicp` snapshot at the docked pose and writes it to `/ros2_ws/maps/dock_scan.pcd` plus a metadata YAML (`/ros2_ws/maps/dock_scan_meta.yaml` with timestamp, dock pose, sensor extrinsic, ROS2 fix type)
   - Acceptance: Running the dock-yaw calibration on a fresh test setup writes both files; `pcl_viewer` opens the `.pcd`; metadata YAML round-trips through `yaml.safe_load`

2. **dock_scan_match node continuously publishes ICP-derived pose**: A new node performs scan-vs-snapshot ICP and publishes the result.
   - Current: No such node exists
   - Target: New ROS2 node `dock_scan_match` subscribes `/scan_kicp`, loads `dock_scan.pcd` at startup, runs ICP using the PRBonn Kinematic-ICP solver (in-tree at `ros2/src/kinematic_icp/`), publishes `/dock_match/pose` (PoseWithCovarianceStamped, map frame) at minimum 5 Hz, and publishes `/dock_match/confidence` (custom msg with `inlier_ratio` + `rmse_m`) at the same cadence
   - Acceptance: With the robot stationary on the dock, `/dock_match/pose` reports a position within 5 cm of the true dock pose for ≥ 95% of samples over 30 s; with the robot 1 m off-dock, the published pose tracks the GPS position to within 5 cm

3. **ICP trust threshold is enforced**: `dock_scan_match` self-classifies its output as trustworthy or not.
   - Current: No such gate
   - Target: A match is "trusted" iff `inlier_ratio ≥ 0.70` AND `rmse ≤ 0.05` (overridable via ROS params `min_inlier_ratio` and `max_rmse_m`). `/dock_match/confidence.trusted` is a bool reflecting this; downstream consumers (FineDock, dock_yaw_to_set_pose, RecordDockApproachPose) MUST gate on `.trusted`
   - Acceptance: Synthetic test: feed the node a corrupted `/scan_kicp` (random noise replacing 50% of points); `/dock_match/confidence.trusted` reports false within 1 s

4. **dock_yaw_to_set_pose extends seed-priority cascade with `/dock_match/pose`**: When LiDAR match is trusted, it seeds the EKF in preference to the YAML-derived dock yaw.
   - Current: Cascade is `dock_calibration.yaml` (file) > `/gnss/heading` (live)
   - Target: Cascade becomes `/dock_match/pose` (LiDAR, gated on `.trusted`) > `dock_calibration.yaml` (file) > `/gnss/heading` (live)
   - Acceptance: With LiDAR available + trusted, log line shows source=lidar; with LiDAR untrusted, source falls back to file; with neither file nor LiDAR, source falls back to /gnss/heading

5. **RecordDockApproachPose BT node writes `dock_approach.yaml` at end of UndockSequence**: The exact post-undock pose is persisted for next docking.
   - Current: No such node; UndockSequence ends without persisting the outbound pose
   - Target: New BT node `RecordDockApproachPose` runs after `CalibrateHeadingFromUndock`. Reads current `/dock_match/pose` (preferred, when trusted) or `map→base_footprint` TF (fallback). Writes `/ros2_ws/maps/dock_approach.yaml` with keys `dock_approach_x`, `dock_approach_y`, `dock_approach_yaw_to_dock_rad`, `source` (lidar/tf), `timestamp`, where `yaw_to_dock = atan2(dock.y - here.y, dock.x - here.x)`
   - Acceptance: After a successful undock, the YAML file exists and `dock_approach_x` is within 5 cm of the wheel-odom-derived end pose; `yaw_to_dock` points within 1° of the bearing from end pose to `dock_calibration.yaml` pose

6. **ApproachDock BT node replaces opennav_docking RPP approach with NavigateToPose to dock_approach.yaml**: RTK-coarse approach lands the robot exactly where the last undock ended.
   - Current: All 4 `DockRobot` BT calls go through `opennav_docking::SimpleChargingDock` which navigates to the static `home_dock.pose`
   - Target: New BT node `ApproachDock` loads `dock_approach.yaml`, sends `NavigateToPose` goal to `(dock_approach_x, dock_approach_y, dock_approach_yaw_to_dock_rad)`. Returns SUCCESS when the goal is reached within 10 cm + 5° tolerance. Returns FAILURE if NavigateToPose fails or `dock_approach.yaml` is missing
   - Acceptance: With a valid `dock_approach.yaml` and clear path, `ApproachDock` returns SUCCESS in < 60 s; without `dock_approach.yaml`, returns FAILURE immediately

7. **FineDock BT node closed-loop crawls the final 1.5 m**: Replaces the final approach segment of every dock with continuous LiDAR correction.
   - Current: opennav_docking SimpleChargingDock plugin handles the final approach blind
   - Target: New BT node `FineDock` runs after `ApproachDock`. Subscribes `/dock_match/pose` (gated on `.trusted`). Computes lateral_y error and yaw error in the dock frame. Drives `cmd_vel.x = +0.05 m/s` (configurable param `crawl_speed_ms`) with P-controllers on lateral_y (output: angular_z) and yaw (output: angular_z component). Stops on `is_charging == true`. Publishes to `/cmd_vel_docking` (twist_mux priority 15)
   - Acceptance: From a known starting pose 1.5 m before the dock, FineDock drives the robot to `is_charging` within 60 s; lateral error at contact (measured by post-contact LiDAR pose) is ≤ 2 cm; yaw error is ≤ 1°

8. **FineDock aborts on confidence loss mid-approach**: If LiDAR match degrades during crawl, the robot stops and reports failure.
   - Current: N/A
   - Target: If `/dock_match/confidence.trusted == false` for ≥ 1 s during FineDock execution, the node publishes `cmd_vel = 0`, returns FAILURE, and a log line at `LOG_ERROR` level explains why (e.g., "LiDAR ICP confidence dropped: inlier_ratio=0.42 rmse=0.08 — aborting fine dock")
   - Acceptance: Inject a corrupted `/scan_kicp` mid-crawl in test; FineDock issues `cmd_vel=0` within 1 s, returns FAILURE, and the log line is present

9. **Emergency stop mid-FineDock holds and waits for explicit Operator action**: E-Stop during fine-dock follows the standard hold-and-restart pattern.
   - Current: E-Stop during opennav_docking docking stops the motion; opennav resumes if the goal is still active
   - Target: When emergency is asserted during FineDock, `cmd_vel = 0` is published immediately (firmware also enforces). FineDock returns FAILURE on the next tick. After Operator clears emergency (manual `ResetEmergency` or auto-reset on dock charging), FineDock does NOT auto-restart — Operator must explicitly re-trigger via the GUI or BT command flow
   - Acceptance: Manual E-Stop test during FineDock: motion stops within firmware deadline (< 200 ms); after ResetEmergency, FineDock does not start a new approach automatically; sending COMMAND_HOME again triggers a fresh ApproachDock + FineDock cycle

10. **All 4 DockRobot BT calls migrate to ApproachDock + FineDock**: Consistent docking behavior across every BT path.
    - Current: 4 `<DockRobot dock_id="home_dock" dock_type="simple_charging_dock"/>` calls in `main_tree.xml` at CriticalBatteryDock (line 171), RainDockAndResume (line 332), BatteryDockAndResume (line 378), FailedCoverageDock (line 445)
    - Target: All 4 sites replaced with a `<Sequence><ApproachDock/><FineDock/></Sequence>` subtree (or a reusable BT subtree node `LidarDock` that wraps both)
    - Acceptance: `grep -c "DockRobot" main_tree.xml` returns 0; all 4 dock-trigger code paths exercise the new subtree end-to-end in BT unit tests

11. **dock_scan.pcd auto-refreshes after 7 days when undock confidence ≥ 95%**: Keeps the snapshot current against environmental drift without operator intervention.
    - Current: N/A
    - Target: At end of every successful UndockSequence, `RecordDockApproachPose` checks `dock_scan_meta.yaml` timestamp. If file age > 7 days AND that undock's `/dock_match/confidence.inlier_ratio ≥ 0.95`, the node also writes a fresh `dock_scan.pcd` from the current `/scan_kicp` snapshot. If file age > 7 days but confidence < 0.95, log a warning and skip the refresh (keep the older snapshot)
    - Acceptance: Set `dock_scan_meta.yaml` timestamp 8 days back; trigger an undock with synthetic `inlier_ratio=0.97`; verify `dock_scan.pcd` mtime updates and the new metadata reflects today. Same setup with `inlier_ratio=0.85`; verify the file is NOT updated and a warning is logged

## Smart Undock requirements (from #43 — bundled into Phase 2)

12. **Pre-undock LiDAR free-space check**: Before issuing the BackUp, the rear sector of `/scan_kicp` is queried for clearance.
    - Current: `BackUp` runs unconditionally — robot can drive into a person standing behind the dock
    - Target: New BT node `PreUndockClearanceCheck` runs before `BackUp`. Queries the rear sector (180° ± `rear_sector_half_deg`, default 30°) of `/scan_kicp` for the minimum point distance. If clearance < (`min_undock_distance_m` + `rear_safety_buffer_m`, default 0.20 m), node returns FAILURE and `UndockSequence` aborts before any motion. If clearance ≥ threshold, node returns SUCCESS and BackUp proceeds
    - Acceptance: Place a test obstacle 0.5 m behind the dock; trigger undock; verify `PreUndockClearanceCheck` returns FAILURE and logs "rear clearance 0.50 m < required 1.70 m"; remove obstacle and re-trigger; verify SUCCESS

13. **Post-undock RTK position validation**: After BackUp, the wheel-odom-predicted pose is compared against the GPS pose.
    - Current: `CalibrateHeadingFromUndock` derives yaw from odom displacement only — does not cross-check against GPS
    - Target: New BT node `PostUndockRtkValidation` runs after `CalibrateHeadingFromUndock`. Reads current `/gps/absolute_pose` and the odom→base_footprint pose at undock end. Computes `error_xy = ||gps_pose - odom_predicted_pose||`. If RTK fix is `RTK_FIXED` AND `error_xy > 0.5 m`, returns WARN (logs the discrepancy, marks `BTContext::dock_pose_suspect = true`). If `error_xy > 1.5 m`, returns FAILURE (UndockSequence aborts; operator alerted)
    - Acceptance: With matching odom and GPS, returns SUCCESS silently. With injected 0.7 m discrepancy, returns SUCCESS but logs WARN. With injected 2 m discrepancy, returns FAILURE.

## Boundaries

**In scope:**
- 6 new components: `dock_scan_capture` (extension to calibrate_imu_yaw_node), `dock_scan_match` (new ROS2 node), `RecordDockApproachPose` (new BT node), `ApproachDock` (new BT node), `FineDock` (new BT node), GUI Dock-card extension
- 2 additional BT nodes from #43: `PreUndockClearanceCheck`, `PostUndockRtkValidation`
- Migration of all 4 `DockRobot` BT calls to the new subtree
- Extension of `dock_yaw_to_set_pose` priority cascade to include `/dock_match/pose`
- Auto-refresh of `dock_scan.pcd` after 7 days when confidence ≥ 95%
- 2 new YAML files: `dock_scan_meta.yaml`, `dock_approach.yaml` (alongside existing `dock_calibration.yaml`)
- 1 new PCD file: `dock_scan.pcd`
- Custom msg type for `dock_match.confidence` (under `mowgli_interfaces`)
- New ROS2 package `mowgli_lidar_docking` (or extension to `mowgli_localization` — discuss-phase decides)
- GUI Dock-card display: ICP confidence (inlier_ratio + rmse), dock_scan.pcd age in days, last-fine-dock lateral error in cm

**Out of scope:**
- Replacing `opennav_docking` framework entirely — FineDock is a parallel BT node, the opennav_docking package stays installed for future use (e.g., as a fallback if Phase 2 is disabled via param)
- Implementing a custom Nav2 ChargingDock plugin (`SimpleChargingDockLidar`) — separate future work, this phase uses the BT-node approach instead
- Parametric LiDAR feature-detection of the dock structure (e.g., RANSAC for the V-funnel) — we use raw scan-vs-snapshot ICP only
- Multi-dock support — single `home_dock` only, identical to current code
- Dock recognition from arbitrary LiDAR scans (i.e., "auto-detect dock anywhere") — `dock_scan.pcd` must be captured first via the calibrate_imu_yaw_node extension
- 3D point cloud handling — LD19 is 2D LiDAR, all ICP runs on 2D scan vs 2D snapshot
- LiDAR feature for navigation in mow areas — the LiDAR remains read-only for everything outside docking and obstacle avoidance via collision_monitor (Architecture Invariant #5)
- Encryption / signing of the captured `dock_scan.pcd` — local file only, no security boundary
- E-Stop auto-restart — Operator must explicitly re-trigger COMMAND_HOME after E-Stop
- Pre-mow vs post-mow differentiation — every `DockRobot` trigger uses the new subtree (see Requirement 10)
- BCD or multi-pass coverage planner changes — Phase 1 scope, already done
- Smooth outline transitions (#70) — explicitly demoted to Phase 3
- Live coverage visualisation in GUI (#71, #72) — Phase 4

## Constraints

- **ICP solver:** PRBonn Kinematic-ICP (in-tree at `ros2/src/kinematic_icp/`). PCL ICP and Open3D explicitly rejected (kinematic constraints needed for robust 2D dock geometry; PCL would hallucinate symmetric matches).
- **Trust threshold:** `inlier_ratio ≥ 0.70` AND `rmse ≤ 0.05 m` (both overridable via ROS params).
- **Refresh threshold:** `inlier_ratio ≥ 0.95` for the auto-refresh trigger (stricter than the trust threshold to ensure the snapshot itself is high-quality).
- **FineDock crawl speed:** 0.05 m/s default (configurable). Empirically slow enough that ICP at 5 Hz can correct lateral drift before significant translation; fast enough that 1.5 m approach completes in ≤ 30 s.
- **Frame consistency:** all components operate in the `map` frame (where the dock is anchored). `dock_scan.pcd` is captured in `lidar_link_wheels` frame (parallel TF tree); ICP transforms scan-frame poses to map-frame using the static `lidar_link_wheels → base_footprint_wheels` extrinsic + `map → base_footprint_wheels` lookup at capture time.
- **Twist mux priority:** FineDock publishes to `/cmd_vel_docking` (priority 15, already configured per CLAUDE.md invariant #13). Does not introduce a new twist source.
- **Hardware Architecture Invariant compliance:** does not violate #1 (single localizer), #5 (costmap obstacles disabled in coverage mode — fine_dock is post-coverage), #10 (no UndockRobot reintroduction), #13 (cmd_vel routing). New invariant candidate: "FineDock is the sole closed-loop dock approach — opennav_docking SimpleChargingDock is reserved for fallback only."
- **No new hardware required.** Uses existing LD19 LiDAR + AltIMU-10v6 + UM980 GPS. All Pi5 capacity.
- **Cross-platform parity:** must work on both Pi5 (production) and the simulation Gazebo world (for E2E tests). Sim must publish a synthetic `/scan_kicp` for the simulated dock.
- **Persistence layout:** all new YAML files live under `/ros2_ws/maps/` alongside `dock_calibration.yaml`. PCD file is < 100 KB (LD19 LaserScan→PointCloud, single rev).

## Acceptance Criteria

- [ ] `dock_scan.pcd` is created by the existing dock-yaw calibration GUI button without any extra operator step (Requirement 1)
- [ ] `/dock_match/pose` is published at ≥ 5 Hz when LiDAR is healthy (Requirement 2)
- [ ] `/dock_match/confidence.trusted` flips false within 1 s of injected scan corruption (Requirement 3)
- [ ] `dock_yaw_to_set_pose` log line shows the resolved seed source (lidar/file/heading) per cycle (Requirement 4)
- [ ] `dock_approach.yaml` is written at end of every successful UndockSequence with correct `yaw_to_dock` (Requirement 5)
- [ ] `ApproachDock` BT node returns SUCCESS within 60 s under healthy conditions; FAILURE on missing yaml (Requirement 6)
- [ ] FineDock achieves ≤ 2 cm lateral error AND ≤ 1° yaw error at first contact in 5 of 5 consecutive runs (Requirement 7)
- [ ] FineDock issues `cmd_vel=0` within 1 s of injected confidence loss and logs the abort reason (Requirement 8)
- [ ] E-Stop during FineDock stops motion within firmware deadline; no auto-restart after ResetEmergency (Requirement 9)
- [ ] `grep -c "DockRobot" main_tree.xml` returns 0 (Requirement 10)
- [ ] Auto-refresh: synthetic 8-day-old metadata + `inlier_ratio=0.97` triggers PCD update; `inlier_ratio=0.85` skips with warning (Requirement 11)
- [ ] PreUndockClearanceCheck fails when rear obstacle within 1.7 m, succeeds when clear (Requirement 12)
- [ ] PostUndockRtkValidation logs WARN at 0.5–1.5 m discrepancy, fails > 1.5 m (Requirement 13)
- [ ] **End-to-end:** COMMAND_START → undock (with PreUndockClearanceCheck + PostUndockRtkValidation) → mow → return → autodock with `is_charging` engaged on first attempt
- [ ] **Drift verification:** after 7 days of normal operation, `dock_scan.pcd` has been auto-refreshed at least once with `inlier_ratio ≥ 0.95` log line proving it
- [ ] All BT unit tests pass; new gtest cases cover `RecordDockApproachPose`, `ApproachDock` (mocked NavigateToPose), `FineDock` (mocked /dock_match/pose), `PreUndockClearanceCheck`, `PostUndockRtkValidation`
- [ ] No regressions in existing planner / undock / coverage tests
- [ ] Pi5 hardware smoke: 3 consecutive end-to-end mow cycles with autodock success on first attempt

## Ambiguity Report

| Dimension          | Score | Min  | Status | Notes                                                  |
|--------------------|-------|------|--------|--------------------------------------------------------|
| Goal Clarity       | 0.90  | 0.75 | ✓      | Quantitative end-to-end target (≤2cm + ≤1°)            |
| Boundary Clarity   | 0.92  | 0.70 | ✓      | All 4 DockRobot sites named; explicit out-of-scope     |
| Constraint Clarity | 0.90  | 0.65 | ✓      | Solver, thresholds, frequencies, frames all locked     |
| Acceptance Criteria| 0.85  | 0.70 | ✓      | 17 pass/fail criteria, including 5-of-5 hardware run   |
| **Ambiguity**      | 0.105 | ≤0.20| ✓      |                                                        |

## Interview Log

| Round | Perspective       | Question summary                              | Decision locked                                                                       |
|-------|-------------------|-----------------------------------------------|---------------------------------------------------------------------------------------|
| 1     | Researcher        | Phase scope: #43 + #75 vs split?              | #43 + #75 zusammen (full bundle) — shared scan-match infrastructure                   |
| 1     | Researcher        | Pre-mow vs post-mow dock trigger?             | Immer wenn DockRobot getriggered wird — universal replacement                          |
| 1     | Researcher        | Which ICP solver?                             | PRBonn Kinematic-ICP (in-tree, kinematic constraints)                                 |
| 2     | Boundary Keeper   | ICP trust threshold (quantitative)?           | inlier_ratio ≥ 0.70 AND rmse ≤ 0.05 m (both ROS params)                              |
| 2     | Failure Analyst   | Behavior on confidence loss mid-approach?     | Sofort cmd_vel=0 + abort + Operator-Notification                                      |
| 2     | Boundary Keeper   | Refresh trigger UX (Variante γ secondary)?    | Auto beim ersten Undock nach 7d wenn confidence > 95%                                  |
| 3     | Seed Closer       | E-Stop mid-FineDock behavior?                 | Sofort cmd_vel=0, BT hold-state; explicit Operator re-trigger needed                  |
| 3     | Seed Closer       | Initial dock_scan.pcd capture trigger?        | Erweitert calibrate_imu_yaw_node — kein extra Operator-Step                            |
| 3     | Seed Closer       | Welche DockRobot calls migrieren?             | Alle 4 (CriticalBattery + RainDock + BatteryDock + FailedCoverage)                    |

---

*Phase: 02-lidar-dock-pose-estimation*
*Spec created: 2026-04-29*
*Next step: /gsd-discuss-phase 2 — implementation decisions (package layout, scan_match algorithm details, GUI surface specifics, test scope)*
