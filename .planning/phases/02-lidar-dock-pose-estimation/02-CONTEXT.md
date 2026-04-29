# Phase 2: lidar-dock-pose-estimation - Context

**Gathered:** 2026-04-29
**Status:** Ready for planning

<domain>
## Phase Boundary

Replace the open-loop GPS-only docking final approach with closed-loop LiDAR-fine ICP correction over the last 1.5 m, plus shared-infrastructure Smart Undock (rear free-space probe + post-undock RTK validation). Capture once, refresh on high-confidence undocks. Achieves ≤ 2 cm lateral / ≤ 1° yaw at first contact.

</domain>

<spec_lock>
## Requirements (locked via SPEC.md)

**13 requirements are locked.** See `02-SPEC.md` for full requirements, boundaries, and acceptance criteria.

Downstream agents MUST read `02-SPEC.md` before planning or implementing. Requirements are not duplicated here.

**In scope (from SPEC.md):**
- 6 new components: `dock_scan_capture` (extension to calibrate_imu_yaw_node), `dock_scan_match` (new ROS2 node), `RecordDockApproachPose` (new BT node), `ApproachDock` (new BT node), `FineDock` (new BT node), GUI Dock-card extension
- 2 additional BT nodes from #43: `PreUndockClearanceCheck`, `PostUndockRtkValidation`
- Migration of all 4 `DockRobot` BT calls to the new subtree
- Extension of `dock_yaw_to_set_pose` priority cascade to include `/dock_match/pose`
- Auto-refresh of `dock_scan.pcd` after 7 days when confidence ≥ 95%
- 2 new YAML files: `dock_scan_meta.yaml`, `dock_approach.yaml` (alongside existing `dock_calibration.yaml`)
- 1 new PCD file: `dock_scan.pcd`
- Custom msg type for `dock_match.confidence` (under `mowgli_interfaces`)
- New ROS2 package `mowgli_lidar_docking`
- GUI Dock-card display: ICP confidence (inlier_ratio + rmse), dock_scan.pcd age in days, last-fine-dock lateral error in cm

**Out of scope (from SPEC.md):**
- Replacing `opennav_docking` framework entirely
- Custom Nav2 ChargingDock plugin (`SimpleChargingDockLidar`) — separate future work
- Parametric LiDAR feature-detection (RANSAC for V-funnel) — raw scan-vs-snapshot ICP only
- Multi-dock support — single `home_dock` only
- Dock recognition from arbitrary LiDAR scans (must capture first)
- 3D point cloud handling — LD19 is 2D, all ICP runs on 2D
- LiDAR feature for navigation in mow areas (collision_monitor only)
- Encryption / signing of `dock_scan.pcd`
- E-Stop auto-restart — Operator must explicitly re-trigger COMMAND_HOME
- Pre-mow vs post-mow differentiation — every `DockRobot` trigger uses the new subtree

</spec_lock>

<decisions>
## Implementation Decisions

### Package Layout & ROS2 interface

- **D-01:** New ROS2 package `mowgli_lidar_docking` for `dock_scan_match` node + `dock_scan_capture` library + helper utilities. Consistent with Phase 1 pattern (mowgli_coverage_planner separate from mowgli_map). Dependencies: `kinematic_icp` solver (in-tree), `sensor_msgs`, `mowgli_interfaces`, `pcl_ros`.
- **D-02:** New BT nodes (`RecordDockApproachPose`, `ApproachDock`, `FineDock`, `PreUndockClearanceCheck`, `PostUndockRtkValidation`) live in `mowgli_behavior` (Phase 1 Plan 01-08 pattern). `mowgli_lidar_docking` exposes a public lib (`dock_match_subscriber`, `dock_approach_loader`) that BT nodes link against. main_tree.xml registers a single BT plugin set from `mowgli_behavior`.
- **D-03:** New custom msg `mowgli_interfaces::msg::DockMatchConfidence` with fields `std_msgs/Header header`, `float32 inlier_ratio`, `float32 rmse_m`, `bool trusted`. Generates firmware/Go/TS bindings via standard codegen (`sync_ros_lib.py`, `generate_go_msgs.sh`, `generate_ts_types.sh`). NOT a JSON-in-string hack and NOT a private internal msg — the GUI needs it.
- **D-04:** `dock_approach.yaml` uses flat key=value format (Phase 1 D-05 pattern, no yaml-cpp dependency for runtime read paths). Schema:
  ```
  dock_approach_x: <metres>
  dock_approach_y: <metres>
  dock_approach_yaw_to_dock_rad: <radians>
  source: <lidar|tf>
  captured_at: <ISO-8601>
  ```
  Parser identical to existing `load_dock_calibration_file()` in `hardware_bridge_node.cpp:130-143`.

### dock_scan_match algorithm details

- **D-05:** ICP cropping box default `±3 m around expected dock pose` (config param `crop_radius_m: 3.0`). Only `/scan_kicp` points within this box are matched against `dock_scan.pcd`. Faster + robust against false matches with foliage/furniture beyond dock.
- **D-06:** PCD format is **PCL ASCII** (`pcl::io::savePCDFile` with binary=false). Human-inspectable in text editor + `pcl_viewer`. ~10 KB for 1 LD19 revolution. Operator can manually tweak in field if needed.
- **D-07:** ICP match cadence is **10 Hz** (rclcpp wall_timer 100 ms period). Above SPEC R-2 minimum of 5 Hz; 30 ms ICP wall-time on cropped scan leaves headroom. For FineDock crawl at 5 cm/s = 5 mm per match cycle = sub-pixel control.
- **D-08:** ICP defaults `max_correspondence_distance: 0.30 m`, `max_iterations: 20`. Both as ROS2 params (`min_inlier_ratio`, `max_rmse_m` already in SPEC R-3 are also params). 30 cm correspondence covers realistic single-cycle drift; 20 iters terminate in <30 ms on Pi5.

### GUI surface (operator-facing)

- **D-09:** Extend the **existing Dock-card** in the GUI dashboard (NOT a new card). Adds a LiDAR-confidence row + Recapture-button below the existing Charging indicator. Operator finds all dock-relevant info in one card.
- **D-10:** Confidence visualization is **status-badge + numeric pair**: green badge (trusted), yellow (border zone, e.g. inlier 60-70% or rmse 5-10 cm), red (untrusted). Underneath: `Inlier 78% / RMSE 3.2 cm`. Operator gets at-a-glance go/no-go + drill-in numbers when needed.
- **D-11:** "Recapture dock scan" button lives **in the Dock-card with a confirm-modal**. Click opens modal: "Capture wird den aktuellen LiDAR-Snapshot als neuen dock_scan.pcd speichern. Roboter sollte sicher auf dem Dock sitzen. Fortfahren?" Operator confirms. Prevents accidental re-captures from misclicks.
- **D-12:** GUI subscribes to `/dock_match/pose` and `/dock_match/confidence` via the **existing Go-relayed `topicMap` pattern** (`gui/pkg/providers/ros.go::topicMap`, ~2-line extension). DockMatchConfidence.msg auto-generated TS bindings via `generate_ts_types.sh`. **Reconciliation 2026-04-29 evening:** original wording "direct WebSocket via foxglove_bridge" was a Phase 1 D-13 carry-over but Phase 1's D-13 itself referenced rosbridge_server, which has been replaced in this fork by foxglove_bridge. There is no installed rosbridge in the deployed image and `@foxglove/ws-protocol` is not loaded in the browser. EVERY existing live-data hook (`useImuYaw`, `useMagYaw`, `useDockingSensor`, `useCoveragePlan`, etc.) routes through `topicMap` — Phase 2 follows the same pattern for consistency and zero new dependencies.

- **D-17 (added 2026-04-29 evening from RESEARCH §Open Questions):** `dock_scan_meta.yaml` uses **flat key=value** format (same as `dock_approach.yaml` D-04 and `dock_calibration.yaml`). Schema:
  ```
  dock_scan_pcd_path: /ros2_ws/maps/dock_scan.pcd
  dock_pose_x: <metres>
  dock_pose_y: <metres>
  dock_pose_yaw_rad: <radians>
  sensor_extrinsic_x: <metres>
  sensor_extrinsic_y: <metres>
  sensor_extrinsic_yaw_rad: <radians>
  point_count: <integer>
  captured_at: <ISO-8601>
  fix_type: <string, e.g. RTK_FIXED>
  ```
  Single shared C++ parser in `mowgli_geometry` (extension of existing key=value scanner) used by `dock_calibration_loader`, `dock_approach_loader`, `dock_scan_meta_loader`. Operator can read all three files with the same mental model.

### Test scope & Verification strategy

- **D-13:** **Unit test coverage:** all unit-testable components get gtest. `dock_scan_io` (PCD round-trip), `scan_match wrapper` (mocked solver), `BT nodes` (mocked Pose subscriber, mocked TF buffer, mocked /scan_kicp), `yaml parsers` (round-trip + corruption rejection). ~12-15 unit tests across the new package + extensions to `mowgli_behavior` test suite.
- **D-14:** **Sim test:** new `synthetic_scan_kicp_publisher.py` in `mowgli_simulation` publishes `/scan_kicp` matching the sim-dock geometry. `dock_scan_capture` runs normally and writes `fake_dock_scan.pcd` into `mowgli_sim_maps` volume. dock_scan_match + FineDock run normally. End-to-end FineDock approach is verifiable in headless sim using `sim_full_system.launch.py`.
- **D-15:** **Hardware acceptance** (5-of-5 acceptance criterion): extend `mow_session_monitor.py` to log `dock_match.pose`, `dock_match.confidence`, `lateral_error_at_contact` per cycle. Operator runs 5 mow-cycles, JSONL files committed (per memory: "commit notable sessions"). `02-SUMMARY.md` includes operator-checklist with pass/fail per run.
- **D-16:** **ICP solver mocking:** introduce wrapper interface `IDockMatcher` (abstract base in `mowgli_lidar_docking/include/mowgli_lidar_docking/idock_matcher.hpp`). Production: `KinematicIcpDockMatcher : IDockMatcher`. Tests: `MockDockMatcher : IDockMatcher` returns fixed Pose + confidence. Standard dependency injection, no `#ifdef UNIT_TESTING` branches.

### Claude's Discretion

- P-controller gain tuning for FineDock lateral_y and yaw control loops — start with empirical defaults `k_lateral: 1.5`, `k_yaw: 2.0`, override via ROS params; refine on Pi5 hardware testing
- Exact UI layout / colour palette for the Dock-card extension (within design system defaults)
- mow_session_monitor.py JSONL schema additions (new field names, ordering)
- Whether to ship a `02-VERIFICATION.md` template alongside the operator checklist
- DDS/QoS profiles for `/dock_match/pose` and `/dock_match/confidence` (sensible default: SensorDataQoS for confidence; ReliableQoS depth 1 for pose, mirroring the EKF-feeder pattern in `dock_yaw_to_set_pose.py`)

### Folded Todos

(none reviewed in this discussion)

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Phase 2 spec
- `.planning/phases/02-lidar-dock-pose-estimation/02-SPEC.md` — Locked requirements (13), boundaries, acceptance criteria — MUST read before planning
- `.planning/ROADMAP.md` § "Phase 2 — LiDAR-based dock pose estimation" — phase goal + cross-ref to #43, #75

### Project invariants (CLAUDE.md)
- `CLAUDE.md` § "Architecture Invariants" — Invariants #1 (single localizer), #5 (costmap obstacles disabled in coverage), #10 (no UndockRobot reintroduction), #13 (cmd_vel routing via twist_mux priority 15). Phase 2 must NOT violate any.
- `CLAUDE.md` § "What NOT to Do" — Specifically the lines about K-ICP feedback loops and not feeding scan-derived pose back into robot_localization as TF.

### Reference implementations (read before building)
- `ros2/src/mowgli_localization/scripts/dock_yaw_to_set_pose.py` — Existing seeder cascade. Phase 2 extends with `/dock_match/pose` priority above file.
- `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp` § lines 90-143 — `load_dock_calibration_file()` parser pattern. Reuse for `load_dock_approach_file()`.
- `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp` — BTContext struct extension point for new fields (e.g., `dock_pose_suspect`, `last_dock_match_confidence`).
- `ros2/src/mowgli_behavior/src/calibration_nodes.cpp` — Pattern for BT nodes that read TF + write yaml. `RecordDockApproachPose` will mirror `RecordUndockStart` style.
- `ros2/src/mowgli_localization/scripts/calibrate_imu_yaw_node.py` — Extension point for `dock_scan_capture` (runs at successful charge after dock-yaw calibration).
- `ros2/src/mowgli_localization/scripts/kinematic_icp_scan_frame_relay.py` — Source of `/scan_kicp` (already publishes at correct sensor frame on parallel TF tree).
- `ros2/src/kinematic_icp/` — In-tree PRBonn solver; we instantiate `KinematicICP` directly, not the wrapper node.
- `ros2/src/mowgli_behavior/trees/main_tree.xml` § lines 171, 332, 378, 445 — 4 `DockRobot` call sites to migrate.

### Phase 1 patterns to follow
- `.planning/phases/01-coverage-planner-rewrite/01-CONTEXT.md` — Decision style, separation between BT and library packages, atomic write pattern, gtest-per-source pattern.
- `.planning/STATE.md` § Decision log entries from Phase 1 Plans 01-02 / 01-05 — atomic_write helper, key=value parser, gtest patterns, in-process service stub for tests.

### Action / message interfaces
- `ros2/src/mowgli_interfaces/msg/HighLevelStatus.msg` — Style template for the new `DockMatchConfidence.msg` (constants block + field block).
- `ros2/src/mowgli_interfaces/srv/HighLevelControl.srv` — Existing command set; FineDock failure modes propagate to `HighLevelStatus.sub_state_name`.

### Configuration
- `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` — `dock_pose_x/y/yaw` operator-facing config (now overridden by `dock_calibration.yaml` since #74).
- `ros2/src/mowgli_bringup/config/nav2_params.yaml` § `collision_monitor` — Phase 2 does NOT modify; `PreUndockClearanceCheck` queries `/scan_kicp` directly, not via collision_monitor.

### GUI
- `gui/web/src/types/ros.ts` — Auto-generated TS types; will gain `DockMatchConfidence` after `generate_ts_types.sh` re-run.
- `gui/web/src/pages/MapPage.tsx` (or equivalent dock-card location) — Existing Dock-card site to extend.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets

- **`/scan_kicp` topic + parallel TF tree** (`mowgli_localization/kinematic_icp_scan_frame_relay.py`): Phase 2 substrate — same scan we already run K-ICP against, same sensor extrinsic frame.
- **`load_dock_calibration_file()` C++ parser** (`hardware_bridge_node.cpp:130-143`): Pattern for `dock_approach.yaml` parser. Reuse the no-yaml-cpp scanner verbatim for the new fields.
- **`RecordUndockStart` / `CalibrateHeadingFromUndock` BT nodes** (`mowgli_behavior/calibration_nodes.cpp`): Just refactored to use TF buffer in #73. `RecordDockApproachPose` is a sibling — same TF-lookup + yaml-write pattern.
- **`dock_yaw_to_set_pose.py` priority cascade**: Already supports a fallback hierarchy. Phase 2 extends to `/dock_match/pose > file > heading`.
- **Twist mux priority 15 for `/cmd_vel_docking`**: Already configured in `nav2_params.yaml` per Architecture Invariant #13. FineDock publishes there, no new mux config needed.
- **Codegen pipelines** (`sync_ros_lib.py`, `generate_go_msgs.sh`, `generate_ts_types.sh`): Standard hooks for the new `DockMatchConfidence.msg`.
- **`mow_session_monitor.py`**: Already logs Pose, EKF state, BT state — extension to log `/dock_match/*` is additive (new column in JSONL header).

### Established Patterns

- **Single source of truth pattern (Phase 1 D-05, also #74)**: every dock-related value has exactly one writer + many readers. dock_approach.yaml writer = `RecordDockApproachPose`; readers = `ApproachDock` BT, GUI (via Go bridge of yaml file).
- **Atomic write via tempfile + rename(2)**: Phase 1 Plan 01-02 atomic_write helper. Reuse for `dock_scan.pcd` and `dock_scan_meta.yaml` writes.
- **BT delegates filesystem I/O via service** (Phase 1 Plan 01-08): BT nodes don't write yaml directly; they call a service in the library node. Apply: `RecordDockApproachPose` calls a `WriteDockApproach.srv` in `dock_scan_match` node (or its own writer thread).
- **Key=value YAML format for runtime-read configs** (Phase 1 D-05): No yaml-cpp dependency for hot paths. dock_approach.yaml follows this convention.
- **gtest with mocked DDS via in-process service stubs** (Phase 1 Plan 01-11): `WriteCheckpointAreaIndexTest` pattern — protected lift of internal state, in-process server. Apply to `RecordDockApproachPose` test.

### Integration Points

- **`main_tree.xml` Sequence migration**: 4 sites (CriticalBatteryDock, RainDockAndResume, BatteryDockAndResume, FailedCoverageDock) replace `<DockRobot dock_id="home_dock" .../>` with `<Sequence><ApproachDock/><FineDock/></Sequence>` (or a reusable `<LidarDockSubtree/>` plugin).
- **`UndockSequence` BT extension**: insert `PreUndockClearanceCheck` before `BackUp`; insert `PostUndockRtkValidation` after `CalibrateHeadingFromUndock`; insert `RecordDockApproachPose` at very end of UndockSequence (writes the captured pose for next docking).
- **`navigation.launch.py`**: spawn `dock_scan_match` node alongside existing nodes (after `kinematic_icp_group`, before `wait_for_map_odom_tf`). Param `dock_calibration_path`, `dock_scan_path`, `dock_scan_meta_path`, `min_inlier_ratio`, `max_rmse_m`, `crop_radius_m`, `max_correspondence_distance`, `max_iterations`.
- **GUI Dock-card extension**: TypeScript hook `useDockMatch()` subscribes to `/dock_match/confidence` + `/dock_match/pose`. Renders in existing Dock-card. Mirror useImuYaw / useMagYaw hooks already in `gui/web/src/hooks/`.

</code_context>

<specifics>
## Specific Ideas

- Operator's words on architecture: "beim Undocking die exakte position per LiDAR ermittelt wird. beim Docking soll dann per LiDAR wieder genau an die Position gefahren werden. Vorher aber per RTK Fix an die Stelle fahren, an der das Docking beginnt." This is the canonical formulation for downstream agents — quote it in plan docs.
- `dock_scan.pcd` is captured at the existing dock-yaw calibration GUI button — operator does NOT learn a new workflow, the calibration just gets richer.
- 5-of-5 acceptance is a hardware test, not a CI test — Pi5 outdoors with RTK; same pattern as Phase 1 SPEC AC-13 hardware smoke (operator-gated, results in SUMMARY.md).
- All 4 `DockRobot` calls migrate, no halfway split. Operators get one consistent docking behaviour everywhere.
- Pre-undock LiDAR free-space check returns FAILURE on rear obstacle — UndockSequence aborts cleanly, the robot does NOT drive into a person standing behind the dock.

</specifics>

<deferred>
## Deferred Ideas

- **Custom Nav2 ChargingDock plugin (`SimpleChargingDockLidar`)** — replacing opennav_docking framework entirely. Out-of-scope per SPEC.md. Future phase if FineDock approach proves robust enough to justify a Nav2-native plugin.
- **3D dock geometry support** — would need a 3D LiDAR (current LD19 is 2D). Not on hardware roadmap.
- **Multi-dock support** — multiple home_docks per garden. Future phase if mow gardens with multiple dock points become a use case.
- **Parametric LiDAR feature-detection** (RANSAC for V-funnel) — interesting research direction, not needed if scan-vs-snapshot ICP is good enough.
- **dock_scan auto-recapture on operator-defined timer** — alternative to 7-day-on-undock-confidence. Possibly desired by operators with fast-changing dock environments. Out-of-scope this phase.
- **Pre-mow vs post-mow differentiation** — different dock approaches for "robot pushed off dock" vs "robot returning from mow". Phase 2 uses universal FineDock for both; differentiation is a future tuning option.
- **Pre-existing planner test failures triage** (separate task) — 4 tests in coverage_planner test suite fail due to pre-existing bugs surfaced by the Phase 2 compile fixes. Tracked outside this phase.

</deferred>

---

*Phase: 02-lidar-dock-pose-estimation*
*Context gathered: 2026-04-29*
