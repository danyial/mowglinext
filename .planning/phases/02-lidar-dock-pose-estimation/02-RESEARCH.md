# Phase 2: LiDAR-based dock pose estimation — Research

**Researched:** 2026-04-29
**Domain:** LiDAR scan-vs-snapshot ICP for closed-loop docking on ROS2 Kilted (C++17, BehaviorTree.CPP v4, in-tree PRBonn Kinematic-ICP, foxglove_bridge GUI bridge)
**Confidence:** HIGH

## Summary

Phase 2 wires the existing `/scan_kicp` substrate (LD19 2D LiDAR on the parallel TF tree, already feeding K-ICP) into a second consumer: a `dock_scan_match` node that runs `kinematic_icp::pipeline::KinematicICP` against a one-time `dock_scan.pcd` snapshot whenever the robot is near the dock. The match output `(SE3 pose, inlier_ratio, rmse)` is published as `/dock_match/pose` + `/dock_match/confidence` and consumed by `FineDock` (final-1.5 m P-controller crawl), `RecordDockApproachPose` (writes `dock_approach.yaml`), and `dock_yaw_to_set_pose` (extends seed cascade). The Kinematic-ICP solver C++ class is fully public, header-only against three direct includes, and the canonical RegisterFrame call site pattern is `LidarOdometryServer.cpp` upstream — we mirror it.

The 16 D-XX decisions in CONTEXT.md are locked. This research fills implementation gaps within them: the exact KinematicICP API signature, how to convert a 2D `sensor_msgs/LaserScan` into the `vector<Eigen::Vector3d>` frame the solver expects, how to compute `inlier_ratio` and `rmse` (which the solver doesn't expose directly — we compute them post-hoc from the local map vs registered frame), how `RegisterFrame` returns nearest-correspondence pairs that we leverage for confidence, where to spawn the new node in `navigation.launch.py`, the codegen ergonomics for the new `DockMatchConfidence.msg`, the GUI hook pattern (Go-relayed via `topicMap`, NOT direct WebSocket — D-12 needs that nuance reconciled), and the validation strategy aligned to the SPEC's hardware acceptance gate.

**Primary recommendation:** Use `kinematic_icp::pipeline::KinematicICP` directly via two header includes; wrap it behind `IDockMatcher` (D-16) so unit tests inject a mock; convert 2D LaserScan → `vector<Eigen::Vector3d>` via `laser_geometry::LaserProjection::projectLaser` (already used by `kinematic_icp_online_node` upstream); compute confidence as `inlier_ratio = matched_pairs / frame_size` and `rmse = sqrt(mean(squared_residuals))` from the post-RegisterFrame frame-vs-map nearest-neighbour pairs; gate downstream consumers on `confidence.trusted` (D-03 field). For the GUI: **D-12 needs a small clarification** — there is no rosbridge_server in this fork (replaced by foxglove_bridge), so "direct WebSocket" most pragmatically means using the existing `topicMap` Go-relay pattern (matches `useMagYaw`/`useDockingSensor`) with auto-generated TS bindings — adding 2 lines to `gui/pkg/providers/ros.go::topicMap`.

## User Constraints (from CONTEXT.md)

### Locked Decisions

**Package Layout & ROS2 interface**

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

**dock_scan_match algorithm details**

- **D-05:** ICP cropping box default ±3 m around expected dock pose (config param `crop_radius_m: 3.0`).
- **D-06:** PCD format is **PCL ASCII** (`pcl::io::savePCDFile` with binary=false). ~10 KB for 1 LD19 revolution.
- **D-07:** ICP match cadence is **10 Hz** (rclcpp wall_timer 100 ms period). Above SPEC R-2 minimum of 5 Hz.
- **D-08:** ICP defaults `max_correspondence_distance: 0.30 m`, `max_iterations: 20`. Both as ROS2 params (`min_inlier_ratio`, `max_rmse_m` already in SPEC R-3 are also params).

**GUI surface (operator-facing)**

- **D-09:** Extend the existing Dock-card in the GUI dashboard (NOT a new card).
- **D-10:** Confidence visualization is status-badge + numeric pair: green badge (trusted), yellow (border zone), red (untrusted). Underneath: `Inlier 78% / RMSE 3.2 cm`.
- **D-11:** "Recapture dock scan" button lives in the Dock-card with a confirm-modal.
- **D-12:** GUI subscribes to `/dock_match/pose` and `/dock_match/confidence` directly via WebSocket through foxglove_bridge (Phase 1 D-13 pattern). DockMatchConfidence.msg auto-generated TS bindings via `generate_ts_types.sh`. NO Go-backend relay — fastest path, no extra Go code. **NOTE FROM RESEARCH §5:** the existing fork has no rosbridge_server; "direct WebSocket" is interpreted as the foxglove websocket protocol via @foxglove/ws-protocol if a real direct path is required; pragmatic interpretation: extend `topicMap` in `gui/pkg/providers/ros.go` (2 lines), matches every other live-data hook in the GUI.

**Test scope & Verification strategy**

- **D-13:** **Unit test coverage:** all unit-testable components get gtest. ~12-15 unit tests across the new package + extensions to `mowgli_behavior` test suite.
- **D-14:** **Sim test:** new `synthetic_scan_kicp_publisher.py` in `mowgli_simulation` publishes `/scan_kicp` matching the sim-dock geometry.
- **D-15:** **Hardware acceptance** (5-of-5 acceptance criterion): extend `mow_session_monitor.py` to log `dock_match.pose`, `dock_match.confidence`, `lateral_error_at_contact`.
- **D-16:** **ICP solver mocking:** introduce wrapper interface `IDockMatcher` (abstract base in `mowgli_lidar_docking/include/mowgli_lidar_docking/idock_matcher.hpp`). Production: `KinematicIcpDockMatcher : IDockMatcher`. Tests: `MockDockMatcher : IDockMatcher`.

### Claude's Discretion (from CONTEXT.md)

- P-controller gain tuning for FineDock — start `k_lateral: 1.5`, `k_yaw: 2.0`, override via ROS params; refine on Pi5
- Exact UI layout / colour palette for the Dock-card extension
- mow_session_monitor.py JSONL schema additions (new field names, ordering)
- Whether to ship a `02-VERIFICATION.md` template alongside the operator checklist
- DDS/QoS profiles for `/dock_match/pose` and `/dock_match/confidence`

### Deferred Ideas (OUT OF SCOPE)

- Custom Nav2 ChargingDock plugin (`SimpleChargingDockLidar`)
- 3D dock geometry support
- Multi-dock support
- Parametric LiDAR feature-detection (RANSAC for V-funnel)
- dock_scan auto-recapture on operator-defined timer
- Pre-mow vs post-mow differentiation
- Pre-existing planner test failures triage

## Project Constraints (from CLAUDE.md)

Phase 2 plan must comply with these CLAUDE.md directives:

- **Architecture Invariant #1 (single localizer):** `dock_scan_match` output may NEVER be published to `/tf`. It enters robot_localization only via the `dock_yaw_to_set_pose` `/set_pose` seed (Requirement 4). The match pose is treated as a measurement at boot/fine-dock seed time, never as a pose source for K-ICP or for an EKF input topic. `[VERIFIED: CLAUDE.md "Do NOT publish a map→odom TF from Kinematic-ICP, Nav2, or any other node"]`
- **Architecture Invariant #5 (costmap obstacles disabled in coverage mode):** FineDock runs *post*-coverage in the dock approach phase; collision_monitor is the only safety net. No new costmap layers. `[VERIFIED: CLAUDE.md]`
- **Architecture Invariant #10 (no UndockRobot reintroduction):** FineDock is a new BT node, NOT a Nav2 dock plugin. opennav_docking's UndockRobot stays deprecated. Pre-undock continues to use Nav2 `BackUp`. `[VERIFIED: SPEC §Background bullet 7]`
- **Architecture Invariant #13 (cmd_vel via twist_mux priority 15):** FineDock publishes to `/cmd_vel_docking`, no new mux source. `[VERIFIED: twist_mux.yaml lines 15-18]`
- **Architecture Invariant #15 (footprint sync):** Phase 2 does not touch footprint geometry — out of scope.
- **Cyclone DDS only.** All publishers must respect this. `[VERIFIED: CLAUDE.md]`
- **No yaml-cpp.** dock_approach.yaml uses key=value flat parser — already locked in D-04. dock_scan_meta.yaml is human-written-once (calibration); a minimal yaml.safe_load Python read in calibrate_imu_yaw_node, but the C++ runtime read paths in dock_scan_match avoid yaml-cpp by using the same `parse_yaml_double` scanner. `[VERIFIED: hardware_bridge_node.cpp:99]`
- **Pi5 hardware test before PR.** Per memory `workflow_pi5_test_then_pr.md` and project memory `feedback_background_pipeline_deploy.md`. SPEC AC-13 5-of-5 hardware acceptance is operator-gated. `[CITED: ~/.claude/projects/.../memory/workflow_pi5_test_then_pr.md]`
- **English in PRs/issues/code; German in chat replies.** Plan docs are English. `[CITED: memory feedback_language.md]`

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| R-1 | dock_scan.pcd capture extends calibrate_imu_yaw_node | §1 (extension point), §2 (PCL save), §3 (PCD format) |
| R-2 | dock_scan_match node publishes ≥ 5 Hz | §1 (Kinematic-ICP API), §6 (cadence) |
| R-3 | ICP trust threshold inlier ≥ 0.70 AND rmse ≤ 0.05 | §1 (confidence computation) |
| R-4 | dock_yaw_to_set_pose extends cascade with /dock_match/pose | §7 (seeder extension pattern) |
| R-5 | RecordDockApproachPose BT writes dock_approach.yaml at undock end | §4 (BT TF + write pattern), §3 (atomic write) |
| R-6 | ApproachDock BT — NavigateToPose to dock_approach.yaml | §4 (NavigateToPose action client pattern) |
| R-7 | FineDock closed-loop crawls last 1.5 m | §1 (registration cycle), §8 (FineDock state machine) |
| R-8 | FineDock aborts on confidence loss | §8 (state machine, confidence gate) |
| R-9 | E-Stop hold + no auto-restart after FineDock | §4 (BT pattern), §8 |
| R-10 | All 4 DockRobot calls migrate to ApproachDock + FineDock | §9 (main_tree.xml sites verified) |
| R-11 | dock_scan auto-refresh after 7 days at confidence ≥ 0.95 | §3 (atomic-write helper for PCD) |
| R-12 | PreUndockClearanceCheck — rear-sector clearance from /scan_kicp | §10 (LaserScan rear-sector parsing) |
| R-13 | PostUndockRtkValidation — odom vs GPS error gate | §11 (existing GPS+TF available in BTContext) |

## Architectural Responsibility Map

Phase 2 spans the existing tier model (no new tiers introduced):

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| LiDAR scan-vs-snapshot ICP | ROS2 Node (`mowgli_lidar_docking::dock_scan_match`) | mowgli_localization (substrate `/scan_kicp`) | Stateful pipeline tied to TF lookups; not a BT decision |
| Closed-loop dock crawl P-controller | BT node (`mowgli_behavior::FineDock`) | twist_mux (priority 15) | Lifecycle bound to BT goal; uses existing teleop-style direct cmd_vel |
| dock_scan.pcd capture | rclpy node (`calibrate_imu_yaw_node` extension) | mowgli_lidar_docking lib (PCD save) | Gated by GUI-triggered calibration service; reuses operator's existing calibration drive |
| dock_approach.yaml writer | BT node (`RecordDockApproachPose`) | mowgli_lidar_docking lib (atomic_write helper) | Lifecycle bound to undock-sequence completion |
| dock_approach.yaml reader | BT node (`ApproachDock`) + `mowgli_hardware` startup parser (optional fallback) | none | One-shot read at goal start; no live updates |
| EKF seed from LiDAR match | Python node (`dock_yaw_to_set_pose`) | none | Existing seeder; just adds top-priority cascade entry |
| GUI confidence display | TS hook (`useDockMatch`) + Dock-card React component | Go provider relay (`topicMap`) | Read-only telemetry; matches every other live data hook |
| Pre-undock clearance check | BT node (`PreUndockClearanceCheck`) | none | Direct `/scan_kicp` subscriber inside BT (matches LaserScan reads in e2e_test.py) |
| Post-undock RTK validation | BT node (`PostUndockRtkValidation`) | none | Pure consumer of BTContext::gps_x/y + TF |
| Auto-refresh policy (7 days, ≥ 0.95 inlier) | BT node (`RecordDockApproachPose`) | mowgli_lidar_docking lib (PCD save + meta read) | One-shot at undock end; no separate scheduler needed |

**Cross-tier integrity rules to preserve in plan tasks:**
1. `dock_scan_match` MUST NOT publish any TF and MUST NOT subscribe to robot_localization-fused topics (e.g., `/odometry/filtered_map` or `map → base_footprint` TF) for its scan-input pipeline. The match operates ON the parallel `lidar_link_wheels` frame; the result is *transformed* into the map frame using the static dock-pose anchor from `dock_calibration.yaml`. This preserves invariant #1: no LiDAR-fed loop into the EKF.
2. The match pose may be expressed in the map frame for downstream consumers (FineDock control, `/set_pose` seed) — but the *registration* itself is dock-frame-relative.
3. FineDock publishes `/cmd_vel_docking` only. No direct firmware writes, no blade commands.

## Standard Stack

### Core (additions for Phase 2)

| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `kinematic_icp` (in-tree submodule) | upstream main (PRBonn) | LiDAR scan registration | Already wired for K-ICP; same library + headers; SPEC R-2 explicitly mandates it |
| `kiss_icp` (transitive of kinematic_icp) | upstream main | VoxelHashMap, Preprocessor | Required by KinematicICP::Config; transitive |
| `Sophus` | apt `libsophus-dev` (or transitive) | SE(3) types | Required by KinematicICP RegisterFrame signature; already in image (kinematic_icp depends on it) |
| `Eigen3` | apt `libeigen3-dev` | Linear algebra | Already used by mowgli_geometry; transitive of every ROS2 package |
| `laser_geometry` | apt `ros-kilted-laser-geometry` | LaserScan → PointCloud2 projection | Same library upstream `kinematic_icp_online_node` uses for `use_2d_lidar` mode `[VERIFIED: PRBonn online_node.cpp]` |
| `pcl_ros` + `pcl_conversions` | apt `ros-kilted-pcl-ros`, `ros-kilted-pcl-conversions` | PCL ROS2 integration | Standard for PCD I/O in ROS2; `[CITED: pcl_ros docs]` |
| `libpcl-dev` | apt | PCD save/load (`pcl::io::savePCDFile` / `loadPCDFile`) | D-06 mandates PCL ASCII; `[VERIFIED: PCL docs]` |
| `tf2_ros`, `tf2_geometry_msgs` | already in stack | TF lookups for dock pose, lever-arm extrinsic | Used everywhere in mowgli_localization |

**Dockerfile additions required** (currently missing per `ros2/Dockerfile`):

```dockerfile
ros-kilted-laser-geometry \
ros-kilted-pcl-conversions \
ros-kilted-pcl-ros \
libpcl-dev \
```

`[VERIFIED: ros2/Dockerfile lines 10-71 do not list any of these]`

`Sophus` is **not** in the apt list either, but it is pulled in transitively as a `<depend>` of the kinematic_icp ros wrapper `[VERIFIED: PRBonn ros/package.xml]`. New `mowgli_lidar_docking::package.xml` MUST list `<depend>sophus</depend>` explicitly so `find_package(Sophus REQUIRED)` resolves.

### Supporting (already in stack)

| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `BehaviorTree.CPP v4` | `ros-kilted-behaviortree-cpp` | BT nodes for FineDock, ApproachDock, etc. | Every new BT node (D-02) |
| `tf2_ros::Buffer` | already in mowgli_behavior | TF lookups in BT nodes | RecordDockApproachPose, FineDock |
| `nav2_msgs::action::NavigateToPose` | `ros-kilted-nav2-msgs` | ApproachDock dispatches via this | `[VERIFIED: PlanCoverageGoal dispatch pattern in FollowCoveragePlan]` |
| `mowgli_geometry::detail::atomic_write` | `mowgli_geometry/atomic_write.hpp` | Atomic temp+rename for dock_approach.yaml + dock_scan.pcd | Phase 1 D-05 atomic-write helper, **already exists** `[VERIFIED: ls of include dir]` |

### Alternatives Considered (rejected per SPEC §Constraints)

| Instead of | Could Use | Why Rejected |
|------------|-----------|----------|
| Kinematic-ICP | PCL ICP | Hallucinates symmetric matches on dock V-funnel (no kinematic prior) `[CITED: SPEC §Constraints]` |
| Kinematic-ICP | Open3D ICP | Same issue; also adds large new dep |
| Custom RANSAC for V-funnel | Parametric feature-detection | Out of scope per SPEC; brittle vs scan-vs-snapshot `[CITED: CONTEXT §Out of scope]` |
| In-process spawn (calibrate_imu_yaw + dock_scan_capture as one Python process) | Separate node spawned by launch | The extension-in-place is simpler (D-01: capture is a *library* called from calibrate_imu_yaw_node) AND the operator UX is one button. Pure addition to the existing service handler, no new launch entry. |

**Installation (one-time, before plan execution):**

```bash
# Inside the dev container (or update ros2/Dockerfile):
apt-get install -y ros-kilted-laser-geometry ros-kilted-pcl-conversions ros-kilted-pcl-ros libpcl-dev libsophus-dev
```

**Version verification (planner: re-run before pinning):**

```bash
apt-cache policy ros-kilted-laser-geometry ros-kilted-pcl-conversions ros-kilted-pcl-ros libpcl-dev
dpkg -L ros-kilted-laser-geometry | head -5    # confirm /opt/ros/kilted/include/laser_geometry exists
```

`[ASSUMED: that ros-kilted-* PCL packages exist on Kilted Kaiju]` — Kilted is the active rolling distro and these packages are shipped for every ROS2 distro since Foxy. Confirmed by ros-index for Iron and Jazzy; Kilted parity is conventional.

## Architecture Patterns

### System Architecture Diagram (data flow)

```
                     ┌──────────────────────────────────────────────┐
                     │  GUI: Dock-card (useDockMatch hook)         │
                     │  - inlier_ratio + rmse badge                │
                     │  - Recapture button (modal)                 │
                     └──────────────┬───────────────────────────────┘
                                    │ /api/.../subscribe/dockMatch{Pose,Confidence}
                                    │ (Go provider relay; topicMap entries)
                                    ▼
                ┌─────────────────────────────────────────────────┐
                │  /dock_match/pose  (PoseWithCovarianceStamped)  │
                │  /dock_match/confidence  (DockMatchConfidence)  │
                │  Publisher: dock_scan_match (10 Hz)              │
                └────────────────┬─────────────────────┬───────────┘
                                 │                     │
        ┌────────────────────────┴────┐     ┌──────────┴──────────────────────┐
        │  BT: FineDock (StatefulAction)│   │  dock_yaw_to_set_pose.py        │
        │  - subscribes /dock_match/pose │   │  cascade: lidar > file > heading│
        │  - P-controller lateral + yaw  │   │  one-shot /set_pose seed        │
        │  - publishes /cmd_vel_docking  │   └─────────────────────────────────┘
        │  - aborts on !trusted          │
        │  - stops on is_charging        │
        └────────────────────────────────┘
                                 ▲
                                 │ kicp_->RegisterFrame(frame, ts, lidar_to_base, delta)
                                 │
                ┌────────────────┴───────────────────────────────────────┐
                │  dock_scan_match node (rclcpp, mowgli_lidar_docking)   │
                │                                                          │
                │  Inputs:                                                 │
                │    - /scan_kicp (lidar_link_wheels frame, 10 Hz)        │
                │    - /tf (map → base_footprint_wheels for crop center)  │
                │    - dock_scan.pcd (loaded once at startup)             │
                │    - dock_calibration.yaml (dock anchor pose)           │
                │                                                          │
                │  Pipeline (per scan):                                    │
                │    1. LaserScan → vector<Eigen::Vector3d>               │
                │       via laser_geometry::LaserProjection                │
                │    2. Crop ±crop_radius_m around dock pose              │
                │    3. SetPose(dock_anchor) on first iter; load_map      │
                │    4. RegisterFrame(frame, [], lidar_to_base, identity) │
                │    5. Compute inlier_ratio + rmse from frame ↔ map       │
                │       nearest-neighbour pairs                            │
                │    6. trusted = (inlier ≥ 0.70 AND rmse ≤ 0.05)        │
                │    7. Publish /dock_match/pose + /dock_match/confidence │
                └─────────────────────────────────────────────────────────┘
                                 ▲
                                 │ already published by
                ┌────────────────┴──────────────────┐
                │  kinematic_icp_scan_frame_relay   │
                │  /scan → /scan_kicp + frame swap  │
                └───────────────────────────────────┘
                                 ▲
                                 │ /scan
                ┌────────────────┴──────────────────┐
                │  LDLidar driver (LD19)            │
                └───────────────────────────────────┘

                 ────── Undock-time sub-pipeline ──────

                ┌────────────────────────────────────────────────────┐
                │  BT: UndockSequence (extended)                      │
                │  ┌───────────────────────────────────────────────┐ │
                │  │ PreUndockClearanceCheck                       │ │ NEW (#43)
                │  │  - subscribes /scan_kicp                      │ │
                │  │  - rear sector 180°±30°                       │ │
                │  │  - FAILURE if clearance < 1.7 m               │ │
                │  └─────────────┬─────────────────────────────────┘ │
                │                ▼                                    │
                │  ┌──────────────────────────────────────────────┐  │
                │  │ RecordUndockStart  (existing)                 │  │
                │  └─────────────┬─────────────────────────────────┘  │
                │                ▼                                    │
                │  ┌──────────────────────────────────────────────┐  │
                │  │ Nav2 BackUp (1.5 m, 0.20 m/s)  (existing)    │  │
                │  └─────────────┬─────────────────────────────────┘  │
                │                ▼                                    │
                │  ┌──────────────────────────────────────────────┐  │
                │  │ CalibrateHeadingFromUndock  (existing)        │  │
                │  └─────────────┬─────────────────────────────────┘  │
                │                ▼                                    │
                │  ┌──────────────────────────────────────────────┐  │
                │  │ PostUndockRtkValidation                       │ NEW (#43)
                │  │  - error_xy = ||gps - odom_predicted||        │  │
                │  │  - WARN at >0.5 m, FAIL at >1.5 m             │  │
                │  └─────────────┬─────────────────────────────────┘  │
                │                ▼                                    │
                │  ┌──────────────────────────────────────────────┐  │
                │  │ RecordDockApproachPose                        │ NEW (R-5)
                │  │  - reads /dock_match/pose (preferred) or TF   │  │
                │  │  - writes dock_approach.yaml (atomic)          │  │
                │  │  - if scan age > 7d AND inlier ≥ 0.95         │  │
                │  │    also rewrites dock_scan.pcd (R-11)          │  │
                │  └──────────────────────────────────────────────┘  │
                └────────────────────────────────────────────────────┘

                 ──── Dock-time sub-pipeline (replaces 4× DockRobot) ────

                ┌──────────────────────────────────────────────────────┐
                │  BT: <Sequence name="LidarDock">                     │
                │    <ApproachDock/>  ─── reads dock_approach.yaml,   │ NEW (R-6)
                │                          dispatches NavigateToPose   │
                │    <FineDock/>      ─── 1.5 m crawl, P-controller,  │ NEW (R-7..R-9)
                │                          /cmd_vel_docking, stop on   │
                │                          is_charging or !trusted     │
                │  </Sequence>                                          │
                └──────────────────────────────────────────────────────┘
```

### Recommended Project Structure

```
ros2/src/mowgli_lidar_docking/                # NEW package (D-01)
├── CMakeLists.txt
├── package.xml
├── include/mowgli_lidar_docking/
│   ├── idock_matcher.hpp                    # D-16 abstract base
│   ├── kinematic_icp_dock_matcher.hpp       # production impl
│   ├── dock_scan_io.hpp                     # PCD save/load
│   ├── dock_approach_loader.hpp             # dock_approach.yaml parser (header-only)
│   ├── dock_scan_meta.hpp                   # dock_scan_meta.yaml parser
│   └── confidence_metrics.hpp               # inlier_ratio + rmse computation
├── src/
│   ├── dock_scan_match_node.cpp             # the 10 Hz match node (executable)
│   ├── dock_scan_match_node.hpp
│   ├── kinematic_icp_dock_matcher.cpp       # wraps kinematic_icp::pipeline::KinematicICP
│   ├── dock_scan_io.cpp                     # pcl::io::savePCDFile / loadPCDFile
│   └── confidence_metrics.cpp
├── test/
│   ├── test_dock_scan_io.cpp                # PCD round-trip
│   ├── test_dock_approach_loader.cpp        # yaml parser + corruption rejection
│   ├── test_confidence_metrics.cpp          # inlier_ratio threshold logic
│   ├── test_dock_scan_match_node.cpp        # in-process node test, mock matcher
│   └── CMakeLists.txt
└── config/
    └── dock_scan_match.yaml                 # default params

ros2/src/mowgli_behavior/                    # EXTENDED
├── include/mowgli_behavior/
│   ├── docking_nodes.hpp                    # NEW: ApproachDock, FineDock,
│   │                                          RecordDockApproachPose, PreUndockClearanceCheck,
│   │                                          PostUndockRtkValidation declarations
│   └── bt_context.hpp                       # EXTENDED: add dock_pose_suspect, last_dock_match_*
├── src/
│   └── docking_nodes.cpp                    # NEW: implementations
├── trees/
│   └── main_tree.xml                        # EDITED: 4× DockRobot → <Sequence>ApproachDock+FineDock
└── test/
    └── test_docking_nodes.cpp               # NEW: gtest for new BT nodes

ros2/src/mowgli_interfaces/msg/
└── DockMatchConfidence.msg                  # NEW (D-03)

ros2/src/mowgli_localization/scripts/
├── dock_yaw_to_set_pose.py                  # EDITED: cascade extension (R-4)
└── calibrate_imu_yaw_node.py                # EDITED: add dock_scan capture call (R-1)

ros2/src/mowgli_bringup/launch/
└── navigation.launch.py                     # EDITED: spawn dock_scan_match (R-2)

ros2/scripts/
└── mow_session_monitor.py                   # EDITED: log /dock_match/* + lateral_error_at_contact

gui/pkg/providers/ros.go                     # EDITED: 2 lines in topicMap
gui/pkg/api/mowglinext.go                    # EDITED: cases for dockMatchPose / dockMatchConfidence
gui/web/src/hooks/useDockMatch.ts            # NEW: hook (mirrors useMagYaw)
gui/web/src/pages/.../DockCard.tsx           # EDITED: confidence badge, recapture button
gui/web/src/types/ros.ts                     # AUTO: new DockMatchConfidence type
firmware/stm32/ros_usbnode/src/ros/ros_lib/mowgli_interfaces/  # AUTO via sync_ros_lib.py
gui/pkg/msgs/mowgli/                         # AUTO via generate_go_msgs.sh
```

### Pattern 1: KinematicICP solver instantiation (production)

**What:** Construct `kinematic_icp::pipeline::KinematicICP` with a `Config` derived from ROS params, seed it with the dock pose snapshot, and call `RegisterFrame` per incoming `/scan_kicp`.

**When to use:** In `dock_scan_match_node` once at startup (loads dock_scan.pcd into the local map) and per-scan (registers live scan against the map).

**Source (verified upstream):**

```cpp
// VERIFIED: github.com/PRBonn/kinematic-icp main, cpp/kinematic_icp/pipeline/KinematicICP.hpp
#include <kinematic_icp/pipeline/KinematicICP.hpp>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace mowgli_lidar_docking {

class KinematicIcpDockMatcher : public IDockMatcher {
public:
  KinematicIcpDockMatcher(const kinematic_icp::pipeline::Config& cfg,
                          const std::vector<Eigen::Vector3d>& dock_scan_points,
                          const Sophus::SE3d& dock_anchor_in_map)
    : kicp_(cfg)
  {
    // Seed the LOCAL MAP with the static dock_scan.pcd at the dock anchor.
    // We treat dock_anchor_in_map as the registered "last_pose_" so that
    // the first RegisterFrame call's voxel map is exactly the dock scan
    // expressed in map coordinates. After Setting the pose with SetPose(),
    // we have to manually push the dock points into the voxel map: SetPose()
    // calls local_map_.Clear(), so the voxel map starts empty. We use the
    // public VoxelMap() reference to add the dock points.
    kicp_.SetPose(dock_anchor_in_map);
    kicp_.VoxelMap().AddPoints(dock_scan_points);  // kiss_icp::VoxelHashMap::AddPoints
  }

  MatchResult Match(const std::vector<Eigen::Vector3d>& live_frame,
                    const Sophus::SE3d& lidar_to_base) override {
    // RegisterFrame signature (verified upstream):
    //   Vector3dVectorTuple RegisterFrame(const std::vector<Eigen::Vector3d>& frame,
    //                                     const std::vector<double>& timestamps,
    //                                     const Sophus::SE3d& lidar_to_base,
    //                                     const Sophus::SE3d& relative_odometry);
    // For 2D LaserScan we have no per-point timestamps → empty vector.
    // For dock-snapshot mode (robot stationary or near-stationary), the
    // relative_odometry between frames is small. We pass identity each call
    // because the "last_pose_" is the dock anchor (we don't track delta
    // robot motion via wheel odom — the dock pose is the only target).
    auto [registered_frame, kpoints] =
        kicp_.RegisterFrame(live_frame, /*timestamps=*/{},
                            lidar_to_base, Sophus::SE3d{});

    // Pose after registration: kicp_.pose() is the body pose in map frame.
    Sophus::SE3d pose_in_map = kicp_.pose();

    // Compute confidence post-hoc from registered_frame ↔ local_map nearest
    // neighbours. RegisterFrame returns the registered (transformed) frame
    // and its keypoints; we query the voxel map for nearest neighbours.
    auto [inlier_ratio, rmse] = compute_confidence(
        registered_frame, kicp_.VoxelMap(), config_.max_correspondence_distance);

    return MatchResult{pose_in_map, inlier_ratio, rmse};
  }

private:
  kinematic_icp::pipeline::KinematicICP kicp_;
  // ...
};
}  // namespace mowgli_lidar_docking
```

**Confidence computation (no upstream API exposes it directly; we compute it ourselves):**

```cpp
// Source: derived from KinematicICP source; nearest-neighbour query is on the
// public kiss_icp::VoxelHashMap interface.
// VoxelHashMap exposes: GetClosestNeighbor(const Eigen::Vector3d&) -> Eigen::Vector3d
struct ConfidenceResult { double inlier_ratio; double rmse_m; };

ConfidenceResult compute_confidence(
    const std::vector<Eigen::Vector3d>& registered_frame,
    const kiss_icp::VoxelHashMap& voxel_map,
    double max_correspondence_distance)
{
  size_t n_inliers = 0;
  double sse = 0.0;
  for (const auto& p : registered_frame) {
    const auto nn = voxel_map.GetClosestNeighbor(p);
    const double d2 = (p - nn).squaredNorm();
    if (d2 <= max_correspondence_distance * max_correspondence_distance) {
      ++n_inliers;
      sse += d2;
    }
  }
  const double inlier_ratio = registered_frame.empty()
    ? 0.0
    : static_cast<double>(n_inliers) / registered_frame.size();
  const double rmse = n_inliers > 0 ? std::sqrt(sse / n_inliers) : INFINITY;
  return {inlier_ratio, rmse};
}
```

`[VERIFIED: KinematicICP.hpp public methods include VoxelMap()/pose() accessors; kiss_icp::VoxelHashMap public API includes GetClosestNeighbor(); RegisterFrame return type is std::tuple<Vector3dVector, Vector3dVector> per upstream signature]`

`[ASSUMED: VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d>&) is public — confirmed in upstream KinematicICP::RegisterFrame implementation, which calls local_map_.Update()/AddPoints() to incorporate the registered frame. Planner: verify by reading kiss_icp/core/VoxelHashMap.hpp once submodule is initialised. If the method is private, the workaround is to construct KinematicICP, then synthesise a "scan from PCD" RegisterFrame call once, which causes the local map to ingest the dock points naturally.]`

### Pattern 2: 2D LaserScan → vector<Eigen::Vector3d> conversion

**What:** Convert sensor_msgs/LaserScan to point cloud points usable by KinematicICP::RegisterFrame.

**When to use:** Per-scan callback in dock_scan_match_node, also when capturing the dock_scan.pcd in `calibrate_imu_yaw_node` extension.

**Source (verified upstream — kinematic_icp_online_node uses this exact path):**

```cpp
// VERIFIED: PRBonn kinematic-icp online_node.cpp use_2d_lidar branch
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class DockScanMatchNode : public rclcpp::Node {
  laser_geometry::LaserProjection projector_;

  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
    sensor_msgs::msg::PointCloud2 pc2;
    // -1.0 means "no per-point time interpolation" — matches upstream
    projector_.projectLaser(*msg, pc2, -1.0,
                            laser_geometry::channel_option::Timestamp);

    std::vector<Eigen::Vector3d> points;
    points.reserve(pc2.height * pc2.width);
    sensor_msgs::PointCloud2ConstIterator<float> it(pc2, "x");
    for (size_t i = 0; i < pc2.height * pc2.width; ++i, ++it) {
      points.emplace_back(it[0], it[1], it[2]);
    }
    // points are now in lidar_link_wheels frame (LaserScan frame_id was rewritten
    // by kinematic_icp_scan_frame_relay).

    // ... crop, register, publish ...
  }
};
```

### Pattern 3: PCD save/load (PCL ASCII per D-06)

**What:** Save `dock_scan.pcd` from a captured set of `Eigen::Vector3d`; load it on startup.

**Source:**

```cpp
// VERIFIED: PCL docs — pcl::io::savePCDFile signature is stable since PCL 1.7
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mowgli_lidar_docking {

bool save_dock_scan_pcd(const std::string& path,
                        const std::vector<Eigen::Vector3d>& points)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  for (const auto& p : points) {
    cloud.emplace_back(static_cast<float>(p.x()),
                       static_cast<float>(p.y()),
                       static_cast<float>(p.z()));
  }
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  // savePCDFile returns int (0 on success) — third arg `binary_mode = false`
  // gives the ASCII format mandated by D-06.
  return pcl::io::savePCDFile(path, cloud, false) == 0;
}

bool load_dock_scan_pcd(const std::string& path,
                        std::vector<Eigen::Vector3d>& out_points)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud) != 0) return false;
  out_points.clear();
  out_points.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    out_points.emplace_back(p.x, p.y, p.z);
  }
  return true;
}

}  // namespace mowgli_lidar_docking
```

**Atomic write of dock_scan.pcd (R-11 auto-refresh):** wrap with the existing helper:

```cpp
#include <mowgli_geometry/atomic_write.hpp>
// atomic_write writes via temp + rename(2). Reuse for the PCD path:
bool save_dock_scan_pcd_atomic(const std::string& path, const std::vector<Eigen::Vector3d>& pts) {
  return mowgli_geometry::atomic_write(path, [&](const std::string& tmp_path) {
    return save_dock_scan_pcd(tmp_path, pts);
  });
}
```

`[VERIFIED: ls of /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2/src/mowgli_geometry/include/mowgli_geometry/ — atomic_write.hpp exists]`

### Pattern 4: BT StatefulActionNode for FineDock (continuous control)

**What:** A BT node that ticks a control loop continuously while in RUNNING, observing /dock_match/pose and publishing /cmd_vel_docking, returning SUCCESS when is_charging engages.

**When to use:** Phase 2's central control loop. Mirrors `SeedYawFromMotion` in `calibration_nodes.cpp` which uses the same BT.CPP v4 StatefulActionNode pattern.

**Reference (existing code):**

```cpp
// SOURCE: ros2/src/mowgli_behavior/src/calibration_nodes.cpp:327-463 (SeedYawFromMotion)
// EXTRACTED PATTERN — mirror this for FineDock
class SeedYawFromMotion : public BT::StatefulActionNode {
public:
  BT::NodeStatus onStart() override {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    if (!cmd_pub_) {
      cmd_pub_ = ctx->node->create_publisher<geometry_msgs::msg::TwistStamped>(
          "/cmd_vel_teleop", 10);   // FineDock uses /cmd_vel_docking instead
    }
    // Lazy init: subscribe /dock_match/pose, /dock_match/confidence here.
    // Capture pose, send first cmd_vel.
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    if (ctx->latest_emergency.active_emergency || ctx->latest_emergency.latched_emergency) {
      publish_zero(ctx->node);
      return BT::NodeStatus::FAILURE;
    }
    // ... compute control, publish_forward(speed); read latest /dock_match/pose ...
    return BT::NodeStatus::RUNNING;  // or SUCCESS on is_charging, FAILURE on confidence loss
  }
  void onHalted() override {
    publish_zero(ctx->node);  // CRITICAL: stop motion on halt
  }
};
```

**FineDock specifics (planner: design verbatim from this skeleton):**
- onStart: lazy-init publisher to `/cmd_vel_docking` (NOT teleop), latest-message subscriber to `/dock_match/pose` and `/dock_match/confidence`. Snapshot start time + start pose.
- onRunning: every tick, read latest `/dock_match/pose` + `confidence`, compute `lateral_y_err` and `yaw_err` in dock frame (rotation = `yaw_to_dock` from `dock_approach.yaml`); publish `linear.x = crawl_speed_ms`, `angular.z = -k_lateral * lateral_y - k_yaw * yaw_err` (sign convention: positive lateral_y → robot drifted left of dock approach line → steer right). Stop on `BTContext::latest_status.is_charging == true` (return SUCCESS). On `confidence.trusted == false` for ≥ 1 s (rolling counter against ROS clock), return FAILURE with structured log line.
- onHalted: publish zero twist immediately. Critical for E-Stop-during-FineDock semantics (R-9).
- After E-Stop reset, FineDock does NOT auto-restart. The OuterReactiveSequence in main_tree.xml is responsible for not re-issuing the goal — operator must send COMMAND_HOME again (R-9 semantics).

### Pattern 5: TF lookup with fallback in BT (RecordDockApproachPose)

**Reference (existing code):**

```cpp
// SOURCE: ros2/src/mowgli_behavior/src/calibration_nodes.cpp:89-107
std::optional<std::pair<double, double>> lookup_odom_xy(
    const std::shared_ptr<BTContext>& ctx, const char* who)
{
  try {
    auto tf = ctx->tf_buffer->lookupTransform(
        "odom", "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(0.2));
    return std::make_pair(tf.transform.translation.x, tf.transform.translation.y);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(ctx->node->get_logger(), "%s: TF lookup failed: %s", who, ex.what());
    return std::nullopt;
  }
}
```

**For RecordDockApproachPose:** prefer `/dock_match/pose` (when `confidence.trusted == true`), else `map → base_footprint` TF, else SUCCESS-with-warn (so the rest of UndockSequence still runs and the EKF dock seed remains the dock_calibration.yaml fallback).

### Anti-Patterns to Avoid

- **Anti-pattern: subscribing to `/scan` directly in dock_scan_match.** Use `/scan_kicp` — frame_id is already rewritten to `lidar_link_wheels` (parallel TF tree) so K-ICP's TF lookups stay decoupled. `[VERIFIED: kinematic_icp_scan_frame_relay.py]`
- **Anti-pattern: subscribing to `/odometry/filtered_map` for the match motion prior.** That feeds robot_localization output back into a LiDAR-derived signal — Architecture Invariant #1 violation. Use identity for `relative_odometry` since the dock anchor is a fixed reference; rely on the kinematic prior's regularization to keep the registration honest.
- **Anti-pattern: using `tf2::TimePointZero` AND specifying a heavy duration timeout.** Established mowgli pattern: TimePointZero (latest available) with a 200 ms timeout for staleness tolerance. `[VERIFIED: calibration_nodes.cpp:94, kinematic_icp_scan_frame_relay.py:115]`
- **Anti-pattern: writing dock_approach.yaml directly from a BT tick.** Use mowgli_geometry's `atomic_write` (Phase 1 D-05 lock). BT can call a helper function in `mowgli_lidar_docking` lib synchronously — no need for a separate write service like Phase 1 used for checkpoints (those required ARM-DDS isolation; dock_approach.yaml is one tiny file written once per undock). `[VERIFIED: STATE.md decision log: Plan 01-11 BT-side synchronous WriteCheckpoint via DDS; for Phase 2's even simpler write, direct call is acceptable per Plan 01-08 cost/benefit analysis]`
- **Anti-pattern: calling `RegisterFrame` with the same frame on every tick when robot is stationary.** Wastes CPU. Skip the call when wheel-odom delta is below threshold (e.g., 1 mm in X or 0.1° in yaw); republish last result instead. **Performance note:** at 10 Hz on the Pi5 with ~450-point cropped scan, expect ~20-40 ms per call (KinematicICP with `max_num_iterations=20`, `max_num_threads=1`). Pi5 has 4 cores; this is well within budget. `[ASSUMED based on K-ICP's existing 10-Hz operation in this stack]`

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| 2D ICP solver | Custom point-to-point ICP | `kinematic_icp::pipeline::KinematicICP` | Already in tree; SPEC mandate; kinematic prior crucial for V-funnel symmetry resolution |
| LaserScan → PointCloud projection | Manual angle-loop | `laser_geometry::LaserProjection::projectLaser` | Standard ROS2 pkg; handles range_min/max + intensity channels |
| PCD I/O | Custom XYZ-text parser | `pcl::io::savePCDFile`/`loadPCDFile` ASCII mode | PCL is the canonical PCD format; pcl_viewer compat |
| Atomic file write | Custom temp + rename | `mowgli_geometry::atomic_write` | Already exists in mowgli_geometry; Phase 1 reused |
| BT.CPP v4 long-running control | Custom timer threads | `BT::StatefulActionNode` (onStart/onRunning/onHalted) | Standard BT.CPP pattern; SeedYawFromMotion is the working reference |
| TF Yaw extraction | atan2 from quaternion components | `tf2::getYaw(geometry_msgs::msg::Quaternion)` | Standard; calibration_nodes.cpp:230 |
| Quaternion from yaw | manual `cos/sin` | `tf2::Quaternion::setRPY(0, 0, yaw)` | Standard; calibration_nodes.cpp:274 |
| Codegen of new msg | Hand-write Go/TS structs | Run `gui/generate_go_msgs.sh` + `gui/generate_ts_types.sh` | Phase 1 hooked these up; they read mowgli_interfaces/msg verbatim |

**Key insight:** The dock-approach problem is well-bounded by the existing parallel TF tree (Architecture Invariant #1 already protected) and the existing `/scan_kicp` substrate. The new code is mostly *plumbing* between an existing solver, an existing seeder, an existing BT pattern, an existing atomic-write helper, and an existing GUI relay. The only genuinely new mathematical content is the inlier_ratio/rmse computation (10 lines of C++).

## Runtime State Inventory

> Phase 2 introduces NEW filesystem state and EXTENDS existing seeder state. No rename/refactor.

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | NEW: `/ros2_ws/maps/dock_scan.pcd` (one-time + 7-day refresh), `/ros2_ws/maps/dock_scan_meta.yaml`, `/ros2_ws/maps/dock_approach.yaml` | New writes; ensure operator-facing volume mount in docker-compose covers `/ros2_ws/maps/` (already mounted) |
| Live service config | None — no n8n, no Datadog, no third-party live registrations | None |
| OS-registered state | None — no Task Scheduler, pm2, systemd unit additions | None |
| Secrets/env vars | None | None |
| Build artifacts / installed packages | NEW package `mowgli_lidar_docking` builds a static lib + executable + headers; new mowgli_interfaces msg `DockMatchConfidence` regenerates Go/TS bindings; firmware bindings unchanged (msg is GUI-side only) | Standard `colcon build`; re-run `generate_go_msgs.sh` + `generate_ts_types.sh` after .msg add; **no firmware rebuild needed** |

**Existing state at risk:** `/ros2_ws/maps/dock_calibration.yaml` is read by 4 consumers (per SPEC §Background): `navigation.launch.py`, `hardware_bridge`, `behavior_tree_node`, `dock_yaw_to_set_pose`. Phase 2 reads it as the "dock anchor in map frame" for the ICP — adds a 5th consumer in `dock_scan_match`. No writes to dock_calibration.yaml; no schema change. `[VERIFIED: hardware_bridge_node.cpp:130-143]`

## Common Pitfalls

### Pitfall 1: ICP returning a flipped pose (180° yaw ambiguity)

**What goes wrong:** With a near-symmetric V-funnel scan, ICP can converge to a 180°-rotated solution if the initial guess is off. The match looks reasonable (high inlier_ratio because most points still align) but the pose is backwards.

**Why it happens:** Kinematic-ICP's kinematic prior weakens this somewhat (the prior says "robot is moving forward, not flipped 180°") but a stationary robot has no motion prior signal to disambiguate.

**How to avoid:** The dock_scan_match node ALWAYS calls `SetPose(dock_anchor_in_map)` *before* the first registration when the robot's last known pose deviates from the dock area by > crop_radius_m. This anchors the initial guess to the correct hemisphere. Additionally, post-registration, reject any match whose yaw deviates > 90° from `dock_anchor.yaw` — flag as untrusted via the existing trusted bool, log a WARNING.

**Warning signs:** Position match is good (within 5 cm) but FineDock control overshoots / circles; lateral_y sign appears flipped on the cmd_vel; operator sees "robot mowed forward into dock instead of approaching from the staging point".

### Pitfall 2: dock_scan.pcd captured at a slightly-tilted dock pose

**What goes wrong:** First-time calibration captures `dock_scan.pcd` while the robot is still ~5 cm off the dock laterally (BackUp distance, charging continues). The dock geometry in the PCD is slightly rotated relative to the dock_calibration.yaml anchor.

**Why it happens:** The `dock_yaw_to_set_pose` cascade is approximate (file yaw is ±1°). The robot can be on the dock without being perfectly straight.

**How to avoid:** Capture the PCD when `is_charging == true` AND `wheel_vx ≈ 0` AND TF lookup `map → base_footprint` is fresh. Take 5 consecutive `/scan_kicp` messages and AVERAGE them (point-cloud merge with VoxelHashMap voxelisation, then sample max-points-per-voxel back out). This averages out wheel-position noise. `[CITED: standard practice in scan-match snapshot capture; same trick as openMR's dock-template capture]`

**Warning signs:** First docking attempt after fresh calibration has higher rmse (3-5 cm) than subsequent ones; recapture button quickly fixes it.

### Pitfall 3: foxglove_bridge custom-message typesupport (the documented bug)

**What goes wrong:** Adding `DockMatchConfidence` as a custom mowgli_interfaces msg is fine for **topic** publication. But if anyone later adds a service that returns this msg type, foxglove_bridge can't dispatch the service response back to GUI clients. `[CITED: project_foxglove_typesupport_bug.md memory; CalibrateImuYaw service split-pattern in CalibrateImuYawStatus.msg comments]`

**Why it happens:** rmw_cyclonedds GenericClient typesupport bug for multi-field service responses.

**How to avoid:** D-12 already locks the GUI bridge as topic-only — `/dock_match/pose` and `/dock_match/confidence` are TOPICS, not service responses. Auto-generated TS bindings work cleanly through topic delivery. NO service is needed for the GUI confidence display. The recapture button (D-11) calls a *different* service (`/calibrate_imu_yaw_node/calibrate` already exists) and uses the same topic-side response pattern that calibrate_imu_yaw_node uses today.

**Warning signs:** ~/calibrate~/dock_match/* service replies that work in `ros2 service call` but don't reach the GUI (silent timeout in browser).

### Pitfall 4: rosbridge_server vs foxglove_bridge confusion in D-12

**What goes wrong:** D-12 says "direct WebSocket via foxglove_bridge". A literal reading suggests using `roslib` (rosbridge_v2 client) to connect to a rosbridge_server. **But this fork does not run rosbridge_server** — it was replaced by foxglove_bridge per `gui/web/src/hooks/useCoveragePlan.ts:9-15` and committed history.

**Why it happens:** Phase 1 D-13 (the cited pattern) used the SAME terminology but ALSO ended up routing through the Go provider (per `useCoveragePlan` final design). The "direct WebSocket" wording is aspirational; pragmatically every existing live-data hook (`useMagYaw`, `useDockingSensor`, `useGPS`, `usePower`) uses the `topicMap` Go relay pattern.

**How to avoid:** In the plan, treat D-12 as "auto-generated TS types + topicMap entry" (literal interpretation matching every other hook in the project). Adding `dockMatchPose` and `dockMatchConfidence` keys to `gui/pkg/providers/ros.go::topicMap` is 2 lines. The GUI hook subscribes to `/api/mowglinext/subscribe/dockMatchConfidence` exactly like `useMagYaw`. If the planner needs literal direct-foxglove-bridge access, the alternative is wiring `@foxglove/ws-protocol` browser-side — significantly more work, no precedent.

**Warning signs:** Plan task creates a `roslib`-based hook → fails because rosbridge_server is absent.

### Pitfall 5: Pi5 CPU starvation under K-ICP + dock_scan_match concurrent

**What goes wrong:** Both the existing `kinematic_icp_online_node` (10 Hz on full 360° scan) and the new `dock_scan_match` (10 Hz on cropped ±3 m scan) ICP-register at the same cadence. Plus Nav2 + 2 EKFs + collision_monitor. Under contention on the Pi5's 4 cores, latency can spike.

**Why it happens:** Both Kinematic-ICP instances default `max_num_threads: 1` (to avoid thread-explosion), but they both compete for whichever core the executor schedules.

**How to avoid:**
- `dock_scan_match` runs ICP only when robot is within ~5 m of the dock_anchor (gate on TF distance to `dock_calibration.yaml` pose). When far away, the node skips RegisterFrame and republishes a "not_trusted" confidence.
- Use a `MultiThreadedExecutor` for the new node ONLY if measurements show single-threaded scheduling lag. Default is single-threaded since the gate above keeps duty cycle low.
- For the FineDock 1.5 m crawl phase, the existing K-ICP for navigation dead-reckoning is essentially idle (robot very slow, scans nearly identical) — both nodes can co-exist.
- `[ASSUMED: Pi5 budget headroom; no measured baseline available in repo. Planner: include a smoke task that times one full RegisterFrame on Pi5 hardware before declaring R-7 acceptable.]`

**Warning signs:** Nav2 controller_server reports "Could not transform" warnings; cmd_vel becomes choppy; CPU monitor shows >80% on a single core.

### Pitfall 6: Stale /dock_match/pose during FineDock right after the BT subscribes

**What goes wrong:** FineDock's onStart() lazy-initialises the `/dock_match/pose` subscriber. The first onRunning() tick fires before any pose has been received (subscriber buffer empty). FineDock might issue a control command based on the stale `last_pose_` from a prior session.

**Why it happens:** Standard pub-sub timing race; no transient_local on the dock_match topics (volatile is correct for live data).

**How to avoid:** FineDock's onRunning() must check `last_dock_match_received_at` against `node->now()` and bail (RUNNING for up to N seconds, then FAILURE) if no fresh message has arrived. Patterns: matches `dock_yaw_to_set_pose._try_publish` early-bail on `_latest_gps is None`. Set N = 2 s (20 message cycles).

**Warning signs:** FineDock instantly publishes a non-zero cmd_vel despite robot being far from dock; first 1 s of run has visibly wrong steering direction.

### Pitfall 7: `dock_calibration.yaml` written by `calibrate_imu_yaw_node` uses `yaml.safe_dump` (Python yaml-cpp avoidance does NOT apply to Python)

**What goes wrong:** SPEC R-1 says the dock_scan capture extension also writes `dock_scan_meta.yaml`. If the planner uses Python yaml.safe_dump (matching `_run_dock_yaw_drive` write pattern), the output is conventional YAML — readable by yaml.safe_load on the C++ read side IF using a Python helper, but if `dock_scan_match.cpp` reads it, we need either yaml-cpp (forbidden) or a flat key=value file (D-04 style).

**Why it happens:** Two sides — Python writer (Python tools available) vs. C++ reader (no yaml-cpp by Stack convention).

**How to avoid:** Write `dock_scan_meta.yaml` in flat key=value style from Python too:
```python
with open(meta_path, 'w') as fh:
    fh.write(f"dock_scan_captured_at: {iso_ts}\n")
    fh.write(f"dock_scan_pose_x: {pose.x:.6f}\n")
    fh.write(f"dock_scan_pose_y: {pose.y:.6f}\n")
    fh.write(f"dock_scan_pose_yaw_rad: {pose.yaw:.6f}\n")
    fh.write(f"dock_scan_point_count: {n_points}\n")
```
Then C++ reads with the same `parse_yaml_double` scanner from hardware_bridge_node.cpp:130-143. Date string is parsed in C++ via `std::chrono::sys_seconds` or simple ISO comparison (just compare YYYY-MM-DD substrings for the 7-day check). `[VERIFIED: existing pattern in hardware_bridge_node.cpp]`

**Warning signs:** C++ side throws on yaml.safe_dump output (multi-line block scalars, anchors, etc.).

## Code Examples

### Example 1: dock_scan_match_node skeleton

```cpp
// Source: derived from existing patterns in mowgli_localization.
// File: ros2/src/mowgli_lidar_docking/src/dock_scan_match_node.cpp

#include <chrono>
#include <memory>

#include <Eigen/Core>
#include <kinematic_icp/pipeline/KinematicICP.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sophus/se3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mowgli_interfaces/msg/dock_match_confidence.hpp>

#include "mowgli_lidar_docking/idock_matcher.hpp"
#include "mowgli_lidar_docking/kinematic_icp_dock_matcher.hpp"
#include "mowgli_lidar_docking/dock_scan_io.hpp"

namespace mowgli_lidar_docking {

class DockScanMatchNode : public rclcpp::Node {
public:
  explicit DockScanMatchNode(const rclcpp::NodeOptions& opts = {})
    : rclcpp::Node("dock_scan_match", opts)
  {
    // ---- params ----
    crop_radius_m_         = declare_parameter<double>("crop_radius_m", 3.0);
    min_inlier_ratio_      = declare_parameter<double>("min_inlier_ratio", 0.70);
    max_rmse_m_            = declare_parameter<double>("max_rmse_m", 0.05);
    max_correspondence_m_  = declare_parameter<double>("max_correspondence_distance", 0.30);
    max_iterations_        = declare_parameter<int>("max_iterations", 20);
    dock_calibration_path_ = declare_parameter<std::string>(
        "dock_calibration_path", "/ros2_ws/maps/dock_calibration.yaml");
    dock_scan_path_        = declare_parameter<std::string>(
        "dock_scan_path", "/ros2_ws/maps/dock_scan.pcd");
    dock_scan_meta_path_   = declare_parameter<std::string>(
        "dock_scan_meta_path", "/ros2_ws/maps/dock_scan_meta.yaml");
    publish_rate_hz_       = declare_parameter<double>("publish_rate_hz", 10.0);

    // ---- TF ----
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Publishers ----
    rclcpp::QoS pose_qos(1);  pose_qos.reliable();
    rclcpp::QoS conf_qos = rclcpp::SensorDataQoS();
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/dock_match/pose", pose_qos);
    pub_conf_ = create_publisher<mowgli_interfaces::msg::DockMatchConfidence>(
        "/dock_match/confidence", conf_qos);

    // ---- Load dock_scan.pcd + dock_calibration.yaml ----
    std::vector<Eigen::Vector3d> dock_points;
    if (!load_dock_scan_pcd(dock_scan_path_, dock_points)) {
      RCLCPP_WARN(get_logger(),
          "No %s — dock_scan_match in DEGRADED mode (no matches will publish until "
          "operator captures via calibrate_imu_yaw_node).",
          dock_scan_path_.c_str());
      degraded_ = true;
      return;
    }
    auto dock_anchor = load_dock_anchor_from_calibration(dock_calibration_path_);
    if (!dock_anchor) {
      RCLCPP_ERROR(get_logger(), "No dock_calibration.yaml at %s — cannot operate",
          dock_calibration_path_.c_str());
      degraded_ = true;
      return;
    }
    dock_anchor_in_map_ = *dock_anchor;

    // ---- Build matcher ----
    kinematic_icp::pipeline::Config kicp_cfg;
    kicp_cfg.max_range = 8.0;       // dock_scan envelope ~8 m
    kicp_cfg.min_range = 0.4;       // chassis self-reflection cutoff
    kicp_cfg.voxel_size = 0.1;      // tighter than nav K-ICP — dock has fine detail
    kicp_cfg.max_points_per_voxel = 5;
    kicp_cfg.use_adaptive_threshold = false;
    kicp_cfg.fixed_threshold = max_correspondence_m_;
    kicp_cfg.max_num_iterations = max_iterations_;
    kicp_cfg.convergence_criterion = 0.001;
    kicp_cfg.max_num_threads = 1;
    kicp_cfg.use_adaptive_odometry_regularization = true;
    kicp_cfg.deskew = false;
    matcher_ = std::make_unique<KinematicIcpDockMatcher>(
        kicp_cfg, dock_points, dock_anchor_in_map_);

    // ---- Subscribe last so matcher is ready ----
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_kicp", rclcpp::SensorDataQoS(),
        std::bind(&DockScanMatchNode::on_scan, this, std::placeholders::_1));

    // Publish at 10 Hz from a wall_timer using the latest scan/match. (Could
    // also publish per-scan; timer makes the cadence deterministic and keeps
    // RegisterFrame off the scan callback path so the subscriber never blocks.)
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_)),
        std::bind(&DockScanMatchNode::tick, this));

    RCLCPP_INFO(get_logger(),
        "dock_scan_match ready: dock=(%.3f, %.3f, %.1f°), %zu dock_scan points, "
        "thresholds inlier≥%.2f rmse≤%.3f m",
        dock_anchor_in_map_.translation().x(),
        dock_anchor_in_map_.translation().y(),
        tf2::getYaw(...) * 180.0 / M_PI,  // ... compose from SE3d rotation
        dock_points.size(), min_inlier_ratio_, max_rmse_m_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(scan_mtx_);
    last_scan_ = msg;
  }

  void tick() {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    {
      std::lock_guard<std::mutex> lock(scan_mtx_);
      scan = last_scan_;
    }
    if (degraded_ || !scan || !matcher_) {
      // Publish "not trusted" so consumers can gate.
      publish_not_trusted();
      return;
    }

    // 1. Project LaserScan → Eigen points (in lidar_link_wheels frame)
    sensor_msgs::msg::PointCloud2 pc2;
    projector_.projectLaser(*scan, pc2, -1.0,
                            laser_geometry::channel_option::Timestamp);
    std::vector<Eigen::Vector3d> frame;
    frame.reserve(pc2.height * pc2.width);
    sensor_msgs::PointCloud2ConstIterator<float> it(pc2, "x");
    for (size_t i = 0; i < pc2.height * pc2.width; ++i, ++it) {
      frame.emplace_back(it[0], it[1], it[2]);
    }

    // 2. Lookup lidar_to_base extrinsic
    Sophus::SE3d lidar_to_base;
    try {
      auto tf = tf_buffer_->lookupTransform(
          "base_footprint_wheels", scan->header.frame_id,
          tf2::TimePointZero, tf2::durationFromSec(0.1));
      lidar_to_base = tf2_to_sophus(tf.transform);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "TF lookup failed: %s — skipping match", ex.what());
      publish_not_trusted();
      return;
    }

    // 3. Crop ±crop_radius_m around dock anchor (in map frame). Need to
    //    transform points to map frame for the crop check, OR transform
    //    the dock anchor into the lidar frame and crop there. Either is
    //    fine; crop in lidar frame is cheaper.
    crop_around_dock_in_lidar_frame(frame, lidar_to_base);

    // 4. Run match
    auto result = matcher_->Match(frame, lidar_to_base);

    // 5. Publish
    publish_match(result);
  }

  // ... helpers ...

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<mowgli_interfaces::msg::DockMatchConfidence>::SharedPtr pub_conf_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  laser_geometry::LaserProjection projector_;
  std::unique_ptr<IDockMatcher> matcher_;
  Sophus::SE3d dock_anchor_in_map_;
  std::mutex scan_mtx_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr last_scan_;
  bool degraded_{false};
  // ... params ...
};

}  // namespace mowgli_lidar_docking

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_lidar_docking::DockScanMatchNode>());
  rclcpp::shutdown();
  return 0;
}
```

### Example 2: DockMatchConfidence.msg

```
# DockMatchConfidence.msg
#
# Published by mowgli_lidar_docking/dock_scan_match at 10 Hz alongside
# /dock_match/pose. Downstream consumers (FineDock BT, dock_yaw_to_set_pose,
# RecordDockApproachPose, GUI) MUST gate on `trusted` before using the pose.
#
# Trust threshold (defaults, overridable via dock_scan_match params):
#   trusted = (inlier_ratio >= min_inlier_ratio) AND (rmse_m <= max_rmse_m)
#
# Topic:  /dock_match/confidence  (sensor_msgs::SensorDataQoS)
# Pose companion topic: /dock_match/pose (PoseWithCovarianceStamped, reliable depth=1)

std_msgs/Header header
float32 inlier_ratio    # [0.0, 1.0] — fraction of registered points within max_correspondence_distance
float32 rmse_m          # metres — sqrt(mean(squared_residuals)) over inliers
bool    trusted         # convenience: precomputed gate (saves consumer-side recomputation)
```

### Example 3: ApproachDock BT (skeleton)

```cpp
// File: ros2/src/mowgli_behavior/src/docking_nodes.cpp
// Pattern: standard nav2_msgs::action::NavigateToPose action client BT node.

class ApproachDock : public BT::StatefulActionNode {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ApproachDock(const std::string& name, const BT::NodeConfig& cfg)
    : BT::StatefulActionNode(name, cfg) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("dock_approach_path",
                                        "/ros2_ws/maps/dock_approach.yaml") };
  }

  BT::NodeStatus onStart() override {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    std::string yaml_path;
    getInput("dock_approach_path", yaml_path);

    auto approach = mowgli_lidar_docking::load_dock_approach_yaml(yaml_path);
    if (!approach) {
      RCLCPP_ERROR(ctx->node->get_logger(),
          "ApproachDock: cannot read %s — failure", yaml_path.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (!action_client_) {
      action_client_ = rclcpp_action::create_client<NavigateToPose>(
          ctx->node, "navigate_to_pose");
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
      return BT::NodeStatus::FAILURE;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = ctx->node->now();
    goal.pose.pose.position.x = approach->x;
    goal.pose.pose.position.y = approach->y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, approach->yaw_to_dock_rad);
    goal.pose.pose.orientation = tf2::toMsg(q);

    auto future_goal = action_client_->async_send_goal(goal);
    // ... store future, return RUNNING, poll in onRunning() ...
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    // Poll goal_handle status; SUCCESS when result.error_code == NONE
    // and final pose within 10 cm + 5° of target.
  }
  void onHalted() override {
    if (goal_handle_ && action_client_) action_client_->async_cancel_goal(goal_handle_);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandle::SharedPtr goal_handle_;
};
```

### Example 4: dock_yaw_to_set_pose cascade extension (R-4)

```python
# File: ros2/src/mowgli_localization/scripts/dock_yaw_to_set_pose.py
# EXTENSION: add /dock_match/pose subscriber and prefer it over file when trusted.

# In __init__:
self._sub_dock_match_pose = self.create_subscription(
    PoseWithCovarianceStamped, "/dock_match/pose",
    self._on_dock_match_pose, qos_reliable
)
self._sub_dock_match_conf = self.create_subscription(
    DockMatchConfidence, "/dock_match/confidence",
    self._on_dock_match_conf, qos_sensor
)
self._latest_dock_match_pose: PoseWithCovarianceStamped | None = None
self._latest_dock_match_trusted = False

# In _try_publish: extend cascade lookup BEFORE the existing file-vs-heading branch:
if self._latest_dock_match_trusted and self._latest_dock_match_pose is not None:
    # LiDAR match wins
    yaw_quat = self._latest_dock_match_pose.pose.pose.orientation
    yaw_var = max(self._latest_dock_match_pose.pose.covariance[35], 1e-4)
    source = "lidar"
elif self._file_yaw_rad is not None:
    # ... existing file branch ...
    source = "file"
else:
    yaw_quat = self._latest_heading.orientation
    yaw_var = self._yaw_var
    source = "/gnss/heading"

# Logging: existing log line already prints `source`; just include "lidar" verbatim.
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Open-loop GPS-only `opennav_docking::SimpleChargingDock` final approach | Closed-loop LiDAR-fine FineDock + RTK-coarse ApproachDock | This phase | Eliminates dock-miss failure observed 2026-04-29 (10 cm Y-offset reproducible) |
| FusionCore localization | robot_localization dual-EKF + Kinematic-ICP | 2026-04-26 | Phase 2 builds on top — single localizer invariant must be preserved |
| Phone-compass dock yaw (±10°) | RTK + 2 m undock GPS-track yaw (±1°) via dock_calibration.yaml | 2026-04 | Phase 2 adds LiDAR-match (≤ 0.5° expected) as top-priority cascade entry |
| 5 separate coverage BT nodes (pull pattern) | Single PlanCoverageGoal + FollowCoveragePlan (Plan 01-08) | Phase 1 | Phase 2 follows the SAME monolithic-BT pattern for new docking nodes (no new sub-action explosion) |
| `dock_pose_yaw` from `mowgli_robot.yaml` (legacy) | `dock_calibration.yaml` (4 consumers, persists) | #74 | Phase 2 reads dock_calibration.yaml as the dock anchor; no schema change |

**Deprecated/outdated:**
- `opennav_docking::UndockRobot` — superseded by Nav2 BackUp (Architecture Invariant #10, since Phase 1 era)
- Manual operator dock-yaw with phone compass — superseded by `calibrate_imu_yaw_node` 2-m undock pre-phase since #74

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `kiss_icp::VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d>&)` is public | Pattern 1 | Need a workaround: synthesise a "fake first scan from PCD points" that triggers VoxelHashMap population via the standard RegisterFrame -> internal Update path. Workaround documented in Pattern 1. |
| A2 | `kiss_icp::VoxelHashMap::GetClosestNeighbor(const Eigen::Vector3d&)` is public and returns nearest neighbour as Eigen::Vector3d | Pattern 1 | Need to expose via reflection or replicate logic; ~30 lines of voxel-bucket query code. Plan task should probe by reading kiss_icp/core/VoxelHashMap.hpp once submodule is initialised. |
| A3 | Pi5 can sustain dock_scan_match (10 Hz cropped ICP) + Kinematic-ICP nav (10 Hz full scan) + Nav2 + 2× EKF without breaking ARM scheduling | Pitfall 5 | Mitigation: gate dock_scan_match registration on TF-distance-to-dock; smoke-time RegisterFrame on Pi5 in Wave 0 of plan. |
| A4 | `ros-kilted-laser-geometry`, `ros-kilted-pcl-conversions`, `ros-kilted-pcl-ros`, `libpcl-dev` are available on Kilted Kaiju | Standard Stack | Confirm with `apt-cache policy` in Wave 0. If a package is missing, use the upstream PCL deb. |
| A5 | `dock_scan_meta.yaml` 7-day age check works via simple ISO-8601 string comparison (lexicographic = chronological for valid ISO timestamps) | Pitfall 7 | True for `YYYY-MM-DD` substrings; falsifiable only for invalid timestamps. Plan should enforce YYYY-MM-DD prefix in the writer. |
| A6 | D-12's "direct WebSocket via foxglove_bridge" is operationally equivalent to Phase 1's Go-relay pattern (since rosbridge_server is absent) | Pitfall 4 / D-12 reconciliation | Plan should clarify with the operator before Wave 1 GUI work. If the operator insists on `@foxglove/ws-protocol` browser-side, scope expands by ~1 day (new dependency, new connection-management code). |
| A7 | Nav2 `NavigateToPose` action is suitable for the `ApproachDock` 10-cm tolerance — no custom controller needed | Example 3 | Standard pattern, used by every existing DockRobot site indirectly via opennav_docking which itself goes through Nav2. Confirmed by `goal_blackboard_id: goal` parameter in nav2_params.yaml. |
| A8 | Pre-undock LiDAR rear-sector clearance check (R-12) can read `/scan_kicp` directly within a BT tick without TF lookup overhead | Pattern in BT/architecture map | LaserScan has a fixed angular layout; rear sector indices are deterministic from `angle_min/angle_max/angle_increment`. ~10 lines of C++. |

## Open Questions

1. **D-12 literal interpretation: rosbridge-direct vs Go-relay?**
   - What we know: D-12 says "direct WebSocket via foxglove_bridge", but Phase 1 D-13 used the same wording and ended up at Go-relay (`useCoveragePlan` final design after the rosbridge_server absence was discovered).
   - What's unclear: Should Phase 2 add a new `@foxglove/ws-protocol` dependency for browser-side direct subscription, OR follow the established `topicMap` Go-relay pattern?
   - Recommendation: Follow Go-relay (Pitfall 4 mitigation). Document the deviation from D-12's literal wording in plan-task-level notes; raise as a check question before Wave 1 GUI work if operator wants the literal interpretation.

2. **Dock_scan_meta.yaml format: YAML safe_dump or flat key=value?**
   - What we know: SPEC R-1 calls it "metadata YAML"; existing `dock_calibration.yaml` is yaml.safe_dump (Python writer, Python reader by dock_yaw_to_set_pose).
   - What's unclear: dock_scan_meta needs to be read by C++ (dock_scan_match) and Python (calibrate_imu_yaw_node); flat-format avoids yaml-cpp.
   - Recommendation: Flat key=value (Pitfall 7 mitigation). Trivially compatible with both readers. Documented in research; planner should adopt without further interview.

3. **dock_scan_match degraded-mode startup behavior**
   - What we know: dock_scan.pcd may not exist on first deployment.
   - What's unclear: Does the node refuse to start, start in degraded mode (no publishes), or wait for the file to appear (filewatcher)?
   - Recommendation: Start in degraded mode, log a clear WARN, publish `/dock_match/confidence{trusted=false}` continuously so consumers gate cleanly. Mirror the `mag_yaw_publisher.py` polling-for-mag_calibration.yaml pattern (CLAUDE.md "publisher sits idle until calibration appears, polls every 30 s and self-activates").

4. **Refresh-trigger thread safety**
   - What we know: R-11 says auto-refresh runs in `RecordDockApproachPose` BT node at end of UndockSequence.
   - What's unclear: If `dock_scan_match` is mid-RegisterFrame against the OLD `dock_scan.pcd` and the BT writes a NEW PCD via atomic rename, does the matcher pick up the new file naturally?
   - Recommendation: dock_scan_match loads the PCD ONCE at startup. To pick up a refresh, either (a) restart the node (heavyweight, drops 1-2 s of matches), or (b) implement a file-mtime watcher in dock_scan_match that triggers an `atomic` reload (matcher reset + new VoxelMap population). Recommend (b); ~30 lines of C++.

5. **FineDock control-loop direction sign convention**
   - What we know: Robot drives forward (`+x`) toward dock. Dock anchor yaw points "out of dock"; robot approaches with `yaw_to_dock = atan2(dock - here)`.
   - What's unclear: Is `lateral_y_err > 0` "robot is left of approach line" or "right of"?
   - Recommendation: Discretion item per CONTEXT — pick one convention, document it inline in the FineDock source. Sim test (D-14) catches sign errors before hardware.

## Environment Availability

> Phase 2 introduces 4 new system deps. Probe before Wave 1 starts.

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| `ros-kilted-laser-geometry` | dock_scan_match (LaserScan→PointCloud) | needs verification | — | Inline projection ~30 lines (last-resort; not recommended) |
| `ros-kilted-pcl-conversions` | dock_scan_match, dock_scan_capture | needs verification | — | Manual sensor_msgs/PointCloud2 ↔ pcl::PointCloud<PointXYZ> conversion ~20 lines |
| `ros-kilted-pcl-ros` | dock_scan_match | needs verification | — | Same as above |
| `libpcl-dev` (PCL headers/libs) | save/load PCD | needs verification | — | None acceptable — D-06 requires real PCL |
| `kinematic_icp` (in-tree submodule) | matcher | available (if submodule initialised) | upstream main | None — solver is mandatory per SPEC |
| `Sophus` | RegisterFrame SE3d signature | transitive of kinematic_icp | — | None — required by API |

**Probe commands (run inside dev container):**

```bash
apt-cache policy ros-kilted-laser-geometry ros-kilted-pcl-conversions ros-kilted-pcl-ros libpcl-dev libsophus-dev
ls /opt/ros/kilted/include/laser_geometry 2>&1
ls /opt/ros/kilted/include/pcl_ros 2>&1
ls /opt/ros/kilted/include/sophus 2>&1
ls /usr/include/pcl-1.* 2>&1
git -C ros2/src/kinematic_icp log -1 --oneline 2>&1   # confirm submodule populated
```

**Missing dependencies with no fallback:**
- `kinematic_icp` if submodule not initialised — `git submodule update --init ros2/src/kinematic_icp`
- `libpcl-dev` if not on Kilted apt — `apt install libpcl-dev` (the Debian package; works on Ubuntu 24.04 base)

**Missing dependencies with fallback:**
- `ros-kilted-laser-geometry` — fallback: ~30 lines of inline LaserScan→Eigen conversion (worse: loses the scan-to-pc2 channels handling). Prefer to install the package.

## Validation Architecture

### Test Framework

| Property | Value |
|----------|-------|
| Framework | gtest (C++) via `ament_cmake_gtest` + Python rclpy `e2e_test.py` for integration |
| Config file | `ros2/src/mowgli_lidar_docking/test/CMakeLists.txt` (new), `ros2/src/mowgli_behavior/test/CMakeLists.txt` (extended) |
| Quick run command | `cd ros2 && make test PKG=mowgli_lidar_docking` (per-package), `cd ros2 && colcon test --packages-select mowgli_lidar_docking mowgli_behavior` |
| Full suite command | `cd ros2 && make test` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| R-1 | dock_scan.pcd written by calibrate_imu_yaw_node | unit + manual | `pytest ros2/src/mowgli_localization/test/test_calibrate_imu_yaw_capture.py -k dock_scan` | ❌ Wave 0 |
| R-1 | PCD file readable by pcl_viewer | manual / acceptance | (operator opens pcl_viewer) | manual |
| R-1 | dock_scan_meta.yaml round-trips through parse_yaml_double | unit | `colcon test --packages-select mowgli_lidar_docking --ctest-args -R DockScanMetaRoundTrip` | ❌ Wave 0 |
| R-2 | /dock_match/pose published at ≥ 5 Hz | unit (rate test) + sim | gtest `DockScanMatchNodeRateTest` (in-process, mock matcher) | ❌ Wave 0 |
| R-2 | Stationary on-dock match within 5 cm | hardware | mow_session_monitor extension; operator checklist | manual |
| R-3 | Confidence trips false within 1 s of corrupt scan | unit | gtest `ConfidenceMetricsRejectsNoise` (synthetic noisy points fed to compute_confidence) | ❌ Wave 0 |
| R-4 | Cascade priority lidar > file > heading | unit (Python) | `pytest ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py` | ❌ Wave 0 |
| R-5 | RecordDockApproachPose writes correct yaw_to_dock | unit | gtest `RecordDockApproachPoseGeometry` (mocks TF + dock_match) | ❌ Wave 0 |
| R-6 | ApproachDock returns SUCCESS within 60 s under healthy conditions | unit (mocked NavigateToPose) + sim | gtest `ApproachDockMockClient` + e2e_test.py phase | ❌ Wave 0 |
| R-6 | ApproachDock FAILS immediately on missing yaml | unit | gtest `ApproachDockNoYaml` | ❌ Wave 0 |
| R-7 | FineDock achieves ≤ 2 cm lateral + ≤ 1° yaw at first contact | hardware (5-of-5) | mow_session_monitor + operator checklist; can pre-validate in sim with synthetic_scan_kicp_publisher | manual+sim |
| R-7 | FineDock control loop converges in sim | sim | new e2e_test.py phase: `_run_fine_dock_phase(self)` — synthetic dock + scan publisher; assert is_charging within 60 s | ❌ Wave 0 |
| R-8 | FineDock issues cmd_vel=0 on confidence loss | unit | gtest `FineDockAbortsOnConfidenceLoss` (mock /dock_match publisher) | ❌ Wave 0 |
| R-9 | E-Stop during FineDock stops motion + no auto-restart | unit + hardware | gtest `FineDockHaltsOnEmergency`; manual: operator presses estop during sim | unit ❌, hw manual |
| R-10 | grep -c "DockRobot" main_tree.xml == 0 | smoke / lint | `grep -c "DockRobot" ros2/src/mowgli_behavior/trees/main_tree.xml` returns 0 | ✅ trivial CI step |
| R-11 | Auto-refresh on 8-day-old + inlier ≥ 0.95 → PCD updated | unit | gtest `AutoRefreshTriggersOnHighConfidence` (mock filesystem timestamps) | ❌ Wave 0 |
| R-11 | Auto-refresh skipped when inlier < 0.95 | unit | gtest `AutoRefreshSkipsOnLowConfidence` | ❌ Wave 0 |
| R-12 | PreUndockClearanceCheck fails on rear obstacle | unit | gtest `PreUndockClearanceFailsOnRearObstacle` (synthetic LaserScan with 0.5 m return at 180°) | ❌ Wave 0 |
| R-13 | PostUndockRtkValidation WARN/FAIL thresholds | unit | gtest `PostUndockRtkValidationThresholds` | ❌ Wave 0 |
| End-to-end | COMMAND_START → undock → mow → autodock first attempt | sim + hardware | extended e2e_test.py phase with synthetic dock; hardware via operator | ❌ Wave 0 (sim phase) + manual hw |
| Drift verification | 7-day window auto-refresh fires once with inlier ≥ 0.95 | hardware (long-running) | mow_session_monitor JSONL inspection across runs; not automatable in CI | manual |

### Sampling Rate

- **Per task commit:** `cd ros2 && colcon test --packages-select mowgli_lidar_docking mowgli_behavior --ctest-args -L gtest --output-on-failure` (~30 s)
- **Per wave merge:** `cd ros2 && make test` (~3 min full workspace) plus `make sim` + `make e2e-test` (~5 min sim run, headless)
- **Phase gate:** Full suite green; sim e2e_test passes the new FineDock phase; operator runs Pi5 5-of-5 acceptance per `02-SUMMARY.md` checklist before Phase 2 is closed.

### Wave 0 Gaps

Wave 0 must establish all test infrastructure before Wave 1 implementation begins:

- [ ] `ros2/src/mowgli_lidar_docking/test/CMakeLists.txt` — gtest pattern (mirror Phase 1 mowgli_coverage_planner)
- [ ] `ros2/src/mowgli_lidar_docking/test/test_dock_scan_io.cpp` — PCD round-trip
- [ ] `ros2/src/mowgli_lidar_docking/test/test_confidence_metrics.cpp` — inlier_ratio + rmse against synthetic scans
- [ ] `ros2/src/mowgli_lidar_docking/test/test_dock_approach_loader.cpp` — yaml parser corruption rejection
- [ ] `ros2/src/mowgli_lidar_docking/test/test_idock_matcher_mock.cpp` — D-16 wrapper interface
- [ ] `ros2/src/mowgli_behavior/test/test_docking_nodes.cpp` — extends existing test/ for new BT nodes
- [ ] `ros2/src/mowgli_simulation/scripts/synthetic_scan_kicp_publisher.py` — D-14 sim publisher
- [ ] `ros2/src/e2e_test.py` extension: `_run_fine_dock_phase` Python method + sim-dock placement
- [ ] `ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py` — Python pytest for R-4
- [ ] `ros2/src/mowgli_localization/test/test_calibrate_imu_yaw_capture.py` — Python pytest for R-1 (synthetic /scan_kicp)
- [ ] Framework install: `apt install ros-kilted-laser-geometry ros-kilted-pcl-conversions ros-kilted-pcl-ros libpcl-dev libsophus-dev` (also add to `ros2/Dockerfile`)
- [ ] Submodule init: `git submodule update --init --recursive ros2/src/kinematic_icp`

## Security Domain

Phase 2 has no authentication, session, or network-attack surface beyond what already exists. Probing each ASVS category:

### Applicable ASVS Categories

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V2 Authentication | no | N/A — internal ROS2 IPC over Cyclone DDS, dock_scan files are local |
| V3 Session Management | no | N/A |
| V4 Access Control | partial | dock_scan.pcd / dock_approach.yaml live in `/ros2_ws/maps/` (operator-write, ROS2-read). Filesystem permissions on the volume are the access control. No new authz needed. |
| V5 Input Validation | yes | All YAML reads validate types via the existing `parse_yaml_double` (returns nullopt on parse failure). PCD load via `pcl::io::loadPCDFile` returns non-zero on malformed file → caller bails. ICP outputs validated by inlier_ratio + rmse gate. |
| V6 Cryptography | no | dock_scan.pcd is local data; SPEC explicitly excludes encryption/signing. |

### Known Threat Patterns for {LiDAR + filesystem + EKF seed}

| Pattern | STRIDE | Standard Mitigation |
|---------|--------|---------------------|
| Malformed dock_approach.yaml causes BT/ApproachDock crash | Tampering / DoS | Parser returns optional; ApproachDock returns FAILURE on missing/invalid → BT outer Sequence handles failure path |
| Corrupted dock_scan.pcd causes dock_scan_match crash | Tampering / DoS | pcl::io::loadPCDFile returns error code; node enters degraded mode (publishes trusted=false continuously) |
| Confidence-loss attack: adversarial /scan_kicp degrades match → FineDock aborts | Spoofing / DoS | Already designed-in: trust threshold + 1-s confidence-loss abort. Adversary needs physical access to LiDAR FoV. |
| Stale dock_scan.pcd from previous garden config returns spurious high-confidence match | Tampering | 7-day auto-refresh keeps PCD fresh. Operator can manually recapture via D-11 button. |
| EKF set_pose flooding via spoofed /dock_match/pose | Tampering | dock_yaw_to_set_pose already has rising-edge debounce (30 s) and high_level_state gate (no seed during AUTONOMOUS). The trusted-bool gate in the cascade only enables seeding when the node's own confidence metric clears. |
| FineDock cmd_vel attack via spoofed /dock_match/pose | Tampering | collision_monitor still active (Architecture Invariant #5 holds at fine-dock); twist_mux priority 15 means teleop (priority 20) and emergency (100) override. Firmware safety latch is sole authority for blade. |

**Bottom line:** Phase 2's threat model is filesystem-local + intra-host ROS2 IPC. No network surface added. The trust-gating in D-03/SPEC R-3 is the primary defence-in-depth control, complementing the existing dock_yaw_to_set_pose cascade gates (debounce + high_level_state).

## Sources

### Primary (HIGH confidence)

- `ros2/src/mowgli_localization/scripts/dock_yaw_to_set_pose.py` (read in full) — seeder cascade pattern
- `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp:80-200` (read in full) — dock_calibration parser, no-yaml-cpp pattern
- `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp` (read in full) — BTContext struct
- `ros2/src/mowgli_behavior/src/calibration_nodes.cpp` (read in full) — TF lookup + atomic seed pattern, StatefulActionNode
- `ros2/src/mowgli_localization/scripts/calibrate_imu_yaw_node.py` (read in full) — extension point + service pattern
- `ros2/src/mowgli_localization/scripts/kinematic_icp_scan_frame_relay.py` (read in full) — substrate
- `ros2/src/mowgli_bringup/launch/kinematic_icp.launch.py` (read in full) — K-ICP node spawn pattern
- `ros2/src/mowgli_bringup/launch/navigation.launch.py` lines 280-350 — where to spawn dock_scan_match
- `ros2/src/mowgli_bringup/config/kinematic_icp.yaml` (read in full) — Pi5-tuned K-ICP params
- `ros2/src/mowgli_bringup/config/nav2_params.yaml` lines 580-642 — docking_server config
- `ros2/src/mowgli_bringup/config/twist_mux.yaml` (read in full) — priority 15 verified
- `ros2/src/mowgli_behavior/trees/main_tree.xml` lines 160-540 — 4× DockRobot sites verified
- `ros2/src/mowgli_geometry/include/mowgli_geometry/atomic_write.hpp` (existence verified)
- `ros2/src/mowgli_interfaces/msg/CalibrateImuYawStatus.msg` — DockMatchConfidence template
- `gui/web/src/hooks/useMagYaw.ts`, `useDockingSensor.ts`, `useCoveragePlan.ts` (read in full) — hook patterns
- `gui/pkg/providers/ros.go` lines 1-120 — topicMap relay pattern
- `gui/pkg/api/mowglinext.go` line 253 — dispatch pattern for new keys
- `https://raw.githubusercontent.com/PRBonn/kinematic-icp/main/cpp/kinematic_icp/pipeline/KinematicICP.hpp` — full text retrieved (Config + KinematicICP class verified)
- `https://raw.githubusercontent.com/PRBonn/kinematic-icp/main/cpp/kinematic_icp/registration/Registration.hpp` — KinematicRegistration verified
- `https://raw.githubusercontent.com/PRBonn/kinematic-icp/main/cpp/kinematic_icp/correspondence_threshold/CorrespondenceThreshold.hpp` — verified
- `https://github.com/PRBonn/kinematic-icp/blob/main/ros/src/kinematic_icp_ros/server/LidarOdometryServer.cpp` — RegisterFrame call site verified
- `https://github.com/PRBonn/kinematic-icp/tree/main/ros/src/kinematic_icp_ros/nodes` — online_node use_2d_lidar laser_geometry::projectLaser path verified
- `https://raw.githubusercontent.com/PRBonn/kinematic-icp/main/ros/package.xml` — Sophus + laser_geometry deps verified
- `.planning/phases/01-coverage-planner-rewrite/01-CONTEXT.md` and `01-SPEC.md` — Phase 1 patterns
- `.planning/STATE.md` — Plan 01-08 / 01-11 BT delegation patterns

### Secondary (MEDIUM confidence)

- `https://github.com/PRBonn/kinematic-icp` README, paper abstract — high-level architecture
- `gui/generate_ts_types.sh`, `gui/generate_go_msgs.sh`, `firmware/scripts/sync_ros_lib.py` — codegen pipeline behaviour (verified by inspection of existing CalibrateImuYawStatus codegen)
- PCL docs (`pcl::io::savePCDFile` signature) — stable since PCL 1.7

### Tertiary (LOW confidence — needs validation)

- VoxelHashMap::AddPoints / GetClosestNeighbor public visibility (assumption A1, A2) — verified by reading kiss_icp source once submodule is initialised
- ros-kilted-laser-geometry / ros-kilted-pcl-* package availability on Kilted Kaiju (A4) — verified by `apt-cache policy` in dev container

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — every dep verified against existing usage or upstream package.xml
- Architecture: HIGH — diagram traces existing data flows; new components mirror established patterns
- KinematicICP API: HIGH — public C++ class verified via raw header retrieval; call site pattern verified via upstream LidarOdometryServer.cpp
- Confidence metric computation: MEDIUM — derived from KISS-ICP voxel-map nearest-neighbour API; needs A1/A2 verification
- Pitfalls: MEDIUM-HIGH — most are observed (Pitfall 4 confirmed by Phase 1 history; Pitfall 7 confirmed by hardware_bridge_node.cpp:99 convention)
- GUI integration: HIGH — all four reference hooks verified, topicMap pattern read

**Research date:** 2026-04-29
**Valid until:** 2026-06-15 (the kinematic_icp upstream is on a stable v0.1.x line; mowgli_localization patterns are recently refactored and stable; Pi5 perf assumptions valid until next major hardware change)
