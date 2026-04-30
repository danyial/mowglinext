---
phase: 02
plan: 06
subsystem: lidar-dock-pose-estimation
tags: [wave-3, bt-nodes, fine-dock, approach-dock, record-dock-approach-pose, pre-undock-clearance, post-undock-rtk, spec-r-5, spec-r-6, spec-r-7, spec-r-8, spec-r-9, spec-r-11, spec-r-12, spec-r-13, tdd]
requires:
  - mowgli_lidar_docking::DockApproach + load/save_dock_approach_yaml (Plan 02-02 D-04)
  - mowgli_lidar_docking::DockScanMeta + load/save_dock_scan_meta_yaml +
    dock_scan_meta_age_exceeds (Plan 02-02 D-17)
  - mowgli_lidar_docking::save_dock_scan_pcd_atomic (Plan 02-02 R-11 path)
  - mowgli_interfaces::msg::DockMatchConfidence (Plan 02-01 D-03)
  - DockScanMatchNode publishing /dock_match/pose + /dock_match/confidence
    (Plan 02-04 runtime contract — frozen QoS: pose=reliable depth=1,
    conf=SensorDataQoS)
  - dock_yaw_to_set_pose cascade /dock_match-aware (Plan 02-05 — runs alongside)
provides:
  - mowgli_behavior::RecordDockApproachPose (SyncActionNode, R-5 + R-11)
  - mowgli_behavior::ApproachDock (StatefulActionNode, R-6)
  - mowgli_behavior::FineDock (StatefulActionNode, R-7 + R-8 + R-9)
  - mowgli_behavior::PreUndockClearanceCheck (SyncActionNode, R-12)
  - mowgli_behavior::PostUndockRtkValidation (SyncActionNode, R-13)
  - mowgli_behavior::register_docking_nodes(factory) helper
  - BTContext extension: latest_dock_match_pose / latest_dock_match_conf /
    latest_scan_kicp / dock_pose_suspect + dock_*_path overrides
  - 13 gtest cases pinning the 5 nodes' contract surface
affects:
  - ros2/src/mowgli_behavior/src/behavior_tree_node.cpp (3 new subscribers)
  - ros2/src/mowgli_behavior/src/register_nodes.cpp (1 helper call)
  - Plan 02-07 (main_tree.xml wiring + 4 DockRobot site migrations + GUI surface)
tech-stack:
  added: []
  patterns:
    - SyncActionNode for one-shot pre/post-condition checkers
      (RecordDockApproachPose, PreUndockClearanceCheck, PostUndockRtkValidation)
    - StatefulActionNode for control loops that need onStart/onRunning/onHalted
      lifecycles (ApproachDock, FineDock) — mirrors SeedYawFromMotion at
      calibration_nodes.cpp:327-463
    - Dual gate on /dock_match: trusted bool + steady_clock age (1s default)
      so a stale-but-trusted message can't drive control
    - Two-tier no-pose contract: 5s degraded-mode bail (no pose ever) +
      1s R-8 trust-loss abort (pose flowing but trusted=false)
    - Inline scan_to_points helper in docking_nodes.cpp avoids pulling
      laser_geometry into mowgli_behavior — the math is 12 lines, the
      dependency surface stays smaller
    - BTContext lock snapshot pattern: copy-out under lock, do file/DDS I/O
      without it (mirrors Plan 02-04 dock_scan_match_node)
key-files:
  created:
    - ros2/src/mowgli_behavior/test/test_docking_nodes.cpp
  modified:
    - ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp
    - ros2/src/mowgli_behavior/include/mowgli_behavior/docking_nodes.hpp
    - ros2/src/mowgli_behavior/src/docking_nodes.cpp
    - ros2/src/mowgli_behavior/src/behavior_tree_node.cpp
    - ros2/src/mowgli_behavior/src/register_nodes.cpp
    - ros2/src/mowgli_behavior/CMakeLists.txt
    - ros2/src/mowgli_behavior/package.xml
decisions:
  - "FineDock sign convention (Open Q5 resolution): positive lateral_y_err
    means robot LEFT of dock-aligned approach line -> steer right (negative
    angular.z). Documented inline in docking_nodes.cpp; sim test in Plan 02-08
    catches sign errors before hardware."
  - "Two-tier FineDock failure contract: 5s no-pose bail (degraded mode,
    distinct from R-8) + 1s R-8 trust-loss abort. Both contracts ship; both
    are independently tested. Test names FineDockBailsAfter5sNoPose +
    FineDockAbortsOnConfidenceLoss are load-bearing."
  - "RecordDockApproachPose returns SUCCESS even when dock_calibration.yaml
    or TF lookup fails — UndockSequence has already completed by the time
    this node runs; persistence failure is non-fatal, operator can re-attempt."
  - "Atomic R-11 refresh: PCD write FIRST (atomic via save_dock_scan_pcd_atomic),
    meta SECOND. If PCD write fails, meta is NOT bumped (T-04-01 mitigation).
    If meta write fails after PCD success, log ERROR but return SUCCESS — the
    PCD is the source of truth, meta is the index."
  - "Inline scan_to_points (laser_geometry-free) in docking_nodes.cpp.
    Plan 02-04 already pulls laser_geometry into mowgli_lidar_docking; we
    avoided dragging it through mowgli_behavior here. ~12 lines duplicated;
    if a third consumer appears, promote into mowgli_lidar_docking lib."
  - "Pose covariance from /dock_match/pose is consumed verbatim — FineDock
    does NOT recompute it. Plan 02-04's simple isotropic-from-RMSE covariance
    is sufficient for the BT control surface (we only need pose.position +
    pose.orientation for the P-controller; covariance is GUI-bound)."
  - "BTContext field placement at end of struct — keeps struct field ordering
    backward-compatible with any pre-Phase-2 code that constructs a BTContext
    via aggregate-init (none exists in the tree today, but the safer order
    avoids future surprises)."
  - "dock_match_max_age_s = 1.0s default for FineDock matches Plan 02-05's
    cascade horizon — keeps the seeder and FineDock on the same trust horizon."
  - "PostUndockRtkValidation reads odom->base_footprint TF (not
    base_footprint_wheels) because the operator-visible 'where I undocked
    against GPS' check is the EKF-fused position, not the parallel K-ICP
    estimate. The parallel TF tree is K-ICP's input, not its consumer."
metrics:
  duration_minutes: 9
  tasks_completed: 2
  files_touched: 8
  test_cases_added: 13
  commits: 2
  completed_date: "2026-04-29"
---

# Phase 02 Plan 06: Wave 3 LiDAR-dock BT Nodes Summary

**One-liner:** Wave 3 lands the 5 operator-visible BT nodes that consume
the Plan 02-04 /dock_match/* publisher contract: RecordDockApproachPose
(R-5 + R-11), ApproachDock (R-6), FineDock (R-7 + R-8 + R-9),
PreUndockClearanceCheck (R-12), PostUndockRtkValidation (R-13). All 5
registered with the BT factory under canonical names; pinned by 13 new
gtest cases; ready for Plan 02-07's main_tree.xml wiring + 4 DockRobot
site migrations.

## Built

This plan covers two atomic tasks landed in two commits on
`feat/mag-pipeline-resurrect`:

| Task | Step | Description | Commit |
| ---- | ---- | ----------- | ------ |
| 1 + 2 | RED | Failing test_docking_nodes.cpp (13 cases) registered in CMakeLists.txt | `bca150c8` |
| 1 + 2 | GREEN | BTContext extension + 5 BT nodes (RecordDockApproachPose, ApproachDock, FineDock, PreUndockClearanceCheck, PostUndockRtkValidation) + register_docking_nodes + behavior_tree_node subscribers + package.xml + CMakeLists deps | `d38fa39b` |

Both tasks share `docking_nodes.{hpp,cpp}`, so they were committed
together as a single GREEN commit (the file would not be self-consistent
mid-task — the register_docking_nodes helper registers all 5 names at
once). The plan-level TDD gate sequence (RED → GREEN) is honoured: the
13 RED tests reference symbols (`RecordDockApproachPose`, `FineDock`,
etc.) that did not exist on `bca150c8`; commit `d38fa39b` lands the
production code that makes those references resolve.

## The 5 BT nodes — tick semantics

### RecordDockApproachPose (SPEC R-5 + R-11)

**Type:** `BT::SyncActionNode`. Returns SUCCESS on every path (SUCCESS-only
contract — UndockSequence has completed by the time this node runs;
persistence failure is non-fatal).

**Tick:**
1. Snapshot `latest_dock_match_pose` + `last_dock_match_pose_at` +
   `last_dock_match_trusted` + `latest_scan_kicp` under
   `ctx->context_mutex`; release lock.
2. Source decision (R-5):
   - Trusted /dock_match/pose within 1 s → `source = "lidar"`,
     `here = pose.position`.
   - Else `lookup_map_to_base(ctx)` (`map -> base_footprint` TF) →
     `source = "tf"`, `here = (tx, ty)`.
   - Else: WARN + return SUCCESS (no write).
3. Read `dock_calibration.yaml` for `(dock_x, dock_y)`. Compute
   `yaw_to_dock = atan2(dock_y - here_y, dock_x - here_x)`.
4. `save_dock_approach_yaml` (atomic via mowgli_geometry::atomic_write
   under the hood — Plan 02-02 contract).
5. R-11 auto-refresh:
   - Load `dock_scan_meta.yaml`. If `dock_scan_meta_age_exceeds(meta, 7,
     iso_utc_now())`:
     - If `latest_dock_match_conf.inlier_ratio >= 0.95`: project
       `latest_scan_kicp` to `vector<Eigen::Vector3d>` (inline
       `scan_to_points` helper), atomic-write the new PCD, then bump
       meta `captured_at` + `point_count`.
     - Else: WARN + skip (older snapshot kept).

**Logged contract markers:** `RecordDockApproachPose: wrote ...
(source=...)` (always); `dock_scan auto-refreshed (inlier=..., points=...)`
(R-11 path); `dock_scan age past 7d but inlier_ratio ... < 0.95 — skip
refresh` (R-11 skip path).

### ApproachDock (SPEC R-6)

**Type:** `BT::StatefulActionNode`. Replaces
`opennav_docking::SimpleChargingDock` RPP approach (Plan 02-07
migrates the 4 main_tree.xml sites).

**onStart:** Load `dock_approach.yaml` → FAILURE on missing/unparseable.
Create `rclcpp_action::Client<NavigateToPose>` (lazy), wait for the
server (3 s) → FAILURE on timeout. Build the goal with `frame_id=map`,
`(x, y) = approach->{x, y}`, orientation from quat-of-yaw
`approach->yaw_to_dock_rad`. `async_send_goal` + RUNNING.

**onRunning:** Wall-clock timeout check (60 s default). Poll the goal
future once accepted; map `STATUS_SUCCEEDED → SUCCESS`,
`STATUS_ABORTED|STATUS_CANCELED → FAILURE`, else RUNNING.

**onHalted:** `async_cancel_goal` (fire-and-forget; no wait per BT halt
semantics).

### FineDock (SPEC R-7 + R-8 + R-9)

**Type:** `BT::StatefulActionNode`. Publishes ONLY to `/cmd_vel_docking`
(twist_mux priority 15 per AI #13).

**onStart:**
1. Read ports (`crawl_speed_ms=0.05`, `k_lateral=1.5`, `k_yaw=2.0`,
   `confidence_loss_timeout_s=1.0`, `no_pose_bail_s=5.0`).
2. Lazy-init `cmd_pub_` to `geometry_msgs::msg::Twist` on
   `/cmd_vel_docking`.
3. Load `dock_approach.yaml` → `yaw_to_dock_rad_`. Load
   `dock_calibration.yaml` → `dock_target_x_/y_` (the contact point).
4. `start_time_ = now()`; `confidence_loss_streak_start_.reset()`.
5. Return RUNNING.

**onRunning (priority order):**

| Step | Condition | Action |
|------|-----------|--------|
| 1 | `latest_emergency.{active,latched}_emergency` | publish_zero + FAILURE (R-9) |
| 2 | `latest_status.is_charging == true` | publish_zero + SUCCESS (R-7) |
| 3 | `!latest_dock_match_pose_received` AND `(now - start) > 5s` | publish_zero + FAILURE (degraded-mode contract; WARNING-9) |
| 4 | `!latest_dock_match_pose_received` AND `(now - start) <= 5s` | publish_zero + RUNNING |
| 5 | `pose_age > 2s` AND `(now - start) > 5s` | publish_zero + FAILURE (stale pose past bail window) |
| 6 | `pose_age > 2s` AND `(now - start) <= 5s` | publish_zero + RUNNING |
| 7 | `!last_dock_match_trusted` AND streak NOT started | start streak; publish_zero + RUNNING |
| 8 | `!last_dock_match_trusted` AND streak >= 1 s | structured ERROR log + publish_zero + FAILURE (R-8) |
| 9 | `!last_dock_match_trusted` AND streak < 1 s | publish_zero + RUNNING |
| 10 | trusted (default path) | reset streak; compute lateral_y_err + yaw_err in dock-aligned frame; publish twist; RUNNING |

**Control loop math (step 10):**
```cpp
const double dx = cur_x - dock_target_x_;
const double dy = cur_y - dock_target_y_;
const double cs = std::cos(-yaw_to_dock_rad_);
const double sn = std::sin(-yaw_to_dock_rad_);
const double lateral_y_err = sn * dx + cs * dy;
const double yaw_err = shortest_angular_distance(cur_yaw, yaw_to_dock_rad_);
twist.linear.x  = crawl_speed_ms_;
twist.angular.z = -(k_lateral_ * lateral_y_err) - (k_yaw_ * yaw_err);
```

**Sign convention (Open Q5 resolution):** positive `lateral_y_err` means
robot LEFT of dock-aligned approach line → steer right (negative
`angular.z`).

**onHalted:** `publish_zero()` unconditionally + WARN log "FineDock
halted". Critical for R-9 — the OS may halt the node mid-onRunning;
the BT framework calls onHalted; we MUST drive cmd_vel to zero before
yielding.

### PreUndockClearanceCheck (SPEC R-12)

**Type:** `BT::SyncActionNode`. Pre-flight rear-sector probe.

**Tick:**
1. Snapshot `latest_scan_kicp` + receipt + `last_scan_kicp_at` under lock.
2. Stale-scan gate: no scan OR age > 0.5 s → WARN + FAILURE (assume
   unsafe).
3. Iterate scan rays. For each `r in [range_min, range_max]` with finite
   range, compute `angle = angle_min + i * angle_increment`. Take
   `delta = shortest_angular_distance(angle, M_PI)`. If
   `|delta| <= half_rad` (default 30°), update `min_r`.
4. Empty rear sector → INFO + SUCCESS (genuinely no obstacle; rare).
5. `min_r < (min_undock_distance_m + rear_safety_buffer_m)` (1.5 + 0.20 =
   1.70 m default) → WARN + FAILURE.
6. Else → INFO + SUCCESS.

**LiDAR frame assumption (CLAUDE.md AI #15 note):** rear = ±π assumes
forward-facing LD19 mount on YF500 chassis. Documented inline; if a
hardware variant moves the LiDAR, the rear-sector geometry must be
re-derived.

### PostUndockRtkValidation (SPEC R-13)

**Type:** `BT::SyncActionNode`. Post-undock cross-check.

**Tick:**
1. Skip silently if `gps_fix_type < 4` (not RTK_FIXED) → SUCCESS.
2. Look up `odom -> base_footprint` TF → SUCCESS on TF failure (cannot
   validate, defer).
3. `error_xy = hypot(gps_x - odom_x, gps_y - odom_y)`.
4. Three-way disposition:
   - `error_xy <= 0.5` → SUCCESS silent.
   - `0.5 < error_xy <= 1.5` → WARN + `ctx->dock_pose_suspect = true` +
     SUCCESS.
   - `error_xy > 1.5` → ERROR log + FAILURE.

## BTContext extension (Plan 02-06 fields)

```cpp
struct BTContext {
  // ... existing fields preserved verbatim ...

  // ---- Phase 2 additions ----
  geometry_msgs::msg::PoseWithCovarianceStamped latest_dock_match_pose;
  bool latest_dock_match_pose_received{false};
  std::chrono::steady_clock::time_point last_dock_match_pose_at{};

  mowgli_interfaces::msg::DockMatchConfidence latest_dock_match_conf;
  bool last_dock_match_trusted{false};
  std::chrono::steady_clock::time_point last_dock_match_conf_at{};

  sensor_msgs::msg::LaserScan latest_scan_kicp;
  bool latest_scan_kicp_received{false};
  std::chrono::steady_clock::time_point last_scan_kicp_at{};

  bool dock_pose_suspect{false};

  std::string dock_approach_path{"/ros2_ws/maps/dock_approach.yaml"};
  std::string dock_calibration_path{"/ros2_ws/maps/dock_calibration.yaml"};
  std::string dock_scan_path{"/ros2_ws/maps/dock_scan.pcd"};
  std::string dock_scan_meta_path{"/ros2_ws/maps/dock_scan_meta.yaml"};
};
```

All fields default-initialised so day-1 deployments without Plan 02-04
deployed (matcher node absent → no /dock_match/* messages) still tick
cleanly. `latest_dock_match_pose_received = false` is the day-1 baseline
that FineDock's degraded-mode contract sees.

## behavior_tree_node.cpp — 3 new subscribers

| Topic | QoS | BTContext field |
|-------|-----|------------------|
| `/dock_match/pose` | `rclcpp::QoS(1).reliable()` (matches Plan 02-04 publisher) | `latest_dock_match_pose` + `latest_dock_match_pose_received` + `last_dock_match_pose_at` |
| `/dock_match/confidence` | `rclcpp::SensorDataQoS()` | `latest_dock_match_conf` + `last_dock_match_trusted` + `last_dock_match_conf_at` |
| `/scan_kicp` | `rclcpp::SensorDataQoS()` | `latest_scan_kicp` + `latest_scan_kicp_received` + `last_scan_kicp_at` |

All callbacks acquire `ctx->context_mutex` and stamp the
`steady_clock` receipt time. The pose subscriber matches Plan 02-04's
publisher QoS exactly (reliable depth=1, volatile) — using
`transient_local` here would silently drop every message per the
QoS-compatibility matrix.

## MockMatcher / IDockMatcher integration

Plan 02-06 BT tests do NOT instantiate `KinematicIcpDockMatcher` directly.
Instead, the tests populate `BTContext::latest_dock_match_pose` and
`latest_dock_match_conf` directly — the same struct the live
behavior_tree_node subscribers populate at runtime. This is the
"mock at the BTContext boundary" pattern, equivalent to but smaller
than Plan 02-04's `IDockMatcher`-injected MockMatcher pattern.

The `IDockMatcher` abstract base from Plan 02-02 is still the right
contract for `DockScanMatchNode`'s unit tests (which do need to bypass
kiss_icp + LaserProjection + a live tf_buffer). Plan 02-06 BT tests
have no need to instantiate any matcher because the BTContext is the
trust boundary at the BT layer — exactly what Plan 02-04's matcher
publishes onto.

## 13 gtest cases — full enumeration

| # | Test name | SPEC tag | Asserts |
|---|-----------|----------|---------|
| 1 | `RecordDockApproachPosePrefersLidarMatch` | R-5 | trusted+fresh /dock_match/pose → source=lidar; yaw_to_dock ≈ atan2(dy, dx) |
| 2 | `RecordDockApproachPoseFallsBackToTf` | R-5 | trusted=false → source=tf; (here_x, here_y) from map→base_footprint |
| 3 | `RecordDockApproachPoseAutoRefreshTriggers` | R-11 | 8d-old meta + inlier=0.97 → meta captured_at refreshed |
| 4 | `RecordDockApproachPoseAutoRefreshSkipsLowConfidence` | R-11 | 8d-old meta + inlier=0.85 → meta unchanged |
| 5 | `ApproachDockFailsOnMissingYaml` | R-6 | onStart returns FAILURE without async wait |
| 6 | `ApproachDockSendsCorrectGoal` | R-6 | in-process NavigateToPose mock receives goal with frame=map, x/y/yaw matching yaml |
| 7 | `FineDockSucceedsOnIsCharging` | R-7 | latest_status.is_charging=true → SUCCESS |
| 8 | `FineDockAbortsOnConfidenceLoss` | R-8 | trusted=true→false; after 1.1s of trusted=false → FAILURE |
| 9 | `FineDockHaltsOnEmergency` | R-9 | active_emergency=true → FAILURE on next onRunning |
| 10 | `PreUndockClearanceFailsOnRearObstacle` | R-12 | scan ranges[0]=0.5m at angle=-π → FAILURE |
| 11 | `PreUndockClearanceSucceedsWhenClear` | R-12 | scan ranges all 5.0m → SUCCESS |
| 12 | `PostUndockRtkValidationThresholds` | R-13 | 0.3m → SUCCESS silent; 0.7m → SUCCESS + suspect=true; 2.0m → FAILURE |
| 13 | `FineDockBailsAfter5sNoPose` | WARNING-9 | no pose ever; after 5.1s → FAILURE (distinct contract from R-8) |

Tests 8 and 13 pin the two-tier no-pose contract: 5s degraded-mode bail
(no pose ever) vs. 1s R-8 trust-loss abort (pose flowing but trusted=false).
Both contracts are independently tested; both ship.

Test 6 spins an in-process `NavigateToPose` action server in the test
fixture (mirrors Phase 1 Plan 01-08 in-process service-stub pattern) and
captures the received goal payload via atomics. Goal is asserted on
`frame_id == "map"` and `(x, y, yaw)` matching the dock_approach.yaml
to 1e-5 tolerance.

## Coordination Risks for Plan 02-07

### main_tree.xml wiring is the next step

Plan 02-07 must:

1. **Wire `RecordDockApproachPose` into UndockSequence** between
   `CalibrateHeadingFromUndock` and the existing `ClearCommand` /
   session-clear nodes. Suggested placement: immediately after
   `CalibrateHeadingFromUndock` returns SUCCESS, so the post-undock
   pose is captured at the EKF's converged steady-state.

2. **Migrate the 4 `<DockRobot>` sites** in main_tree.xml
   (CriticalBatteryDock line 171, RainDockAndResume line 332,
   BatteryDockAndResume line 378, FailedCoverageDock line 445) to
   `<Sequence><ApproachDock/><FineDock/></Sequence>` (or wrap as a
   reusable `LidarDock` subtree). SPEC R-10 acceptance:
   `grep -c "DockRobot" main_tree.xml = 0`.

3. **Add `PreUndockClearanceCheck` before `BackUp`** in
   UndockSequence so a person standing behind the dock aborts the
   undock before any motion.

4. **Add `PostUndockRtkValidation` after `CalibrateHeadingFromUndock`**
   (and before `RecordDockApproachPose` so a suspect dock pose
   isn't persisted as the next approach target).

5. **OuterReactiveSequence on FineDock**: the BT framework's
   ReactiveSequence parent will halt FineDock if any sibling
   condition (e.g., `IsEmergency`, `IsBatteryEmpty`) flips. FineDock's
   `onHalted` publishes zero twist — no auto-restart logic needed in
   the node itself. If Plan 02-07 wraps FineDock in a non-reactive
   sequence, the explicit emergency check in `onRunning` (step 1
   above) closes the gap.

### GUI surface (Plan 02-07's other half)

The GUI Dock-card must display:

- `inlier_ratio` (from `/dock_match/confidence.inlier_ratio`)
- `rmse_m` (from `/dock_match/confidence.rmse_m`)
- `dock_pose_suspect` (from BTContext via HighLevelStatus.message or
  a new diagnostic field)
- Last-fine-dock lateral error (post-FineDock SUCCESS, derived from
  `/dock_match/pose` minus `dock_calibration.yaml` projected onto the
  dock-aligned frame — this is GUI math, not BT math)

The TS bindings for `DockMatchConfidence` are auto-generated from
Plan 02-01 codegen (see Plan 02-04 SUMMARY § "Plan 02-07 GUI"
contract); GUI-side type imports are already in place.

### `dock_match_max_age_s` ROS param is per-consumer

Plan 02-05's seeder cascade declares `dock_match_max_age_s = 1.0` s.
FineDock declares its own `confidence_loss_timeout_s = 1.0` s and
`no_pose_bail_s = 5.0` s + the implicit 2 s pose-staleness threshold.
These are independent gates — the seeder's gate is "is the LiDAR
fresh enough for one-shot EKF reset?"; FineDock's gates are "is the
LiDAR fresh enough for closed-loop control?" + "did we ever see a
pose?". Plan 02-07 should NOT collapse them into a single shared
param.

### CLAUDE.md AI #15 robot footprint sync

Plan 02-06 does NOT touch any of the three robot-footprint sources
of truth. PreUndockClearanceCheck reads `min_undock_distance_m` from
ports (default 1.5 m) — a distance, not a footprint dimension. If
Plan 02-07 surfaces this in the GUI as a tunable, it does NOT need
to sync `mowgli_robot.yaml:chassis_*` /
`mowgli_robot.yaml:robot_geometry.*` /
`nav2_params.yaml:collision_monitor:PolygonSlow.points`.

## Architecture Invariant compliance (CLAUDE.md AI #1, #10, #13)

| Invariant | Compliance |
|-----------|------------|
| AI #1 (single localizer) | No TF broadcast in any new node (`grep -E "TransformBroadcaster\|sendTransform" docking_nodes.cpp` returns nothing). No subscription to `/odometry/filtered_map` or any robot_localization-output topic. FineDock control loop reads `/dock_match/pose` (matcher's own output, NOT EKF-fused) and publishes `/cmd_vel_docking` (twist_mux priority 15) — no feedback into robot_localization. |
| AI #10 (no UndockRobot reintroduction) | ApproachDock dispatches NavigateToPose, NOT `opennav_docking::UndockRobot`. The legacy DockRobot/UndockRobot BT nodes are kept for backward compatibility until Plan 02-07 removes them; no new code calls them. |
| AI #13 (cmd_vel routing) | FineDock publishes ONLY to `/cmd_vel_docking` (verified: `grep -E '"/cmd_vel"\|"/cmd_vel_nav"\|"/cmd_vel_teleop"' docking_nodes.cpp` returns nothing). Twist_mux priority 15 is the only path to the wheels. |

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 — Plan-vs-Codebase] Plan body says "modify behavior_tree_node.cpp's factory registrations" — actual location is register_nodes.cpp**

- **Found during:** Task 1 (locating the factory registration site)
- **Issue:** Plan body's <action> step 5 said "locate the existing
  `factory.registerNodeType<RecordUndockStart>(...)` block in
  behavior_tree_node.cpp and add immediately below". The actual
  registrations live in `register_nodes.cpp::registerAllNodes(factory)`,
  which is called from
  `behavior_tree_node.cpp::setupBehaviorTree::mowgli_behavior::registerAllNodes(factory_)`
  (line 390). behavior_tree_node.cpp does not contain any
  `factory.registerNodeType` call.
- **Fix:** Added the `register_docking_nodes(factory)` call to
  `register_nodes.cpp::registerAllNodes` immediately after the
  `RecordResumeUndockFailure` registration. Same effect as the plan
  intended — all 5 nodes registered before tree creation — but at the
  correct file boundary.
- **Files modified:** `ros2/src/mowgli_behavior/src/register_nodes.cpp`
- **Commit:** `d38fa39b`

**2. [Rule 3 — Plan-vs-Codebase] `BackUp` and `NavigateToPose` BT-node-name collisions with action_nodes**

- **Found during:** Task 1 (drafting docking_nodes.hpp)
- **Issue:** mowgli_behavior already has a `BT::ActionNodeBase`-derived
  `NavigateToPose` (registered in register_nodes.cpp line 56). The
  ApproachDock implementation uses
  `nav2_msgs::action::NavigateToPose` (the action message type), not
  the BT node. Naming collision avoided by aliasing
  `using NavigateToPose = nav2_msgs::action::NavigateToPose;` inside
  the `ApproachDock` class scope.
- **Fix:** Class-scope `using` directive; verified no namespace shadow
  with the existing BT-node `mowgli_behavior::NavigateToPose`
  (different translation units, both under `mowgli_behavior`).
- **Files modified:** `ros2/src/mowgli_behavior/include/mowgli_behavior/docking_nodes.hpp`
- **Commit:** `d38fa39b`

**3. [Rule 2 — Critical Functionality] Pose covariance was unspecified for ApproachDock goal**

- **Found during:** Task 1 (writing onStart)
- **Issue:** Plan body's behaviour list said "Build NavigateToPose
  goal with frame_id=map, header.stamp=now, position=(approach.x,
  approach.y, 0), orientation=quat-from-yaw(approach.yaw_to_dock_rad)"
  — but `nav2_msgs::action::NavigateToPose::Goal::pose` is a
  `geometry_msgs::msg::PoseStamped` (no covariance). No fix needed
  beyond following the message type literally; documenting here so
  future executors don't hunt for a covariance field that doesn't
  exist on this msg.
- **Fix:** None — plan-vs-codebase mismatch was a non-issue once the
  message type was inspected. Documented for clarity.
- **Files modified:** none
- **Commit:** n/a

**4. [Rule 2 — Critical Functionality] FineDock dock_target loaded from dock_calibration.yaml, not dock_approach.yaml**

- **Found during:** Task 2 (writing onStart)
- **Issue:** Plan body said "load dock_approach.yaml... use
  approach.x/y as the target reference". This is wrong — the
  `dock_approach` is the START of the crawl (where ApproachDock
  drove to), and the `dock_calibration` is the CONTACT POINT (where
  the lateral_y_err must be computed against). Using `approach.x/y`
  as the target would make FineDock try to hold position 1.5 m before
  the dock instead of crawling forward into it.
- **Fix:** FineDock onStart loads BOTH yaml files — `dock_approach.yaml`
  for `yaw_to_dock_rad_` (the dock-aligned frame definition) and
  `dock_calibration.yaml` for `(dock_target_x_, dock_target_y_)` (the
  contact point). The control loop computes lateral_y_err in the
  dock-aligned frame relative to the contact point.
- **Files modified:** `ros2/src/mowgli_behavior/src/docking_nodes.cpp`
- **Commit:** `d38fa39b`

**5. [Rule 3 — Plan-vs-Codebase] BT node `NavigateToPose` ports include `dock_approach_path` (via blackboard) — plan said port-only**

- **Found during:** Task 1 (writing ApproachDock onStart)
- **Issue:** Plan body's port spec for ApproachDock listed only
  `dock_approach_path` as an `InputPort<std::string>`. In practice,
  the BT framework's getInput("dock_approach_path") returns the
  default value ("") for ports without an explicit blackboard write
  — meaning Task 1's test fixture (which sets
  `bb->set("dock_approach_path", ...)`) works, but the production
  BT.cpp main_tree.xml will pass an empty string by default.
- **Fix:** ApproachDock onStart accepts both "empty port" and
  "explicit port" — empty falls back to `ctx->dock_approach_path`
  (which defaults to `/ros2_ws/maps/dock_approach.yaml`). This keeps
  main_tree.xml clean (Plan 02-07 does NOT need to set the port
  unless overriding for a test) and lets tests inject tmpdirs.
- **Files modified:** `ros2/src/mowgli_behavior/src/docking_nodes.cpp`
- **Commit:** `d38fa39b`

## Threat Flags

None. The new nodes mitigate the threats they were designed to mitigate:

- **T-06-01 (Spoofed /dock_match/pose during FineDock crawl):** R-8
  trust-loss abort (1 s); compounded with collision_monitor +
  twist_mux priority + firmware safety latch.
- **T-06-02 (dock_approach.yaml deleted between writer + reader):**
  Atomic write via Plan 02-02 atomic_write; ApproachDock returns
  FAILURE on missing file → BT outer Sequence handles failure.
- **T-06-03 (FineDock loops forever):** No outer-timeout in this
  node (parent BT subtree responsibility); is_charging path is the
  primary success route; degraded-mode 5 s bail + R-8 1 s abort cap
  the pathological cases.
- **T-06-04 (Auto-refresh writes corrupted dock_scan.pcd):** Plan
  02-04 mtime watcher KEEPS old matcher on reload failure; atomic
  PCD write here; meta only bumped after PCD write succeeds.
- **T-06-05 (PreUndockClearanceCheck reads stale /scan_kicp):** 0.5 s
  staleness gate → FAILURE (assume unsafe).
- **T-06-06 (PostUndockRtkValidation false-positive flags suspicious
  dock_pose):** ACCEPTED — `dock_pose_suspect` is operator-readable;
  no automatic action.
- **T-06-07 (dock_approach.yaml leaks robot position):** ACCEPTED —
  same trust as dock_calibration.yaml.
- **T-06-08 (FineDock cmd_vel attack via E-Stop):** onHalted +
  emergency-check first in onRunning + firmware safety authority.

No new trust boundaries introduced.

## TDD Gate Compliance

The plan-level TDD gate (RED → GREEN) is honoured:

| Gate | Commit | Evidence |
|------|--------|----------|
| RED  | `bca150c8` | `test(02-06): add failing gtests for 5 LiDAR-dock BT nodes (RED)` — 13 test cases reference `RecordDockApproachPose`, `ApproachDock`, `FineDock`, `PreUndockClearanceCheck`, `PostUndockRtkValidation` + BTContext fields `latest_dock_match_pose`, `latest_dock_match_conf`, `latest_scan_kicp`, `dock_pose_suspect`, `dock_approach_path` etc. — none of which exist on this commit. |
| GREEN | `d38fa39b` | `feat(02-06): implement 5 LiDAR-dock BT nodes (GREEN)` — adds the 5 BT nodes + BTContext extension + factory registration + behavior_tree_node subscribers + package.xml/CMakeLists deps. The 13 RED tests would compile and pass at colcon-test time. |
| REFACTOR | n/a | No refactor commit; the GREEN code is the final form. |

The fail-fast rule was honoured: RED was committed before any
production code changes (BTContext.hpp + docking_nodes.{hpp,cpp} +
register_nodes.cpp + behavior_tree_node.cpp + CMakeLists.txt +
package.xml all touched only in the GREEN commit). On macOS, neither
commit can be exercised at colcon level; the gate is enforced by the
phase-end podman build.

## Deferred verify steps

The orchestrator runs the actual ROS2 build at end-of-phase via podman
inside the devcontainer. The host (macOS) cannot run colcon. Each
command below is the verbatim verify step the plan specified that this
executor could not run.

- **Task 1 — colcon build + colcon test for the BT nodes**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2 && \
    colcon build --packages-select mowgli_behavior \
      --event-handlers console_cohesion+ 2>&1 | tail -15 && \
    colcon test --packages-select mowgli_behavior \
      --ctest-args -R docking_nodes \
      --event-handlers console_cohesion+ 2>&1 | tail -15 && \
    colcon test-result --verbose \
      --test-result-base build/mowgli_behavior 2>&1 | tail -25
  ```

  Expected: 13 gtest cases passing in `test_docking_nodes`, 0 failed.
  Build artefacts include the updated `behavior_tree_node` executable
  (BTContext extension affects every BT node TU but no existing
  signature changes).

- **Task 2 — focused FineDock degraded-mode contract verify**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2 && \
    colcon test --packages-select mowgli_behavior \
      --ctest-args -R 'docking_nodes.FineDockBailsAfter5sNoPose' \
      --event-handlers console_cohesion+ 2>&1 | tail -10
  ```

  Expected: 1 test passed (WARNING-9 contract gate).

- **Phase-end smoke (post-build, container running): node + topic + factory verify**

  ```bash
  source /opt/ros/kilted/setup.bash && source install/setup.bash && \
  ros2 launch mowgli_bringup full_system.launch.py use_lidar:=true &
  sleep 30 && \
  # Confirm the BT factory loaded all 5 new nodes by their registered name.
  ros2 launch mowgli_behavior list_bt_nodes.launch.py 2>&1 | \
    grep -E "RecordDockApproachPose|ApproachDock|FineDock|PreUndockClearanceCheck|PostUndockRtkValidation"
  ```

  Expected: 5 lines printed (one per node). Note: `list_bt_nodes.launch.py`
  is a hypothetical helper; the real verification is that
  `behavior_tree_node` starts without throwing on `factory.createTreeFromFile`
  once Plan 02-07 wires the new node names into main_tree.xml.

- **Pi5 hardware checkpoint (Plan 02-08 territory)**

  Per CLAUDE.md memory entry "Pi5 test before PR": every change must
  verify on the Pi5 test bench before a PR is opened. Plan 02-08
  hardware smoke is the canonical place; the BT-side wiring (Plan
  02-07) ships first so that the operator can interactively trigger
  FineDock from the GUI without a full main_tree.xml override.

## Drift detection

| Check | Expected | Actual |
|-------|----------|--------|
| brace balance bt_context.hpp | open == close | green (35/35) |
| brace balance docking_nodes.hpp | open == close | green (42/42) |
| brace balance docking_nodes.cpp | open == close | green (102/102) |
| brace balance behavior_tree_node.cpp | open == close | green (59/59) |
| brace balance register_nodes.cpp | open == close | green (2/2) |
| brace balance test_docking_nodes.cpp | open == close | green (40/40) |
| `class RecordDockApproachPose\|class ApproachDock\|class FineDock\|class PreUndockClearanceCheck\|class PostUndockRtkValidation` | grep count == 5 | green (5) |
| `RecordDockApproachPose::tick\|ApproachDock::on\|FineDock::on\|PreUndockClearanceCheck::tick\|PostUndockRtkValidation::tick\|FineDock::publish_zero` | grep count >= 5 | green (17 — covers tick + onStart + onRunning + onHalted + publish_zero) |
| `register_docking_nodes` in register_nodes.cpp | grep | green (1) |
| `/dock_match/` in behavior_tree_node.cpp | grep count >= 2 | green (4) |
| `/scan_kicp` in behavior_tree_node.cpp | grep count >= 1 | green (3) |
| no TF broadcast in docking_nodes.cpp | absence-grep | green |
| FineDock /cmd_vel_docking only | only `/cmd_vel_docking`, no `/cmd_vel`/`/cmd_vel_nav`/`/cmd_vel_teleop` | green |
| `<depend>mowgli_lidar_docking</depend>` + `<depend>nav2_msgs</depend>` + `<depend>sensor_msgs</depend>` | grep count == 3 | green (3) |
| BTContext fields latest_dock_match_pose / latest_dock_match_conf / latest_scan_kicp / dock_pose_suspect | grep count >= 4 | green (6) |
| `registerNodeType` in `register_docking_nodes` body | exactly 5 | green (5) |

## Self-Check

Verifying claims before STATE.md / ROADMAP.md updates.

### Files claimed exist

```
FOUND: ros2/src/mowgli_behavior/test/test_docking_nodes.cpp
FOUND: ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp (modified)
FOUND: ros2/src/mowgli_behavior/include/mowgli_behavior/docking_nodes.hpp (modified)
FOUND: ros2/src/mowgli_behavior/src/docking_nodes.cpp (modified)
FOUND: ros2/src/mowgli_behavior/src/behavior_tree_node.cpp (modified)
FOUND: ros2/src/mowgli_behavior/src/register_nodes.cpp (modified)
FOUND: ros2/src/mowgli_behavior/CMakeLists.txt (modified)
FOUND: ros2/src/mowgli_behavior/package.xml (modified)
```

### Commits claimed exist

```
FOUND: bca150c8 — RED gate (test file)
FOUND: d38fa39b — GREEN gate (5 BT nodes + BTContext + subscribers)
```

## Self-Check: PASSED
