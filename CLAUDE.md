# MowgliNext

Open-source autonomous robot mower monorepo. ROS2 Kilted, Nav2, robot_localization (dual EKF: local wheel+gyro, global +GPS, under `two_d_mode`), optional Kinematic-ICP drift correction, BehaviorTree.CPP v4, full-path coverage planner.

**Website:** https://mowgli.garden | **Wiki:** https://github.com/cedbossneo/mowglinext/wiki | **First-boot checklist:** [`docs/FIRST_BOOT.md`](docs/FIRST_BOOT.md)

## Safety — READ FIRST

This robot has spinning blades. The STM32 firmware is the sole blade safety authority.

- NEVER bypass firmware blade safety checks from ROS2
- Blade commands from ROS2 are fire-and-forget — firmware decides whether to execute
- Emergency stop is handled by firmware, not software
- Flag ANY change that could affect physical behavior as safety-critical in PR reviews

## Monorepo Layout

| Directory | Language | Build | Description |
|-----------|----------|-------|-------------|
| `ros2/` | C++17, Python | `colcon build` | ROS2 stack: 12 packages (Nav2, robot_localization, Kinematic-ICP, BT, coverage, hardware bridge) |
| `install/` | Shell | `./mowglinext.sh` | Interactive installer, hardware presets, modular Docker Compose configs |
| `gui/` | Go, TypeScript/React | `go build`, `yarn build` | Web interface for config, map editing, monitoring |
| `docker/` | YAML, Shell | `docker compose` | Manual deployment configs, DDS, service orchestration |
| `sensors/` | Dockerfile | `docker build` | Dockerized sensor drivers (GPS, LiDAR) |
| `firmware/` | C | `pio run` | STM32F103 firmware (motor, IMU, blade, battery) |
| `docs/` | HTML, CSS, JS | GitHub Pages | Landing page + install composer at mowgli.garden |

## Architecture Invariants (DO NOT VIOLATE)

1. **robot_localization (dual EKF) is the sole localizer.** Two cooperating EKFs under `two_d_mode` (15D state with roll/pitch/Z clamped to zero). `ekf_odom_node` (25 Hz target) fuses `/wheel_odom_fused` (vx, vy, vyaw with tight `vy≈0` covariance for non-holonomic motion) + `/imu/data` (gyro_z, roll/pitch) and publishes `odom → base_footprint`. `ekf_map_node` (10 Hz) fuses the same wheel+IMU inputs plus `/gps/pose_cov` (PoseWithCovarianceStamped from `navsat_to_absolute_pose_node` with map-yaw lever-arm correction — `navsat_transform_node` was removed 2026-04-26, its `/odometry/gps` had no consumers) , `/imu/cog_heading` (absolute yaw from GPS course-over-ground by `cog_to_imu.py`, gated on forward motion + RTK-Fixed), and `/imu/mag_yaw` (tilt-compensated absolute yaw from `mag_yaw_publisher.py` reading the AltIMU-10v6 LIS3MDL after hard/soft-iron calibration — yaw-only, works at standstill and under canopy) and publishes `map → odom`. RTK-Fixed σ ~3 mm flows through directly via `NavSatFix.covariance`. When LiDAR is present, Kinematic-ICP publishes a body-frame twist on `/encoder2/odom`; `wheel_kicp_blend.py` then fuses `/wheel_odom` + `/encoder2/odom` by inverse-variance per axis and republishes `/wheel_odom_fused` (so K-ICP enters the EKF folded into `odom0`, not as `odom1` — robot_localization's `odom1` path caps the EKF publisher at 5 Hz, and 250 ms K-ICP staleness falls back to wheel-only). Kinematic-ICP runs on a **fully decoupled parallel TF tree** (`wheel_odom_raw → base_footprint_wheels → lidar_link_wheels`) so none of its inputs depend on the fused state: `wheel_odom_tf_node` integrates raw `/wheel_odom` into the parallel tree's motion prior, and `kinematic_icp_scan_frame_relay` mirrors the URDF sensor extrinsic and republishes `/scan` on `/scan_kicp` with the matching `frame_id`. Output flows one-way into the EKF via the blend node — no TF feedback. **Magnetometer pipeline restored 2026-04-29** (replaces "Decision B / mag absent" from the 2026-04-27 migration): AltIMU-10v6 (LSM6DSO + LIS3MDL) replaces the unfusable WT901 mag. `mag_yaw_publisher.py` reads `/imu/mag_raw` + `/imu/data`, applies the hard/soft-iron ellipsoid calibration written by `calibrate_imu_yaw_node` to `/ros2_ws/maps/mag_calibration.yaml`, tilt-compensates the horizontal field via `base_footprint→imu_link` TF, applies the local declination correction (Eichenau ≈ 3.5° E from NOAA WMM 2026 — override per-deploy via the ROS param), and publishes absolute yaw on `/imu/mag_yaw` as a `sensor_msgs/Imu` (orientation + cov[8] only; gyro/accel covariances flagged -1 so robot_localization ignores those channels). `ekf_map_node` fuses it as `imu2` with `imu2_config[5]=true` (yaw-only). Complementary to `imu1` (`/imu/cog_heading`): cog needs forward motion + RTK-Fixed, mag works at standstill and under canopy. The publisher sits idle (~0% CPU) until `mag_calibration.yaml` appears on disk; it polls the file every 30 s and self-activates. **`calibrate_imu_yaw_node` is rclcpp** (Decision A during 2026-04-27 migration, closes #19 foxglove_bridge ↔ rclpy ↔ cyclonedds typesupport bug); upstream's Python evolution is intentionally not consumed.
2. **TF chain follows REP-105** — `map → odom → base_footprint → base_link → sensors`. `map→odom` is published by `ekf_map_node` and `odom→base_footprint` by `ekf_odom_node`. All Nav2 nodes use `base_footprint` as the robot frame. `base_link` is at the rear wheel axis (OpenMower convention, do not move).
3. **Cyclone DDS** — not FastRTPS (stale shm issues on ARM)
4. **Map frame = GPS frame** — X=east, Y=north, no rotation transform
5. **Costmap obstacles disabled in coverage mode** — collision_monitor handles real-time avoidance
6. **dock_pose_yaw auto-captured** — `dock_yaw_to_set_pose` writes the measured dock yaw into `dock_calibration.yaml` on first charge; no phone compass step. `hardware_bridge` and `map_server` read this file at startup. Legacy `mowgli_robot.yaml:dock_pose_yaw` is still used as a fallback.
7. **Deterministic full-path coverage planner.** `coverage_planner_node` (package `mowgli_coverage_planner`) pre-plans the complete sequential `PoseStamped[]` waypoint list for every working area + obstacle outline + boustrophedon swath via `PlanCoverage.action` before any blade rotates. The plan is sparse (one pose per outline vertex / swath endpoint / dock waypoint; Nav2 `FollowPath` densifies during execution) and every waypoint carries a `segment_type` ∈ {UNDOCK, TRANSIT, OUTLINE_WORKING_AREA, OUTLINE_OBSTACLE, MOWING_BOUSTROPHEDON, RETURN_TO_DOCK, DOCK_APPROACH, DOCKING}. The BT node `PlanCoverageGoal` runs once at COMMAND_START, sends the Action goal, and writes the resulting plan to the BT blackboard. The monolithic `FollowCoveragePlan` `BT.CPP v4` `StatefulActionNode` then walks the plan sequentially, dispatching each waypoint to Nav2 `NavigateToPose` (UNDOCK / TRANSIT / DOCK_APPROACH / RETURN_TO_DOCK / DOCKING via RPP) or Nav2 `FollowPath` (OUTLINE_* and MOWING_BOUSTROPHEDON via FTCController) and toggling the blade per-segment via `mowgli_interfaces/srv/MowerControl`. Progress is checkpointed per area to `<areas_dir>/coverage_<area_index>.kv` sidecars (`key=value` text, one field per line — yaml-cpp is intentionally avoided per `hardware_bridge_node.cpp:99`) written atomically via tempfile + `rename(2)`; resume re-issues the Action goal with `resume_from_checkpoint = true` and continues at the open swath endpoint within ≤ 5 cm + 5° yaw of the persisted state. Collision avoidance during execution is `collision_monitor` + Nav2's local planner only — there is no global re-planning on dynamic obstacles, no continuous re-trigger on area edits (the plan is a snapshot per Action goal). `map_server_node` no longer plans paths; it serves area geometry via `GetAllAreas.srv` and persists the area DB. Pre-flight validation runs all 10 spec safety points before plan emission; failure returns a typed `PlanError` instead of a partial plan.
8. **FTCController for coverage paths** — RPP for transit only, FTCController (PID on 3 axes) for coverage path following
9. **Emergency auto-reset on dock** — When emergency is active and robot is on dock (charging detected), BT auto-sends `ResetEmergency` to firmware. Firmware is sole safety authority and only clears latch if physical trigger is no longer asserted.
10. **Undock via Nav2 BackUp behavior** — BackUp (1.5m, 0.15 m/s) via `behavior_server`, not `opennav_docking` UndockRobot (isDocked() unreliable with GPS drift near the dock). Costmaps are cleared after undock.
11. **Zero-odom only when charging AND idle** — `hardware_bridge_node` does not reset odometry during undock sequence.
12. **Battery current for dock detection** — Hardware bridge publishes `abs(charging_current)` when charging, `0.0` when not, for `SimpleChargingDock` compatibility.
13. **Docking server cmd_vel** — Remapped to `/cmd_vel_docking` through twist_mux (priority 15).
14. **Coverage grid_map → OccupancyGrid convention** — easy to get wrong. `grid_map::GridMap::getSize()(0)` = cells along X, `getSize()(1)` = cells along Y. grid_map iterates with `r=0 → X_max` (decreasing) and `c=0 → Y_max` (decreasing). `nav_msgs/OccupancyGrid` has `width = X_cells`, `height = Y_cells`, row-major with `data[y_row * width + x_col]`, where `col=0 ↔ origin.x` (X_min) and `row=0 ↔ origin.y` (Y_min). To convert a grid_map cell `(r, c)` to an OccupancyGrid index: `og_col = nx - 1 - r`, `og_row = ny - 1 - c`, `flat_idx = og_row * nx + og_col`. ALWAYS set `mask.info.width = nx` and `mask.info.height = ny` — swapping produces a 90°-rotated mask, which silently marks valid interior polygon cells as lethal and breaks Smac planning with "Start occupied" errors. See `mow_progress_to_occupancy_grid()` in `map_server_node.cpp` as the reference implementation.
15. **Robot footprint geometry sync (three-way manual sync, scheduled to be retired — see GH issue #67).** Three independent files are sources of truth for the same physical chassis dimension:
    1. **`mowgli_robot.yaml` `chassis_*` block** (URDF / Foxglove visualisation / simulation): `chassis_length`, `chassis_width`, `chassis_center_x`. The GUI's "Chassis & Geometry" advanced panel writes here.
    2. **`mowgli_robot.yaml` `robot_geometry:` block** (coverage_planner_node footprint validators, SPEC R-6 Phase 1): `robot_length`, `robot_width`, `drive_axis_x_offset` (= `-chassis_center_x`), `blade_x_offset`, `tool_width`. The redundancy with `chassis_*` is historical — pre-existing footgun.
    3. **`nav2_params.yaml` `collision_monitor:PolygonSlow.points`** (live collision-monitor slow zone, evaluated at runtime against the laser scan). Currently a hard-coded polygon string: chassis ±15 cm slow margin. There is no native footprint-topic subscribe enabled in this config, even though `nav2_collision_monitor` supports `type: polygon_topic` natively (see #67 backlog item). Plus the dormant `coverage_server.robot_width` block which is unused but should still match.
    
    **Always commit changes to all three in the same commit** (Architecture Invariant #15 — currently the only mechanical safety net). Tested baseline (YF500 stock spec): `0.60 × 0.40`, `drive_axis_x_offset=-0.20`. Live measured (this fork's bench, 2026-04-29): `0.57 × 0.43`, `drive_axis_x_offset=-0.18`. Your unit may differ — measure with a caliper from the drive-axle centerline. `blade_x_offset: 0.25`, `tool_width: 0.18` are blade geometry (same on all YF500 chassis variants).

## High-Level Commands and States

### HighLevelControl.srv Commands
| Value | Constant | Description |
|-------|----------|-------------|
| 1 | `COMMAND_START` | Begin autonomous mowing |
| 2 | `COMMAND_HOME` | Return to dock |
| 3 | `COMMAND_RECORD_AREA` | Start area boundary recording |
| 4 | `COMMAND_S2` | Mow next area |
| 5 | `COMMAND_RECORD_FINISH` | Finish recording, save polygon |
| 6 | `COMMAND_RECORD_CANCEL` | Cancel recording, discard trajectory |
| 7 | `COMMAND_MANUAL_MOW` | Enter manual mowing mode (teleop + blade) |
| 254 | `COMMAND_RESET_EMERGENCY` | Reset latched emergency |
| 255 | `COMMAND_DELETE_MAPS` | Delete all maps |

### HighLevelStatus.msg States
| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `HIGH_LEVEL_STATE_NULL` | Emergency or transitional |
| 1 | `HIGH_LEVEL_STATE_IDLE` | Idle, docked, charging, returning home |
| 2 | `HIGH_LEVEL_STATE_AUTONOMOUS` | Autonomous mowing (undocking, transit, mowing, recovering) |
| 3 | `HIGH_LEVEL_STATE_RECORDING` | Area recording in progress |
| 4 | `HIGH_LEVEL_STATE_MANUAL_MOWING` | Manual mowing via teleop |

### Area Recording Flow
1. GUI sends `COMMAND_RECORD_AREA` (3) to start recording
2. BT enters `RecordArea` node — records position at 2 Hz, publishes live preview on `~/recording_trajectory`
3. User drives robot along boundary
4. GUI sends `COMMAND_RECORD_FINISH` (5) — trajectory is simplified (Douglas-Peucker) and saved via `/map_server_node/add_area`
5. Or GUI sends `COMMAND_RECORD_CANCEL` (6) — trajectory discarded

### Manual Mowing
- Dedicated BT state with `COMMAND_MANUAL_MOW` (7) — does not hijack recording mode
- Teleop via `/cmd_vel_teleop` (twist_mux priority)
- Blade managed by GUI (fire-and-forget to firmware)
- Collision_monitor, GPS, robot_localization, Kinematic-ICP (if enabled) all remain active

## Code Style

| Component | Style | Tool |
|-----------|-------|------|
| C++ (ros2/) | 2-space indent, `snake_case` files/params, `CamelCase` classes | `clang-format` (config in `ros2/.clang-format`) |
| Go (gui/) | Standard Go | `gofmt` |
| TypeScript (gui/web/) | Prettier + ESLint | `yarn lint` |
| Python (launch files) | PEP 8 | — |
| YAML (config) | 2-space indent, `snake_case` keys | — |

## Commit Conventions

```
<type>: <description>

Types: feat, fix, refactor, docs, test, chore, perf, ci
```

No Co-Authored-By lines. Keep messages concise and focused on "why".

## ROS2 Specifics

- **Distro:** Kilted
- **DDS:** Cyclone DDS (all containers share `docker/config/cyclonedds.xml`)
- **Topics:** Mowgli-specific topics under `/mowgli/` namespace
- **Frames:** `map` (global, GPS-anchored via fixed datum), `odom` (continuous local, dead-reckoning only — never jumps), `base_footprint` (robot frame for Nav2), `base_link` (rear axle), `lidar_link`, `imu_link`
- **TF chain:** `map→odom` (`ekf_map_node`, 30 Hz — absorbs GPS corrections), `odom→base_footprint` (`ekf_odom_node`, 50 Hz — continuous dead-reckoning), `base_footprint→base_link` (static), `base_link→sensors` (static — `base_link→imu_link` rotation = `imu_yaw/pitch/roll` from `mowgli_robot.yaml`, auto-calibratable via GUI button)
- **Units:** SI throughout (metres, radians, seconds)
- **Sensor fusion:** robot_localization dual EKF under `two_d_mode` (15D state, Z forced to 0). `ekf_odom_node` (25 Hz target, wheel + gyro_z, publishes `odom→base_footprint`) consumes `/wheel_odom_fused` — pre-blended wheel + K-ICP twist via `wheel_kicp_blend.py` (inverse-variance per axis, 250 ms K-ICP staleness fallback to wheel-only). `ekf_map_node` (10 Hz, wheel + gyro + `/gps/pose_cov` + `/imu/cog_heading`, publishes `map→odom`) corrects map-frame drift. `navsat_transform_node` was removed 2026-04-26 — `/gps/pose_cov` is published directly by `navsat_to_absolute_pose_node` from `/gps/fix` with map-yaw lever-arm correction. Config in `ros2/src/mowgli_bringup/config/robot_localization.yaml`. Non-holonomic motion enforced by tight `vy` covariance in `/wheel_odom`. Absolute yaw comes from three sources fused into `ekf_map_node` only (never `ekf_odom_node` — odom must stay continuous): `cog_to_imu.py` (GPS course-over-ground gated on forward motion + RTK-Fixed, fused as `imu1`), `mag_yaw_publisher.py` (tilt-compensated LIS3MDL yaw with hard/soft-iron calibration + local declination, fused as `imu2`, idle until `mag_calibration.yaml` is written by `calibrate_imu_yaw_node`), and `dock_yaw_to_set_pose.py` (seeds both EKFs via `/set_pose` on the charging rising edge from `dock_calibration.yaml`, priority over `/gnss/heading`). cog and mag are complementary: cog needs forward motion, mag works at standstill and under canopy. **Magnetometer pipeline restored 2026-04-29** with the AltIMU-10v6 (LSM6DSO + LIS3MDL) replacing the unfusable WT901 — see Architecture Invariant #1. The `hardware_bridge_node` runs a 20 s IMU bias calibration (`imu_cal_samples: 1000`) every time the robot docks, and logs the implied mounting pitch/roll so the operator can promote any >1° offset into `mowgli_robot.yaml` → `imu_pitch/imu_roll`.
- **Navigation:** RPP for transit, FTCController (Follow-the-Carrot with 3-axis PID) for coverage paths (NOT MPPI — it jumps between adjacent swaths)
- **Coverage:** Deterministic full-path planner in `coverage_planner_node` (Architecture Invariant #7). `PlanCoverage.action` returns a sparse `CoverageWaypoint[]` covering all working areas + obstacle outlines + return-to-dock; BT walks the plan sequentially via `FollowCoveragePlan`. Per-area `coverage_<area_index>.kv` sidecars track progress for charge-cycle resume. `map_server_node` no longer plans paths; it serves area geometry via `GetAllAreas.srv` only.
- **Area Recording:** `RecordArea` BT node records trajectory at 2 Hz, Douglas-Peucker simplification, saves polygon via `/map_server_node/add_area`. Live preview on `~/recording_trajectory`.
- **Manual Mowing:** Dedicated BT state (COMMAND_MANUAL_MOW=7). Teleop via `/cmd_vel_teleop`, blade managed by GUI. Collision_monitor, GPS, robot_localization remain active.
- **Emergency Auto-Reset:** BT auto-resets emergency when robot placed on dock (charging detected). Firmware is safety authority.
- **GPS fusion:** `navsat_transform_node` consumes `/gps/fix` directly and emits `/odometry/gps` in the map frame; `ekf_map_node` fuses the parallel `/gps/pose_cov` (built by `navsat_to_absolute_pose_node` from the same NavSatFix with a covariance derived from `position_accuracy`). `/gps/absolute_pose` remains exposed for the GUI and BT. With RTK-Fixed (σ ~3 mm) and frequent updates, the EKF converges to fix precision without special-case outlier gating.
- **No continuous SLAM.** The `map` frame comes from `navsat_transform` + GPS, not from SLAM. There is no Cartographer, no slam_toolbox, no pose-graph optimization. The `/map` OccupancyGrid is published by `mowgli_map/map_server_node` from user-defined area polygons (not from a SLAM backend) and persisted with the area DB.
- **Kinematic-ICP (optional drift correction):** Gated on `use_lidar`. Kinematic-ICP (PRBonn, 2024 — same team as KISS-ICP) runs on a parallel TF tree (`wheel_odom_raw → base_footprint_wheels → lidar_link_wheels`) that is fully decoupled from the robot_localization fused state, so K-ICP's motion prior can never depend on its own earlier output. Three mowgli_localization helper nodes make this work: `wheel_odom_tf_node` integrates raw `/wheel_odom` twist into the parallel motion-prior TF, `kinematic_icp_scan_frame_relay` mirrors the URDF sensor extrinsic onto the parallel tree (TF allows only one parent per frame, so we republish a second lidar frame) and republishes `/scan` as `/scan_kicp` with the matching frame_id, and `kinematic_icp_encoder_adapter` finite-differences K-ICP's Odometry pose into a body-frame twist on `/encoder2/odom` which feeds `ekf_odom_node` as `odom1`. K-ICP's kinematic prior enforces non-holonomic motion, closing the lateral-drift hole that made plain KISS-ICP hallucinate sideways motion on featureless grass. This shores up dead-reckoning during GPS degradation (tree cover, multipath); when RTK is healthy the GPS update dominates.
- **IMU mounting calibration:** `base_link→imu_link` rotation (imu_roll, imu_pitch, imu_yaw in mowgli_robot.yaml) is critical — if wrong, gravity-removal leaks into pitch and yaw integration degrades. Use the GUI's "Auto-calibrate" button next to IMU Yaw — the robot drives itself ~0.6 m forward then back and solves `imu_yaw = atan2(-ay_chip, ax_chip)` from accel direction vs wheel-derived `a_body`.
- **Nav2 tuning:** Global costmap 30m x 30m rolling window; keepout_filter disabled in global costmap (blocks transit/docking); collision_monitor PolygonStop min_points=8, PolygonSlow min_points=6; source_timeout 5.0s (ARM TF jitter); progress checker 0.15m required movement, 30s timeout; failure_tolerance 1.0; speeds: mowing 0.3/0.15 m/s, transit 0.2 m/s, max 0.3 m/s.
- **Joystick:** Foxglove client passes `schemaName` in `clientAdvertise` for JSON-to-CDR conversion. GUI shows joystick during "RECORDING" state (not just "AREA_RECORDING").

See sections below for detailed package descriptions, topics, and architecture.

## Git Workflow

**NEVER commit directly to main.** Always use feature branches and PRs:
```bash
git checkout main && git pull
git checkout -b feat/my-feature    # or fix/, refactor/, test/, chore/, docs/
# ... make changes ...
git add <files> && git commit -m "feat: description"
gh pr create --title "feat: my feature" --body "..."
```

### Dev Branch Workflow

Docker builds trigger on both `main` and `dev` branches. Images are tagged `:main` and `:dev` respectively. Use `mowgli-dev` / `mowgli-main` commands to switch between environments. Iterate on `dev`, merge to `main` when stable.

## Quick Commands

All ROS2 commands assume you are inside the devcontainer.

```bash
# Build ROS2 workspace
cd ros2 && make build

# Build a single package
cd ros2 && make build-pkg PKG=mowgli_behavior

# Run headless simulation
cd ros2 && make sim

# Run E2E tests (simulation must be running in another terminal)
cd ros2 && make e2e-test

# Format C++ code
cd ros2 && make format

# Run unit tests
cd ros2 && make test

# Build firmware
cd firmware/stm32/ros_usbnode && pio run

# GUI development
cd gui && go build -o openmower-gui && cd web && yarn dev

# --- Code generation (run after changing .msg/.srv files) ---

# Regenerate firmware rosserial C++ headers from ROS2 .msg files
python3 firmware/scripts/sync_ros_lib.py          # writes to firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/
python3 firmware/scripts/sync_ros_lib.py --check   # diff-only, no writes (CI)

# Regenerate Go message/service structs from ROS2 .msg/.srv files
cd gui && ./generate_go_msgs.sh                    # writes to gui/pkg/msgs/*/types_generated.go

# Regenerate TypeScript ROS types (snake_case fields matching rosbridge JSON)
cd gui && ./generate_ts_types.sh                   # writes to gui/web/src/types/ros.ts
```

### Code Generation Workflow

When you modify `ros2/src/mowgli_interfaces/msg/*.msg` or `srv/*.srv`:
1. **Firmware headers:** `python3 firmware/scripts/sync_ros_lib.py` — regenerates rosserial C++ headers
2. **Go types:** `cd gui && ./generate_go_msgs.sh` — regenerates Go structs with JSON tags for rosbridge
3. **TypeScript types:** `cd gui && ./generate_ts_types.sh` — regenerates `gui/web/src/types/ros.ts` with snake_case fields matching rosbridge JSON
4. **Protocol constants:** Update `HL_MODE_*` defines in `firmware/stm32/ros_usbnode/include/mowgli_protocol.h` AND `ros2/src/mowgli_hardware/firmware/mowgli_protocol.h` (these are manually maintained — keep in sync with `HighLevelStatus.msg`)

Do NOT hand-edit `*_generated.go`, `ros_lib/mower_msgs/*.h`, or `gui/web/src/types/ros.ts` — re-run the scripts instead.

## Mowing Session Monitoring

**Whenever a mowing test is run (COMMAND_START, undock, autonomous motion, or any tuning session that involves the robot moving), also run the session monitor in parallel.** Output is a JSONL timeline that can be diffed/plotted across sessions to see how tuning changes affect behavior.

```bash
# Detached background from the host (writes to /ros2_ws/logs/mow_sessions/
# inside the container, which is not mounted — better to bind-mount docker/logs/
# or redirect via --output-dir):
docker exec -d mowgli-ros2 bash -c '
  source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \
  python3 /ros2_ws/scripts/mow_session_monitor.py \
    --session 2026-04-20-kinematic-icp-tuning-v1 \
    --output-dir /ros2_ws/maps'

# Interactively from inside the container (Ctrl-C to stop + write summary):
docker exec -it mowgli-ros2 bash -c '
  source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \
  python3 /ros2_ws/scripts/mow_session_monitor.py --session <name> \
    --output-dir /ros2_ws/maps'
```

The `--output-dir /ros2_ws/maps` redirects to the bind-mounted `install_mowgli_maps` Docker volume so logs persist outside the container. Or bind-mount `docker/logs/mow_sessions/` explicitly in compose for a host-visible path.

**What it records** (per-sample, 10 Hz default):
- Fused pose + twist from `/odometry/filtered_map` (x/y/z, yaw, vx/vy/wz) **+ position covariance (cov_xx, cov_yy, derived sigma_xy_m)**
- TF snapshots: `map→base_footprint` (composed through `map→odom→base_footprint`), `odom→base_footprint` (local EKF)
- Wheel twist + covariance + integrated distance and yaw
- IMU gyro + accel + integrated gyro yaw
- GPS NavSatFix (lat/lon/alt/status/covariance) + `/gps/absolute_pose` ENU
- Dock heading (`/gnss/heading` while charging)
- BT state (state_name, current_area, current_strip), hardware mode, emergency flags, battery
- `cmd_vel_nav` (Nav2 output) + `cmd_vel` (post-safety, what reaches motors)
- Nav2 `/plan` length, next pose, goal pose, distance-to-goal
- LiDAR scan health (valid point count, min range)
- Kinematic-ICP twist (if enabled), for the `fusion ↔ kinematic-icp` cross-check
- **Cross-source consistency**: `fusion ↔ gps` distance, `fusion ↔ kinematic-icp` integrated-pose distance + yaw diff, `wheel ↔ gyro` yaw drift
- **RTK covariance-drop health**: on every RTK-Fixed GPS arrival, confirm `/odometry/filtered_map` cov drops to σ≤~3 cm within 300 ms — surfaced as `cross_checks.rtk_cov_check.{arrivals,ok,violations}` per sample and rolled into a `rtk_cov_check.verdict` ("healthy" / "intermittent" / "gate_rejecting" / "no_rtk") in the summary.

**Metadata header** (first line of the JSONL): session name, UTC timestamp, git branch + commit + dirty flag, docker image tags from `.env`, SHA-256 truncated hashes of `mowgli_robot.yaml`, `localization.yaml`, `nav2_params.yaml`, `kinematic_icp.yaml` — so sessions from different tunings are grouped/comparable.

**Summary record** (last line, written on Ctrl-C or clean shutdown): total duration, samples written, wheel-integrated distance, straight-line displacement, peak `fusion↔gps` error, peak `wheel↔gyro` yaw drift, RTK cov-check totals + verdict, final BT state.

**Log directory:** `docker/logs/mow_sessions/<session_name>.jsonl`. Commit notable sessions (golden runs, failure cases) so they survive in git history.

## Recommended Skills and Agents

When using Claude Code on this project:

### Skills to Use
- `/ros2-engineering` — ROS2 node patterns, QoS, launch files, Nav2 (use for any ros2/ work)
- `/cpp-coding-standards` — C++ Core Guidelines (use for C++ reviews)
- `/docker-patterns` — Dockerfile and compose patterns (use for docker/ and sensors/ work)
- `/tdd` — Test-driven development (use when adding new features)

### Agents to Invoke
- **code-reviewer** — after any code changes
- **cpp-reviewer** — after C++ changes in ros2/
- **security-reviewer** — before commits touching auth, configs, or firmware commands
- **build-error-resolver** — when colcon or Docker builds fail
- **tdd-guide** — when implementing new features
- **architect** — for design decisions spanning multiple packages

## What NOT to Do

- Do NOT add ROS1 patterns (rosserial, roscore, catkin) — this is ROS2 only
- Do NOT use FastRTPS — Cyclone DDS is required
- Do NOT mock the database/firmware in integration tests — use real interfaces
- Do NOT publish a `map→odom` TF from Kinematic-ICP, Nav2, or any other node. `map→odom` is owned by `ekf_map_node`. Kinematic-ICP output goes into `ekf_odom_node` as a `/encoder2/odom` twist, not TF.
- Do NOT re-introduce continuous SLAM (Cartographer, slam_toolbox, rtabmap, etc.). `navsat_transform` + RTK already globally anchors the map frame; SLAM overhead degrades the map under real-world mower conditions (sparse outdoor features, long idle periods on dock, wind-moved foliage).
- Do NOT feed Kinematic-ICP or any LiDAR-derived pose back into robot_localization as an absolute pose or TF — it enters as a twist on `/encoder2/odom` only, to avoid feedback loops
- Do NOT point Kinematic-ICP's `wheel_odom_frame` at `odom` or its `base_frame` at `base_footprint` — those are robot_localization's fused frames. K-ICP must read the parallel tree (`wheel_odom_raw` / `base_footprint_wheels`) and the mirrored sensor frame (`lidar_link_wheels` via `/scan_kicp`) so nothing about its input depends on its own output.
- Do NOT send blade commands without firmware safety checks
- Do NOT hardcode GPS coordinates, dock poses, or NTRIP credentials
- Do NOT use MPPI controller for coverage paths — it jumps between swaths
- Do NOT use RPP for coverage paths — use FTCController for <10mm lateral accuracy on swaths
- Do NOT use `base_link` as robot_base_frame in Nav2/robot_localization — use `base_footprint` (REP-105)
- Do NOT use opennav_docking UndockRobot — use Nav2 BackUp behavior (isDocked() unreliable with GPS drift)
- Do NOT change any of the three robot-footprint sources of truth independently — `mowgli_robot.yaml:chassis_*`, `mowgli_robot.yaml:robot_geometry.*`, and `nav2_params.yaml:collision_monitor:PolygonSlow.points` are manually-synced (Architecture Invariant #15, GH issue #67 tracks the consolidation work). Always change all three in the same commit with consistent values.
- Do NOT mark `do_mag_calibration` true permanently in `calibrate_imu_yaw_node` defaults — adds ~30 s of in-place rotation to every yaw calibration drive. The GUI's "Magnetometer calibration → Enable & run" button flips the param dynamically via `/calibrate_imu_yaw_node/set_parameters` and resets it on the way out (see `gui/pkg/api/calibration.go::runImuYawCalibration`).
