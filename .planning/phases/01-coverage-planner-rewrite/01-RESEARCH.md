# Phase 1: Coverage Planner Rewrite — Research

**Researched:** 2026-04-28
**Domain:** ROS2 C++17 coverage planning, Boost.Geometry, BT.CPP v4, rclcpp_action, Eigen3 PCA, atomic file I/O
**Confidence:** HIGH (all key claims verified against source code; LOW items flagged)

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- **D-01:** New package `mowgli_coverage_planner` in `ros2/src/`. Clean separation from `mowgli_map`.
- **D-02:** Header-only library `mowgli_geometry` in `ros2/src/`. Both packages depend on it. Single geometry source of truth.
- **D-03:** Replace 5 coverage BT nodes with one monolithic `FollowCoveragePlan` `StatefulActionNode`. Dispatches per-waypoint based on `segment_type` to `nav2_msgs/action/NavigateToPose` (TRANSIT/UNDOCK/DOCK_APPROACH/RETURN_TO_DOCK/DOCKING) and `nav2_msgs/action/FollowPath` via FTCController (OUTLINE_*/MOWING_BOUSTROPHEDON).
- **D-04:** `PlanCoverageGoal` BT node sends Action goal, blocks until result, writes plan to BT blackboard. GUI Preview uses same Action.
- **D-05:** Checkpoint format is `key=value` text (NOT yaml-cpp). Atomic write via temp file + `rename(2)`. File extension `.kv`.
- **D-06:** Sidecar per area: `<areas_dir>/coverage_<area_index>.kv`.
- **D-07:** `robot_geometry:` section added to `mowgli_robot.yaml`.
- **D-08:** Manual sync between `mowgli_robot.yaml` `robot_geometry.robot_width` and `nav2_params.yaml` `collision_monitor.robot_width`. Documented as invariant.
- **D-09:** Default values from hardware bench measurement (YardForce 500). Operator measures once.
- **D-10:** SPECIAL_PATTERN uses PCA on polygon vertices via `Eigen::SelfAdjointEigenSolver<Matrix2d>`.
- **D-11:** Single GeoJSON FeatureCollection with `segment_type` property. Two Mapbox layers: `coverage-plan-line` + `coverage-plan-points`. Color via `["match", ["get", "segment_type"], ...]`. Delete old `plan-preview-*` layers.
- **D-12:** New `GetAllAreas.srv` in `mowgli_interfaces`. Empty request, `MapArea[] areas` response. Internal ROS2 IPC only — not bridged to GUI.

### Claude's Discretion
- **Validation pipeline:** `std::vector<std::unique_ptr<Validator>>`, fail-fast, fixed order. Each validator unit-tested.
- **Action feedback:** one `progress_percent` event per planning phase (~5 events total).
- **Eigen3 PCA:** `Eigen::SelfAdjointEigenSolver<Matrix2d>` on 2D covariance of polygon vertices.
- **Atomic-write helper:** temp-file + `rename(2)` promoted to `mowgli_coverage_planner::detail::atomic_write`.

### Deferred Ideas (OUT OF SCOPE)
- BCD for non-convex polygons — Iteration 2
- Operator-defined narrow-area centerline
- Single-source-of-truth via topic for footprint params
- Extending xacro/URDF to define footprint
- Live-replanning on area edits
- Per-segment-type GUI layer toggling
</user_constraints>

---

## 1. Executive Summary

Key findings from research:

- **FTCController already interpolates sparse waypoints.** `setPlan()` stores the path as-is; `computeVelocityCommands()` does SLERP interpolation between consecutive poses. Sparse plans (one pose per swath endpoint) are valid — no densification step required from the planner side. [VERIFIED: `ftc_controller.cpp:882-896`]

- **Boost.Geometry is available (Homebrew 1.90.0) but NOT yet used as a C++ library in the ROS2 stack.** All existing geometry is hand-rolled. The `mowgli_geometry` library will be the first consumer. `find_package(Boost REQUIRED COMPONENTS geometry)` or header-only include will suffice; no new binary dep. [VERIFIED: codebase grep + homebrew info]

- **Eigen3 is already in the ROS2 stack** via `mowgli_nav2_plugins` (`tf2_eigen`, `Eigen/Geometry`). CMakeLists pattern is established: `find_package(Eigen3 REQUIRED)` + `target_link_libraries(... Eigen3::Eigen)`. [VERIFIED: `mowgli_nav2_plugins/CMakeLists.txt:32-34`]

- **BT.CPP v4 `StatefulActionNode` pattern is well-established in the codebase.** The existing `FollowStrip`, `TransitToStrip`, `OutlineArea` nodes are correct reference implementations for the new `FollowCoveragePlan` and `PlanCoverageGoal` nodes. The `BTContext` shared-ptr-on-blackboard pattern is the project standard. [VERIFIED: `coverage_nodes.cpp`, `bt_context.hpp`]

- **The rclcpp_action server pattern is straightforward.** The `execute()` function should run in a detached thread (`handle_accepted` spawns it). Cancel check via `goal_handle->is_canceling()` in the planning loop. `goal_handle->publish_feedback()` for progress. [VERIFIED: ROS2 official docs + codebase patterns]

- **No yaml-cpp is the established pattern.** `hardware_bridge_node.cpp:99-143` documents the `parse_yaml_double()` regex approach. The `.kv` checkpoint parser (~30 lines) follows this exact idiom. [VERIFIED: `hardware_bridge_node.cpp`]

- **Atomic rename on ext4/SD is not power-loss safe without additional fsync on the directory.** Writing checkpoint atomically requires: (1) write to temp file, (2) `fsync(fd)`, (3) `rename()`, (4) open parent dir and `fsync(dir_fd)`. Skipping step 4 risks the rename not surviving a crash. [CITED: LWN.net atomic writes article]

- **The `offset_polygon_inward()` function in `map_server_node.cpp` is production-ready and handles winding detection, collinear vertex degenerate cases, and closing vertex deduplication.** Promote directly to `mowgli_geometry` with zero changes. [VERIFIED: `map_server_node.cpp:3298-3416`]

- **Biggest risk: footprint rotation sweep for in-place yaw validation.** The spec requires that during in-place yaw rotation, the swept disc (bounding circle of the rectangular footprint) is fully inside the allowed area. This is a new geometric operation with no existing reference in the codebase. Must be implemented as a disc-inside-polygon check: disc radius = hypot(robot_length/2 + |drive_axis_x_offset|, robot_width/2). [ASSUMED - formula derived from geometry]

- **foxglove_bridge typesupport bug does NOT affect actions.** The bug is specific to custom-message services. `PlanCoverage.action` and `GetAllAreas.srv` (internal IPC only) are safe. [CITED: `project_foxglove_typesupport_bug.md` memory note + D-12]

---

## 2. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│  PLANNING PHASE (rclcpp_action, ~5-30s)                          │
│                                                                  │
│  GUI (rosbridge)          BT: PlanCoverageGoal                   │
│       │                            │                             │
│       └────── PlanCoverage.action ─┘                             │
│                    │ (goal: start_pose, dock_pose,               │
│                    │  mow_angle_offset, resume_flag)             │
│                    ▼                                             │
│         coverage_planner_node                                    │
│              │                                                   │
│              ├─ 1. GetAllAreas.srv ──► map_server_node           │
│              │      (MapArea[] snapshot)                         │
│              │                                                   │
│              ├─ 2. mowgli_geometry library                       │
│              │      ├─ offset_polygon_inward()                   │
│              │      ├─ compute_optimal_mow_angle() (MBR)         │
│              │      ├─ convex_hull()                             │
│              │      ├─ point_in_polygon()                        │
│              │      ├─ footprint_polygon(pose) → 4-point poly    │
│              │      └─ pca_principal_axis() (Eigen3)             │
│              │                                                   │
│              ├─ 3. ValidatorPipeline (10 checks, fail-fast)      │
│              │      → PlanError on failure                       │
│              │                                                   │
│              ├─ 4. PlanBuilder                                   │
│              │      ├─ UNDOCK segment(s)                         │
│              │      ├─ Per working area:                         │
│              │      │    ├─ TRANSIT to area                      │
│              │      │    ├─ OUTLINE_WORKING_AREA (CCW, inward)   │
│              │      │    ├─ OUTLINE_OBSTACLE (CCW, outward)      │
│              │      │    └─ MOWING_BOUSTROPHEDON swaths          │
│              │      └─ RETURN_TO_DOCK / DOCK_APPROACH / DOCKING  │
│              │                                                   │
│              ├─ 5. CheckpointWriter (per area, .kv sidecar)      │
│              │      atomic_write helper: write→fsync→rename→fsync│
│              │                                                   │
│              └─ 6. Result: CoverageWaypoint[] + PlanMetadata     │
│                                                                  │
│  Action result ──► BT blackboard["coverage_plan"]               │
│  Action result ──► GUI GeoJSON FeatureCollection                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  EXECUTION PHASE (BT: FollowCoveragePlan StatefulActionNode)     │
│                                                                  │
│  Reads coverage_plan[] from blackboard                           │
│  Iterates waypoints by index (current_waypoint_idx_)            │
│                                                                  │
│  Per segment_type dispatch:                                      │
│                                                                  │
│  TRANSIT / UNDOCK /          nav2_msgs/action/NavigateToPose     │
│  DOCK_APPROACH /        ──►  (RPP via FollowPath controller)     │
│  RETURN_TO_DOCK /                                                │
│  DOCKING                                                         │
│                                                                  │
│  OUTLINE_WORKING_AREA /      nav2_msgs/action/FollowPath         │
│  OUTLINE_OBSTACLE /     ──►  controller_id: "FollowCoveragePath" │
│  MOWING_BOUSTROPHEDON        (FTCController, sparse OK)          │
│                                                                  │
│  Blade on/off         ──►  mowgli_interfaces/srv/MowerControl    │
│                                                                  │
│  Checkpoint write     ──►  coverage_<idx>.kv after each swath   │
│  (FollowCoveragePlan                                             │
│   calls planner node                                             │
│   to write checkpoint)                                           │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  NEW INTERFACES (mowgli_interfaces)                              │
│                                                                  │
│  action/PlanCoverage.action  (rewrites existing skeleton)        │
│  msg/CoverageWaypoint.msg    (pose + segment_type + blade + spd) │
│  msg/PlanMetadata.msg        (angle, outline_passes, warnings)   │
│  msg/PlanError.msg           (error_code + affected polygons)    │
│  msg/Checkpoint.msg          (all 7 fields from spec)            │
│  srv/GetAllAreas.srv         (empty req → MapArea[] areas)       │
│  msg/MapArea.msg             (+= uint8 narrow_area_strategy)     │
└─────────────────────────────────────────────────────────────────┘
```

### Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Coverage plan generation | `coverage_planner_node` (C++, ROS2 service tier) | — | Geometry-heavy, pure computation, decoupled from map/sensor state |
| Area/obstacle data source | `map_server_node` (via `GetAllAreas.srv`) | — | Owns area DB, polygon persistence; planner is a consumer |
| Footprint geometry math | `mowgli_geometry` (header-only lib) | — | Shared by planner + any future consumers; single source |
| Plan execution & blade control | BT `FollowCoveragePlan` node | Nav2 sub-actions | BT owns sequencing; Nav2 owns local navigation |
| Checkpoint persistence | `FollowCoveragePlan` BT node (writes after each swath) | `coverage_planner_node` (reads on resume) | Write-side is execution-phase; read-side is planning-phase |
| Plan preview rendering | GUI `MapPage.tsx` | rosbridge → `PlanCoverage.action` | GUI is the visualization tier; data flows from planner action |
| Narrow-area strategy selection | GUI `EditAreaModal` (per-area dropdown) | `MapArea.narrow_area_strategy` field | Operator choice is config, not runtime |
| Pre-flight validation | `coverage_planner_node` (ValidatorPipeline) | — | Validation is planning-time, not execution-time |
| Collision avoidance (dynamic obstacles) | Nav2 `collision_monitor` + local planner | — | Per SPEC: no global replan; local avoidance only |

---

## 3. Geometry Pipeline

### 3.1 Robot Footprint Construction

**Parameters** (from `mowgli_robot.yaml` under new `robot_geometry:` section):
```
robot_length     # L: total chassis length front-to-back (m)
robot_width      # W: total chassis width (m)
drive_axis_x_offset  # dx: distance from robot reference point to drive axle, X (m)
drive_axis_y_offset  # dy: distance from robot reference point to drive axle, Y (m)
blade_x_offset   # bx: blade center from robot reference point, X (m)
blade_y_offset   # by: blade center from robot reference point, Y (m)
tool_width       # tw: effective blade cut width (m)
```

**Measurement methodology (D-09):** Operator places robot on flat surface. Tape from drive axle centerline to front bumper = `robot_length/2 + drive_axis_x_offset`. Tape from drive axle to rear bumper = `robot_length/2 - drive_axis_x_offset`. Lateral width at widest point = `robot_width`. Blade disc center from axle = `(blade_x_offset, blade_y_offset)`. For YardForce 500: chassis_length=0.60, chassis_width=0.40 already in `mowgli_robot.yaml`; drive_axis_x_offset ≈ -0.20 (axle at rear), blade_x_offset ≈ +0.25 (blade on front half).

**Footprint polygon at pose (px, py, yaw)** in map frame — 4-point rectangle:
```
# In robot body frame (drive axle = origin):
corners = [
  (+robot_length/2 - drive_axis_x_offset,  +robot_width/2),  # front-left
  (+robot_length/2 - drive_axis_x_offset,  -robot_width/2),  # front-right
  (-robot_length/2 - drive_axis_x_offset, -robot_width/2),  # rear-right
  (-robot_length/2 - drive_axis_x_offset, +robot_width/2),  # rear-left
]
# Transform to map frame:
for c in corners:
  map_x = px + c.x * cos(yaw) - c.y * sin(yaw)
  map_y = py + c.x * sin(yaw) + c.y * cos(yaw)
```

**In-place rotation sweep disc radius:**
```
# Conservative: bounding circle of the 4-corner footprint from drive axle
r_sweep = max(hypot(c.x, c.y) for c in corners)
       ≈ hypot(max(|+robot_length/2 - drive_axis_x_offset|,
                   |-robot_length/2 - drive_axis_x_offset|),
               robot_width/2)
```
Validation check: disc of radius `r_sweep` centered at (px, py) must be contained in allowed area. [ASSUMED - no existing code reference; formula derived from geometry]

### 3.2 Outline Generation

**Working-area outlines (outside-in):**
```
For pass p in range(outline_passes):
  step = tool_width - strip_overlap
  inset_p = (robot_width/2 + outline_offset) + p * step
  offset_poly = offset_polygon_inward(working_area.polygon, inset_p)
  # Emit CCW traversal of offset_poly as OUTLINE_WORKING_AREA waypoints
  # Yaw = atan2(next.y - cur.y, next.x - cur.x) at each vertex
```

**Obstacle outlines (inside-out):**
```
For pass p in range(outline_passes):
  step = tool_width - strip_overlap
  inset_p = -(robot_width/2 + outline_offset + p * step)  # negative = outward
  offset_poly = offset_polygon_inward(obstacle.polygon, inset_p)
  # Emit CCW traversal as OUTLINE_OBSTACLE waypoints (blade on)
```

**Key formula: outline_offset_robot** = gap from polygon edge to nearest robot body edge.
The existing code uses `outline_offset` (= gap from edge to blade centre ≈ `mower_width/2 + safety`). The new footprint-centric formula: `inset_pass0 = robot_width/2 + outline_offset_param` where `outline_offset_param` is the configurable clearance gap from body edge to polygon edge (default 0.05 m). [VERIFIED: `map_server_node.cpp:3494-3540` for old formula; new formula derived per SPEC R-7]

### 3.3 Boustrophedon AABB Sweep

```
1. mow_angle = derive_mow_angle(area, checkpoint)
   # If -1: new_angle = (last_completed_angle + angle_increment) mod 180°
   # If no history: mow_angle = compute_optimal_mow_angle(area.polygon) [MBR]
   # If explicit: mow_angle = goal.mow_angle_offset_deg

2. rot_angle = π/2 - mow_angle  # rotate so strips run along Y axis
3. rotated_area = rotate_polygon(area.polygon, rot_angle)
4. rotated_obstacles = [rotate_polygon(obs, rot_angle) for obs in obstacles]
5. expanded_obstacles = [offset_polygon_inward(obs, -(robot_width/2 + outline_offset)) 
                         for obs in rotated_obstacles]  # expand outward in rotated frame

6. AABB of rotated_area: [min_x, max_x] × [min_y, max_y]
7. x_inset = first_strip_centerline - strip_step/2
   # first_strip_centerline = outline_inset_innermost_pass + strip_step
   # where outline_inset_innermost_pass = robot_width/2 + outline_offset + 
   #                                      (outline_passes-1) * step
8. For x in arange(min_x + x_inset + strip_step/2, 
                    max_x - x_inset, 
                    strip_step):
   y_intersections = []
   # Outer polygon intersections
   for each edge of rotated_area:
     if edge crosses x: y_intersections.append((y, from_outer=True))
   # Expanded obstacle intersections  
   for each expanded_obstacle:
     for each edge:
       if edge crosses x: y_intersections.append((y, from_outer=False))
   
   sort(y_intersections by y)
   
   # Even-odd fill pairing
   for k in 0,2,4,...:
     y_lo_inset = y_inset if from_outer else 0.0
     y_hi_inset = y_inset if from_outer else 0.0
     y_lo = y_intersections[k].y + y_lo_inset
     y_hi = y_intersections[k+1].y - y_hi_inset
     if (y_hi - y_lo) < robot_length: handle_narrow(strategy)
     
     # Boustrophedon: alternate direction by column index
     if col % 2 == 0: emit (rot_x, y_lo) → (rot_x, y_hi)
     else:             emit (rot_x, y_hi) → (rot_x, y_lo)
     
     # Back-rotate to map frame
     start, end = rotate_back(x, y_lo_or_hi, rot_angle)
     # Emit as MOWING_BOUSTROPHEDON pair
```

The `y_inset` formula (footprint-centric, per SPEC R-8):
```
y_inset = max(safety_inset, 
              robot_length/2 + outline_offset + 
              (outline_passes-1)*step - strip_outline_overlap)
```
This ensures swath endpoints clear the outline band at both ends. [VERIFIED: `map_server_node.cpp:2600-2651` as reference; updated to footprint params per SPEC R-6]

### 3.4 PCA Axis for SPECIAL_PATTERN Narrow Areas

```cpp
// Polygon vertices as Eigen matrix (N×2)
Eigen::MatrixXd pts(n, 2);
for (size_t i = 0; i < n; ++i) {
  pts(i, 0) = poly[i].x;
  pts(i, 1) = poly[i].y;
}
Eigen::Vector2d centroid = pts.colwise().mean();
Eigen::MatrixXd centered = pts.rowwise() - centroid.transpose();
Eigen::Matrix2d cov = (centered.transpose() * centered) / (n - 1);
Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
// eigenvalues() sorted ascending → eigenvectors().col(1) is principal axis
Eigen::Vector2d principal_axis = solver.eigenvectors().col(1);
double angle = atan2(principal_axis.y(), principal_axis.x());
```
[CITED: Eigen3 SelfAdjointEigenSolver docs + Cornell PCA example]

Edge case: collinear vertices (all on a line) → covariance matrix is rank-1 → smallest eigenvalue ≈ 0 → `eigenvectors().col(1)` is still valid (gives the line direction). [ASSUMED - standard linear algebra]

---

## 4. Validation Architecture

### Nyquist Validation Architecture (nyquist_validation not explicitly disabled → ENABLED)

#### Test Framework
| Property | Value |
|----------|-------|
| Framework | GTest via `ament_cmake_gtest` |
| Config file | `ros2/src/mowgli_coverage_planner/CMakeLists.txt` (Wave 0 gap) |
| Quick run command | `cd ros2 && make test` (or `colcon test --packages-select mowgli_coverage_planner`) |
| Full suite command | `cd ros2 && make test` (all packages) |

#### Per-Requirement Test Map

**R-1 [new package]**
- **Test type:** Build acceptance
- **Signal:** `colcon build --packages-select mowgli_coverage_planner` exits 0; `ros2 node list` shows `/coverage_planner_node`
- **Automated command:** CI Docker build + `colcon test`
- **File:** `test_coverage_planner.cpp` (Wave 0 gap)

**R-2 [PlanCoverage.action schema]**
- **Test type:** Integration (action client sends goal, checks result shape)
- **Signal:** `success=true`, non-empty plan; `NO_AREAS` error code for empty area input
- **Automated command:** `colcon test --packages-select mowgli_coverage_planner`
- **File:** `test_coverage_planner.cpp` (Wave 0 gap)

**R-3 [sparse plan output]**
- **Test type:** Unit test on `PlanBuilder`
- **Signal:** 500 m² square + `path_spacing=0.13m` → `50 ≤ plan.size() ≤ 200`
- **Automated command:** `colcon test --packages-select mowgli_coverage_planner`
- **File:** `test_coverage_planner.cpp`

**R-4 [segment_type annotation]**
- **Test type:** Unit test scanning full plan output
- **Signal:** UNDOCK only at indices 0-1; DOCKING at last index; MOWING_BOUSTROPHEDON only inside working areas; blade_enabled correlates with type
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**R-5 [plan metadata]**
- **Test type:** Unit test on plan result
- **Signal:** metadata non-null; `processed.size() + skipped.size() == total_areas`; parallel skip_reasons
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**R-6 [footprint-aware geometry]**
- **Test type:** Unit test — regression guard
- **Signal:** Working area where blade fits but chassis overhang clips obstacle → `FOOTPRINT_VIOLATION`; legacy tool-centric check would accept (assert the difference)
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**R-7 [outlines]**
- **Test type:** Unit test on `OutlineGenerator`
- **Signal:** OUTLINE_WORKING_AREA appears before OUTLINE_OBSTACLE in plan; footprint fully inside area along entire outline
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**R-8 [AABB sweep]**
- **Test type:** Unit test on `BoustrophedonSweeper`
- **Signal:** Square area + circular obstacle → consecutive swath directions alternate; no swath crosses obstacle band
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**R-9 [auto-rotation]**
- **Test type:** Unit test with mocked checkpoint state
- **Signal:** Three sequential plans with `mow_angle_offset_deg=-1` → angles differ by `angle_increment` mod 180°; first run uses MBR seed
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**R-10 [checkpoint persistence]**
- **Test type:** Unit test — power-loss simulation
- **Signal:** Kill process mid-write → YAML on disk is either old or new, never partial (check via file size + field count)
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**R-11 [resume after charging]**
- **Test type:** Integration test
- **Signal:** Cancel mid-swath-4 → re-plan with `resume_from_checkpoint=true` → first MOWING_BOUSTROPHEDON pose ≤ 5 cm + 5° from persisted endpoint
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**R-12 [pre-flight validation]**
- **Test type:** Unit test — 8 error code paths
- **Signal:** Each of the 8 `PlanError.error_code` values has a triggering test case; success path runs all 10 validators
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**R-13 [narrow-area strategies]**
- **Test type:** Unit test — 3 strategy cases
- **Signal:** Same narrow strip with SKIP/OUTLINE_ONLY/SPECIAL_PATTERN produces plans whose metadata reflects the strategy; SPECIAL_PATTERN footprint inside area
- **Automated command:** `colcon test`
- **File:** `test_coverage_planner.cpp`

**E2E sim (`make e2e-test`)**
- **Test type:** E2E simulation
- **Signal:** New planner active, pull-path artifacts removed, robot mows ≥ 1 strip in simulation
- **File:** `ros2/e2e_test.py` (existing; needs update for new BT nodes + action)

**Hardware smoke test**
- **Test type:** Hardware (manual)
- **Signal:** Pi5 in Eichenau garden: plan generated, all 10 validation points pass, ≥ 1 complete strip without #61-class or #64-class failures
- **Automated command:** Manual — see Pi5 test bench access in `reference_pi5_testbench.md`

#### Sampling Rate
- **Per task commit:** `colcon test --packages-select mowgli_coverage_planner` (< 30 s)
- **Per wave merge:** `colcon test` (full workspace)
- **Phase gate:** Full suite green before `/gsd-verify-work`

#### Wave 0 Gaps
- [ ] `ros2/src/mowgli_coverage_planner/test/test_coverage_planner.cpp` — covers R-1 through R-13
- [ ] `ros2/src/mowgli_coverage_planner/CMakeLists.txt` — `ament_add_gtest` block
- [ ] `ros2/src/mowgli_coverage_planner/package.xml` — with `<test_depend>ament_cmake_gtest</test_depend>`

---

## 5. Boost.Geometry Cookbook

**Note: Boost.Geometry is NOT yet used as a C++ library in the codebase.** The existing geometry in `map_server_node.cpp` is hand-rolled. Boost.Geometry will be introduced in `mowgli_geometry`. All existing code uses `geometry_msgs::msg::Polygon` / `Point32` — the Boost types need an adapter or the Boost model needs to be used internally with conversion at the boundary. [VERIFIED: codebase grep shows zero `#include <boost/geometry...>` headers]

### 5.1 Adding to CMakeLists

```cmake
find_package(Boost REQUIRED COMPONENTS headers)
# Boost.Geometry is header-only; just need headers package
target_link_libraries(mowgli_geometry_lib INTERFACE Boost::headers)
```

### 5.2 Polygon Type

```cpp
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint>;  // CCW outer ring, CW holes
using BgRing = bg::model::ring<BgPoint>;
using BgMultiPolygon = bg::model::multi_polygon<BgPolygon>;

// Adapter: geometry_msgs::Polygon → BgPolygon
BgPolygon to_bg(const geometry_msgs::msg::Polygon& poly) {
  BgPolygon result;
  for (const auto& p : poly.points)
    bg::append(result.outer(), BgPoint(p.x, p.y));
  bg::correct(result);  // ensures correct winding + closing point
  return result;
}
```
[CITED: boost.org geometry docs]

### 5.3 Footprint Collision Check

```cpp
// Check footprint polygon (4-point rect) intersects an obstacle
BgPolygon footprint = make_footprint(pose, params);
BgPolygon obstacle_poly = to_bg(obstacle);

bool intersects = !bg::disjoint(footprint, obstacle_poly);
bool fp_inside_area = bg::within(footprint, area_poly) || 
                      bg::covered_by(footprint, area_poly);
```

### 5.4 Buffer (Minkowski Offset) — For Obstacle Outward Expansion

```cpp
#include <boost/geometry/algorithms/buffer.hpp>

bg::strategy::buffer::distance_symmetric<double> distance(inset_m);
bg::strategy::buffer::join_miter join;
bg::strategy::buffer::end_flat end;
bg::strategy::buffer::point_circle point;
bg::strategy::buffer::side_straight side;

BgMultiPolygon result;
bg::buffer(input_polygon, result, distance, side, join, end, point);
// Note: bg::buffer returns MultiPolygon even for single input
```

**Critical note:** Use `join_miter` (not `join_round`) for rectangular robot clearance zones — miter gives straight edges that match physical clearance. `join_round` creates curved edges that undercount clearance at corners. [CITED: Boost.Geometry buffer docs]

**Performance note:** Boost.Geometry buffer with miter strategy on convex polygon: O(n) where n = polygon vertices. For typical garden obstacles (4-8 vertices), negligible on ARM. [ASSUMED - standard computational geometry]

### 5.5 Alternative: Hand-Rolled `offset_polygon_inward()`

For the use case in this phase (convex or mildly concave polygons, outward/inward offset), the existing `offset_polygon_inward()` in `map_server_node.cpp` is more predictable than Boost.Geometry buffer, and is already battle-tested on hardware. **Recommendation: promote the existing function to `mowgli_geometry` and use it for obstacle expansion.** Adopt Boost.Geometry for intersection/covered_by checks only. This minimizes new code surface and ARM performance risk. [VERIFIED: `map_server_node.cpp:3298-3416` — handles negative inset for outward expansion]

---

## 6. Action / BT Integration Patterns

### 6.1 rclcpp_action Server Boilerplate

```cpp
// In coverage_planner_node constructor:
action_server_ = rclcpp_action::create_server<PlanCoverage>(
    this,
    "plan_coverage",
    std::bind(&CoveragePlannerNode::handle_goal, this, _1, _2),
    std::bind(&CoveragePlannerNode::handle_cancel, this, _1),
    std::bind(&CoveragePlannerNode::handle_accepted, this, _1));

// handle_goal (synchronous, must not block):
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const PlanCoverage::Goal> goal)
{
  // Quick sanity: don't accept if already planning
  if (planning_active_) return rclcpp_action::GoalResponse::REJECT;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// handle_cancel:
rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

// handle_accepted — spawn worker thread:
void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  planning_active_ = true;
  std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
}

// execute — long-running work:
void execute(std::shared_ptr<GoalHandle> goal_handle)
{
  auto result = std::make_shared<PlanCoverage::Result>();
  auto feedback = std::make_shared<PlanCoverage::Feedback>();

  // Phase 1: load areas
  feedback->progress_percent = 0.0f;
  feedback->phase = "areas_loaded";
  goal_handle->publish_feedback(feedback);

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    planning_active_ = false;
    return;
  }

  // ... planning phases ...

  result->success = true;
  result->plan = built_plan;
  result->metadata = metadata;
  goal_handle->succeed(result);
  planning_active_ = false;
}
```
[VERIFIED: ROS2 official action server tutorial + existing `coverage_nodes.cpp` patterns]

**Threading note:** Single-threaded executor for the action server is fine (per SPEC constraint). The `execute()` function runs in a detached thread; the main executor loop remains responsive to cancel requests while `execute()` is computing. [VERIFIED: SPEC §Constraints]

### 6.2 PlanCoverageGoal BT Node (sender)

```cpp
class PlanCoverageGoal : public BT::StatefulActionNode
{
  using Action = mowgli_interfaces::action::PlanCoverage;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
public:
  BT::NodeStatus onStart() override {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    if (!action_client_)
      action_client_ = rclcpp_action::create_client<Action>(
          ctx->node, "/coverage_planner_node/plan_coverage");
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
      return BT::NodeStatus::FAILURE;

    Action::Goal goal;
    goal.dock_pose = ...; // from ctx->dock_x/y/yaw
    goal.mow_angle_offset_deg = -1.0f;
    goal.resume_from_checkpoint = ...; // from blackboard or context flag

    auto send_options = rclcpp_action::Client<Action>::SendGoalOptions{};
    goal_future_ = action_client_->async_send_goal(goal, send_options);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    goal_handle_ = goal_future_.get();
    if (!goal_handle_) return BT::NodeStatus::FAILURE;

    auto result_future = goal_handle_->async_result();
    if (result_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;

    auto wrapped = result_future.get();
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;

    // Write plan to blackboard
    config().blackboard->set("coverage_plan", wrapped.result->plan);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {
    if (goal_handle_) action_client_->async_cancel_goal(goal_handle_);
  }
private:
  rclcpp_action::Client<Action>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
};
```
[VERIFIED: `coverage_nodes.cpp:FollowStrip::onStart/onRunning/onHalted` as reference pattern]

### 6.3 FollowCoveragePlan BT Node (executor)

Key state machine inside `FollowCoveragePlan::onRunning()`:

```
states: IDLE → SEND_BLADE → WAIT_BLADE → SEND_NAV_GOAL → WAIT_NAV → 
        SEND_FTC_GOAL → WAIT_FTC → CHECKPOINT_WRITE → ADVANCE_WAYPOINT
```

Critical design decisions:
- Read `coverage_plan` from blackboard as `std::vector<CoverageWaypoint>` in `onStart()`
- Maintain `current_waypoint_idx_` as member state
- For MOWING_BOUSTROPHEDON: group consecutive waypoints into a `nav_msgs::msg::Path` (start + end), send to `/follow_path` with `controller_id = "FollowCoveragePath"` (FTCController)
- For TRANSIT/UNDOCK/etc: send single PoseStamped to `/navigate_to_pose`
- Blade spinup delay: 1.5s (established constant in existing `FollowStrip`)
- `onHalted()` must cancel current active action client goal and disable blade

**Blackboard plan blob type:** `std::vector<mowgli_interfaces::msg::CoverageWaypoint>`. Must be registered as BT type if not a primitive. Use `BT::TypeInfo::Create<std::vector<CoverageWaypoint>>()` in registration. [ASSUMED - BT.CPP v4 requires type registration for custom types on blackboard]

### 6.4 BT XML Integration

New coverage subtree replaces lines 422-474 in `main_tree.xml`:

```xml
<!-- Replace Repeat/GetNextUnmowedArea/OutlineArea/StripLoop block with: -->
<PlanCoverageGoal/>
<FollowCoveragePlan/>
```

The `PlanCoverageGoal` node runs once at the start of `MowingSequence`. `FollowCoveragePlan` executes the full plan sequentially (SUCCESS when plan exhausted). BatteryGuard and RainGuard remain in the `ReactiveSequence` wrapper — they will halt `FollowCoveragePlan` via `onHalted()` which cancels the active sub-action.

**Resume path:** After a charge cycle, the BT re-enters `MowingSequence`, re-runs `PlanCoverageGoal` with `resume_from_checkpoint=true` (set from context state), which reads `.kv` files and generates a resume plan starting at the open swath.

---

## 7. Checkpoint Persistence Pattern

### 7.1 Key=Value Format

File: `<areas_dir>/coverage_<area_index>.kv`

```
current_outline_index=2
current_swath_index=14
swath_direction=FORWARD
last_completed_swath_index=13
next_open_swath_index=14
last_mow_angle_deg=45.000000
last_swath_endpoint_x=12.345678
last_swath_endpoint_y=9.876543
last_swath_endpoint_yaw=1.570796
checksum=CRC32_HEX_8CHARS
```

All fields required. Parser rejects file with missing fields → `error_code = RESUME_CHECKPOINT_INVALID`.

### 7.2 Atomic Write Helper

```cpp
// mowgli_coverage_planner::detail::atomic_write
bool atomic_write(const std::string& path, const std::string& content)
{
  // 1. Write to temp file in same directory (same filesystem as target)
  std::string tmp_path = path + ".tmp";
  int fd = ::open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) return false;
  
  ssize_t written = ::write(fd, content.data(), content.size());
  if (written != static_cast<ssize_t>(content.size())) {
    ::close(fd); return false;
  }
  
  // 2. Sync data to storage
  if (::fsync(fd) != 0) { ::close(fd); return false; }
  ::close(fd);
  
  // 3. Atomic rename
  if (::rename(tmp_path.c_str(), path.c_str()) != 0) return false;
  
  // 4. Sync directory to make rename durable on ext4/SD
  std::string dir = path.substr(0, path.rfind('/'));
  int dir_fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY);
  if (dir_fd >= 0) {
    ::fsync(dir_fd);
    ::close(dir_fd);
  }
  return true;
}
```
[CITED: LWN.net "A way to do atomic writes" + kernel.org ext4 docs]

**Why step 4 matters:** On ext4 without `data=journal` mount option (the default for SD cards), `rename()` is atomic in memory but the directory entry update may not reach storage before a power loss. `fsync(dir_fd)` forces the directory block to disk. [CITED: LWN.net atomic writes article]

### 7.3 Parser Sketch

```cpp
std::optional<Checkpoint> parse_kv(const std::string& path) {
  std::ifstream f(path);
  if (!f.good()) return std::nullopt;
  
  std::map<std::string, std::string> kv;
  std::string line;
  while (std::getline(f, line)) {
    auto eq = line.find('=');
    if (eq == std::string::npos) continue;
    kv[line.substr(0, eq)] = line.substr(eq + 1);
  }
  
  // Validate all required keys present
  for (const auto& key : required_keys) {
    if (kv.find(key) == kv.end()) return std::nullopt;
  }
  
  Checkpoint ck;
  try {
    ck.current_outline_index = std::stoul(kv["current_outline_index"]);
    ck.current_swath_index = std::stoul(kv["current_swath_index"]);
    ck.swath_direction = kv["swath_direction"];
    ck.last_mow_angle_deg = std::stod(kv["last_mow_angle_deg"]);
    // ... etc
  } catch (...) { return std::nullopt; }
  
  return ck;
}
```
[VERIFIED: pattern mirrors `hardware_bridge_node.cpp:109-128` parse_yaml_double pattern]

---

## 8. GUI Integration Pattern

### 8.1 GeoJSON FeatureCollection Shape

The GUI calls `PlanCoverage.action` via rosbridge and receives `CoverageWaypoint[]`. It converts this to:

```javascript
// Each consecutive pair of MOWING_BOUSTROPHEDON waypoints → one LineString Feature
// Each consecutive pair of OUTLINE_* waypoints → one LineString Feature  
// Each TRANSIT segment → one LineString Feature
// Each UNDOCK/DOCKING segment → one LineString Feature

const featureCollection = {
  type: "FeatureCollection",
  features: plan.map((wp, i) => {
    if (i === 0 || wp.segment_type !== plan[i-1].segment_type) return null; // pair start
    return {
      type: "Feature",
      geometry: {
        type: "LineString",
        // Convert map frame (X=east, Y=north) → GeoJSON (lon, lat)
        coordinates: [
          [plan[i-1].pose.pose.position.x + datumLon_approx,
           plan[i-1].pose.pose.position.y + datumLat_approx],
          [wp.pose.pose.position.x + datumLon_approx,
           wp.pose.pose.position.y + datumLon_approx],
        ]
      },
      properties: {
        segment_type: segment_type_name(wp.segment_type),
        blade_enabled: wp.blade_enabled,
        speed: wp.speed,
      }
    };
  }).filter(Boolean)
};
```

**Important:** The GUI uses equirectangular projection (see `transpose()`/`itranspose()` in `map.tsx`). Map frame → lon/lat conversion uses the datum. Check existing `useMapOffset` hook for the established pattern. [VERIFIED: `MapPage.tsx:97-100` for datum variables]

### 8.2 Mapbox Layer Pattern (D-11)

```jsx
// Delete old plan-preview-* layers (plan-preview-coverage, plan-preview-outline,
// plan-preview-outline-arrows, plan-preview-transits, plan-preview-strips,
// plan-preview-arrows)

// Add new layers:
<Source type="geojson" id="coverage-plan-source" data={coveragePlanGeoJson}>
  <Layer type="line" id="coverage-plan-line" paint={{
    "line-color": ["match", ["get", "segment_type"],
      "MOWING_BOUSTROPHEDON", "#1d4ed8",
      "OUTLINE_WORKING_AREA",  "#16a34a",
      "OUTLINE_OBSTACLE",      "#15803d",
      "TRANSIT",               "#9ca3af",
      "UNDOCK",                "#f97316",
      "DOCK_APPROACH",         "#fbbf24",
      "DOCKING",               "#b45309",
      "RETURN_TO_DOCK",        "#facc15",
      "#9ca3af"  // fallback
    ],
    "line-width": 2.5,
    "line-opacity": 0.9,
  }}/>
  <Layer type="circle" id="coverage-plan-points" filter={["==", "$type", "Point"]}
    paint={{"circle-radius": 4, "circle-color": "#f97316"}}/>
</Source>
```
[VERIFIED: D-11 in CONTEXT.md]

### 8.3 Narrow-Area Strategy Dropdown

In `EditAreaModal.tsx` (or wherever per-area editing happens), add a `<Select>` with three options:
```typescript
const narrowAreaOptions = [
  { value: 0, label: 'Skip' },
  { value: 1, label: 'Outline Only' },
  { value: 2, label: 'Special Pattern' },
];
```
Writes `MapArea.narrow_area_strategy = selected_value` on save. The Go backend must serialize this field in `areas.yaml` — check `generate_go_msgs.sh` and `generate_ts_types.sh` scripts: they must be re-run after adding the `uint8 narrow_area_strategy` field to `MapArea.msg`.

**Code generation chain after adding `uint8 narrow_area_strategy` to `MapArea.msg`:**
1. `python3 firmware/scripts/sync_ros_lib.py` (firmware rosserial headers)
2. `cd gui && ./generate_go_msgs.sh` (Go structs)
3. `cd gui && ./generate_ts_types.sh` (TypeScript types)
[VERIFIED: CLAUDE.md §Code Generation Workflow]

---

## 9. Pi5 / ARM Performance Risks

### 9.1 Boost.Geometry Polygon Set Operations

**Risk:** Boost.Geometry `bg::buffer()` with miter join on complex polygons can be slow. For a garden polygon with 10-20 vertices and 5 obstacles, the buffer call is O(n log n) per polygon. Total planning time estimate for 1000 m² area: ~1-5 seconds on ARM Cortex-A76 (Pi5). [ASSUMED - based on general Boost.Geometry performance characteristics]

**Mitigation:** Action server has no hard time budget (per SPEC). Progress feedback covers user expectation. Cancel-goal is the escape hatch. For obstacle outward-expansion, use existing hand-rolled `offset_polygon_inward()` rather than `bg::buffer()` — already O(n) and tested on hardware.

### 9.2 Eigen3 SelfAdjointEigenSolver

**Risk:** Minimal — 2×2 matrix, O(1) for our use case. The Eigen3 small-matrix compile path for `Matrix2d` uses no dynamic allocation. Time on Pi5: < 1 µs. [ASSUMED - Eigen3 fixed-size optimization is well-documented]

### 9.3 Action Feedback Latency over Cyclone DDS

**Risk:** ~5 feedback events per plan. Cyclone DDS on localhost (same node) has negligible latency. GUI rosbridge connection adds ~10-50 ms per message, acceptable for progress bar updates. [ASSUMED]

### 9.4 Checkpoint Write Latency on SD Card

**Risk:** `fsync(fd)` on ext4/SD card can take 50-500 ms due to SD write latency. This is called at the end of each completed swath — acceptable since it is not in the real-time path.

**Risk:** The `fsync(dir_fd)` step (step 4 in atomic_write) adds another 50-500 ms. Total worst case: ~1 second per checkpoint write on a poor SD card. Plan: document this, use class-10 SD or eMMC for Pi5 installation. [CITED: LWN.net "Unix's file durability problem" HN thread]

**Mitigation:** Checkpoint writes are in the BT `FollowCoveragePlan` node, called after the `FollowPath` action completes for a swath. During the write, the robot is paused between swaths (waiting for transit to next swath). The latency is absorbed in the transit time.

### 9.5 Convex-Hull + MBR Computation

The existing `convex_hull()` + `compute_optimal_mow_angle()` code does O(n log n) sort + O(n) convex hull. For 20-vertex polygon: < 1 ms on Pi5. [ASSUMED]

---

## 10. Open Questions

1. **Who writes the checkpoint during execution?**
   - CONTEXT.md says `FollowCoveragePlan` writes checkpoints after each swath. But `coverage_planner_node` is a separate process — `FollowCoveragePlan` would need either (a) a checkpoint service on `coverage_planner_node`, or (b) to write `.kv` files directly (needs to know the `areas_dir` path).
   - **Recommendation:** `FollowCoveragePlan` calls a new `~/write_checkpoint` service on `coverage_planner_node`, passing the area index and checkpoint fields. The planner node owns file I/O. Alternative: BT node writes files directly if given `areas_dir` via ROS parameter.
   - **This needs a planner decision before implementation.**

2. **What is the exact `nav_msgs/action/FollowPath` interface for sparse plans?**
   - FTCController's `setPlan()` stores the path as-is and does SLERP interpolation between consecutive poses. A 2-pose path (swath start + end) is therefore valid. [VERIFIED: `ftc_controller.cpp:357-363, 882-896`]
   - The `controller_id` must match the plugin name in `nav2_params.yaml`. For coverage swaths: `"FollowCoveragePath"` (which maps to `mowgli_nav2_plugins/FTCController`). [VERIFIED: `nav2_params.yaml:172-173`]

3. **How does `FollowCoveragePlan` group consecutive same-type waypoints into a path?**
   - Option A: Each MOWING_BOUSTROPHEDON pair (2 poses: swath start + end) is one `FollowPath` call.
   - Option B: All OUTLINE_WORKING_AREA poses for one area are one `FollowPath` call.
   - **Recommendation:** Option A for boustrophedon (clean blade-off transitions between swaths). Option B for outlines (one closed-loop FTC call per outline pass). This needs to be locked in PLAN.md.

4. **`navigate_to_pose` vs `follow_path` for TRANSIT segments:**
   - `NavigateToPose` uses global planning (Smac) + local control (RPP). For transit between swaths, a straight-line path is usually fine — `FollowPath` with RPP (`controller_id = "FollowPath"`) could be used.
   - **Current BT pattern:** `TransitToStrip` uses `NavigateToPose`. This is the safe default for Phase 1.

5. **Does `FollowCoveragePlan` halt properly under ReactiveSequence guards?**
   - The BatteryGuard's `NeedsDocking` or `RainGuard` will halt `FollowCoveragePlan` via `onHalted()`. This must cancel the current active sub-action client and disable the blade. The existing `FollowStrip::onHalted()` is the reference pattern.

6. **Footprint validation at every pose vs. at swath level:**
   - SPEC R-12 validation point 1 says "no robot footprint leaves allowed area". Checking every swath endpoint (start + end) may miss a swath that goes through a narrow area mid-path.
   - **Recommendation for Phase 1:** Validate at swath endpoints + at outline vertices. Mid-path validation is implicitly handled by ensuring the polygon-clipping (step 6 of sweep) generates segments that stay inside the working area.

---

## 11. Key Files to Read Before Implementing

**Priority order:**

1. **`ros2/src/mowgli_map/src/map_server_node.cpp`** lines 2459-2840 — `compute_optimal_mow_angle()`, `ensure_strip_layout()` (full AABB sweep with obstacle clipping). This is the algorithm to port + upgrade to footprint-centric geometry.

2. **`ros2/src/mowgli_map/src/map_server_node.cpp`** lines 3298-3560 — `offset_polygon_inward()`, `compute_outline_path()`. Functions to promote to `mowgli_geometry` verbatim.

3. **`ros2/src/mowgli_behavior/src/coverage_nodes.cpp`** — `FollowStrip::onStart/onRunning/onHalted`, `TransitToStrip::onStart/onRunning/onHalted`. These are the direct templates for `FollowCoveragePlan`.

4. **`ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp`** — `BTContext` struct. New plan blob field must be added here.

5. **`ros2/src/mowgli_nav2_plugins/src/ftc_controller.cpp`** lines 357-448 — `setPlan()`: confirms sparse path acceptance + SLERP interpolation.

6. **`ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp`** lines 99-143 — `parse_yaml_double()` pattern. Template for `.kv` parser.

7. **`ros2/src/mowgli_behavior/trees/main_tree.xml`** lines 308-491 — Coverage subtree to be replaced. Understand all guards (BatteryGuard, RainGuard) that will halt `FollowCoveragePlan`.

8. **`ros2/src/mowgli_bringup/config/mowgli_robot.yaml`** — Understand existing parameter structure before adding `robot_geometry:` section.

9. **`ros2/src/mowgli_bringup/config/nav2_params.yaml`** lines 72-173 — Controller plugin names: `"FollowPath"` (RPP) and `"FollowCoveragePath"` (FTCController). Required for `FollowCoveragePlan`.

10. **`ros2/src/mowgli_interfaces/CMakeLists.txt`** — Message/service/action registration pattern. Required for adding new interface files.

11. **`gui/web/src/pages/MapPage.tsx`** lines 680-755 — Existing `plan-preview-*` layers to DELETE. Understand the GeoJSON source/layer pattern.

12. **`ros2/src/mowgli_map/test/test_map_server.cpp`** — GTest + rclcpp::NodeOptions pattern for `mowgli_coverage_planner` unit tests.

---

## 12. Standard Stack

### Core

| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| BT.CPP v4 (`behaviortree_cpp`) | v4.x (Kilted default) | BT node base classes | Already in stack; `mowgli_behavior` depends on it |
| rclcpp_action | Kilted | Action server/client | Standard ROS2 action pattern |
| Eigen3 | 3.4 | 2×2 PCA covariance solver | Already in stack via `mowgli_nav2_plugins` |
| Boost.Geometry | 1.90.0 | Polygon intersection/covered_by | Available as Kilted dep; header-only |
| GTest + ament_cmake_gtest | ROS2 Kilted default | Unit tests | Established pattern in all packages |

[VERIFIED: installed versions confirmed by existing CMakeLists.txt + brew info]

### Supporting

| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `tf2_geometry_msgs` | Kilted | Frame transforms for footprint checks | When transforming footprint from base_footprint to map |
| `nav2_msgs` | Kilted | `FollowPath`, `NavigateToPose` action types | BT action clients in `FollowCoveragePlan` |
| `std_srvs` | Kilted | Trigger services | Not needed for Phase 1 |

### Installation

```bash
# All dependencies are already in the ROS2 Kilted workspace image.
# No new apt/pip installs required.
# mowgli_geometry is a new header-only package in ros2/src/ — no install step.
```

---

## 13. Project Constraints (from CLAUDE.md)

These directives MUST be followed by all planning and implementation:

- **Safety:** NEVER bypass firmware blade safety checks from ROS2. Blade commands are fire-and-forget. Emergency stop is firmware-level only.
- **TF frames:** Plans live in `map` frame (`header.frame_id = "map"`). Footprint checks use `base_footprint`. Do NOT publish TF from coverage_planner_node.
- **DDS:** Cyclone DDS only. No FastRTPS.
- **No yaml-cpp:** D-05 / `hardware_bridge_node.cpp:99` precedent. Use custom `parse_kv()` scanner.
- **FTCController for swaths:** `FollowCoveragePath` controller for MOWING_BOUSTROPHEDON and OUTLINE segments. RPP (`FollowPath`) for TRANSIT. No MPPI.
- **No continuous SLAM:** `coverage_planner_node` does NOT call slam_toolbox, Cartographer, or any SLAM service.
- **No MPPI:** Explicitly forbidden for coverage paths.
- **robot_base_frame:** `base_footprint`, not `base_link`.
- **Commit conventions:** `feat: description` / `fix: description` style. No Co-Authored-By.
- **Manual sync:** `mowgli_robot.yaml:robot_geometry.robot_width` and `nav2_params.yaml:collision_monitor.robot_width` must match. Document as Architecture Invariant #15.
- **Code generation:** After adding `uint8 narrow_area_strategy` to `MapArea.msg`, run `sync_ros_lib.py`, `generate_go_msgs.sh`, `generate_ts_types.sh`.
- **No hand-edit of generated files:** `*_generated.go`, `ros_lib/mower_msgs/*.h`, `gui/web/src/types/ros.ts`.
- **`rclcpp::Node` (not lifecycle):** per established pattern in `mowgli_behavior`. No lifecycle manager for the new node.

---

## 14. Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Polygon intersection/within test | Custom ray-casting for footprint vs. obstacle | `boost::geometry::disjoint` / `bg::within` | Handles edge cases (shared edges, degenerate polygons) |
| Polygon offset (Minkowski sum) | Custom bisector for outward obstacle expansion | Existing `offset_polygon_inward()` (promote to `mowgli_geometry`) | Already battle-tested on hardware; handles winding detection |
| PCA principal axis | Power iteration loop | `Eigen::SelfAdjointEigenSolver<Matrix2d>` | Guaranteed convergence, handles degenerate cases |
| Convex hull | Graham scan implementation | `convex_hull()` from `map_server_node.hpp` (promote) | Already Andrew's monotone chain, correct |
| BT action client boilerplate | Custom async wrapper | Standard `rclcpp_action::Client<T>` + `async_send_goal` | Existing `FollowStrip` pattern is correct template |
| Atomic file write | `std::ofstream` direct write | `open() + write() + fsync() + rename() + fsync(dir)` | Power-loss safety on SD cards requires all 4 steps |

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | In-place rotation sweep disc radius = max corner distance from drive axle | §3.1 | Underestimates clearance if drive axle not at geometric center of rotation; need to verify rotation center = drive axle physically |
| A2 | Boost.Geometry buffer O(n) for convex polygons | §9.1 | Could be slower for concave polygons; observable as planning latency |
| A3 | Eigen3 `SelfAdjointEigenSolver<Matrix2d>` with collinear vertices returns valid eigenvector | §3.4 | Degenerate covariance could produce NaN; add guard: if eigenvalue[1] < 1e-9, fall back to longest-edge alignment |
| A4 | BT blackboard `std::vector<CoverageWaypoint>` requires type registration | §6.3 | Compilation failure if type not registered; easy to fix |
| A5 | `FollowCoveragePlan` should write checkpoints via a service to `coverage_planner_node` | §6.3, §10 | If BT node writes directly, it needs `areas_dir` path passed via param; planner service is cleaner but adds IPC |
| A6 | Boost.Geometry buffer with `join_miter` for rectangular clearance | §5.4 | Miter can overshoot at very acute angles; cap with `join_miter` miter limit = 5.0 |

---

## Sources

### Primary (HIGH confidence — verified in source code)
- `ros2/src/mowgli_map/src/map_server_node.cpp` — full algorithm for ensure_strip_layout, offset_polygon_inward, compute_optimal_mow_angle, compute_outline_path
- `ros2/src/mowgli_nav2_plugins/src/ftc_controller.cpp` — setPlan() and SLERP interpolation at lines 357-896
- `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` — established BT action client patterns
- `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp:99-143` — no-yaml-cpp parse pattern
- `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp` — BTContext structure
- `ros2/src/mowgli_bringup/config/nav2_params.yaml` — controller plugin names
- `ros2/src/mowgli_interfaces/CMakeLists.txt` — interface registration pattern

### Secondary (MEDIUM confidence — official docs)
- [ROS2 Action Server Tutorial (Foxy)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) — create_server boilerplate, handle_goal/cancel/accepted
- [Boost.Geometry buffer docs 1.90.0](https://www.boost.org/doc/libs/1_90_0/libs/geometry/doc/html/geometry/reference/algorithms/buffer/buffer_7_with_strategies.html) — strategy types
- [LWN.net "A way to do atomic writes"](https://lwn.net/Articles/789600/) — fsync + rename power-loss safety
- [BehaviorTree.ROS2 RosActionNode docs](https://context7.com/behaviortree/behaviortree.ros2/llms.txt) — setGoal/onResultReceived pattern
- [Eigen3 SelfAdjointEigenSolver](https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html) — 2D PCA pattern

### Tertiary (LOW confidence — inferred)
- Boost.Geometry ARM performance estimates — based on general C++ geometry library characteristics, not benchmarked
- BT blackboard custom type registration requirement — based on BT.CPP v4 general knowledge, not verified against installed version

---

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — all packages verified in existing CMakeLists.txt files
- Geometry pipeline: HIGH (existing functions) / MEDIUM (new footprint formulas) — algorithms verified in source; footprint math derived
- Architecture: HIGH — action/BT patterns verified against existing code
- Pitfalls: HIGH — atomic write safety and Boost.Geometry non-use verified in codebase

**Research date:** 2026-04-28
**Valid until:** 2026-05-28 (stable APIs; Boost.Geometry and BT.CPP are mature)
