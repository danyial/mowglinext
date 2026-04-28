# Phase 1: Coverage Planner Rewrite — Specification

**Created:** 2026-04-28
**Ambiguity score:** 0.17 (gate: ≤ 0.20)
**Requirements:** 13 locked

## Goal

Replace the pull-based ad-hoc strip planner inside `map_server_node` with a new `coverage_planner_node` that exposes `PlanCoverage.action` and emits a deterministic sequential `PoseStamped[]` plan covering all working areas (outlines → boustrophedon swaths → return-to-dock), with footprint-aware geometry on a rectangular robot, structured pre-flight validation against 10 fixed safety points, atomic YAML checkpoints, and resume-after-charging within ≤ 5 cm + 5° of the interrupted swath.

## Background

The current strip planner lives inside `map_server_node` (`ros2/src/mowgli_map/src/map_server_node.cpp`) and works pull-style: the BT calls `~/get_next_strip` per swath, `~/get_outline_path` per area, `~/get_coverage_status` to decide when to move on. Geometry is tool-centric (`outline_offset = mower_width/2 + safety`) — the rectangular robot body never enters the math, even though the chassis is rectangular and overhangs the blade. Coverage progress is tracked as a grid_map layer (`mow_progress`), not as a semantic checkpoint, so a charge-cycle resume restarts at the nearest unmowed cell instead of the open swath endpoint.

A `PlanCoverage.action` skeleton exists in `mowgli_interfaces/action/` from a prior BCD spike (phase tags "headland", "decomposition", "swaths", "routing"). It was never wired up. `MapArea.msg` already carries `is_navigation_area: bool`, and the GUI can already create navigation-area polygons — that part needs no work.

This phase produces the new `coverage_planner_node` package, rewrites `PlanCoverage.action` to match the spec below, deletes the pull-path artifacts (services + BT-nodes) once the new path is verified, and adds GUI-side preview triggering plus per-area narrow-area-strategy selection.

## Requirements

1. **New `coverage_planner_node` package**: A separate ROS2 node that owns coverage planning, decoupled from `map_server_node`'s map/area DB role.
   - Current: Coverage planning is embedded in `map_server_node` (`ensure_strip_layout`, `find_next_unmowed_strip`, `compute_outline_path`, `is_strip_mowed`, `is_strip_blocked`)
   - Target: New `mowgli_coverage_planner` ROS2 package with `coverage_planner_node` executable, action server hosting `PlanCoverage.action`
   - Acceptance: `colcon build --packages-select mowgli_coverage_planner` succeeds; `ros2 node list` shows `/coverage_planner_node`; `ros2 action list` shows `/coverage_planner_node/plan_coverage`

2. **`PlanCoverage.action` schema rewrite**: Action accepts current robot pose + dock pose + checkpoint hint, returns a sequential `PoseStamped[]` plan with metadata, supports cancellation and progress feedback.
   - Current: Existing action is a BCD-style sketch with polygon-only goal and no metadata, error schema, or checkpoint plumbing
   - Target: Goal contains `start_pose` (current robot pose, optional — empty = from dock), `dock_pose`, `mow_angle_offset_deg` (-1 = auto), `resume_from_checkpoint` (bool); Result contains `bool success`, sparse `PoseStamped[] plan`, `PlanMetadata metadata`, `PlanError error` (only valid when success=false); Feedback contains `float32 progress_percent` and `string phase`
   - Acceptance: Action server accepts a goal with valid input and returns `success=true` with a non-empty plan; rejects input with no working areas with `success=false` and `error.error_code = NO_AREAS`; cancel-goal aborts within 1 s

3. **Sparse plan output**: Plan is a sparse `PoseStamped[]` — one pose per outline vertex, swath endpoint, dock waypoint — Nav2 `FollowPath` densifies during execution.
   - Current: No plan exists; pull-pattern produces strip-by-strip paths densified by `compute_outline_path` per call
   - Target: Plan typically 50–500 waypoints for a 1000 m² area with 5 obstacles; one pose per swath start + swath end (not every 5 cm); outlines emit one pose per polygon vertex of the offset polygon
   - Acceptance: For a 500 m² square area with `path_spacing = 0.13 m` and 1 outline pass, `plan.size()` lies between 50 and 200 (test asserts upper bound); each pair of consecutive `MOWING_BOUSTROPHEDON` poses is the start+end of one swath

4. **Segment-type annotation**: Every waypoint carries a `segment_type` enum so the BT can switch blade and speed correctly.
   - Current: Path messages carry no semantic annotation
   - Target: Each plan element is a custom `CoverageWaypoint` with `geometry_msgs/PoseStamped pose`, `uint32 sequence_id`, `float32 speed`, `bool blade_enabled`, and `uint8 segment_type` ∈ {UNDOCK, TRANSIT, OUTLINE_WORKING_AREA, OUTLINE_OBSTACLE, MOWING_BOUSTROPHEDON, RETURN_TO_DOCK, DOCK_APPROACH, DOCKING}
   - Acceptance: For every waypoint in a generated plan, `segment_type` matches the spec table (UNDOCK only at index 0–1, DOCKING at the last index, MOWING_BOUSTROPHEDON only inside working areas, blade_enabled=true only for MOWING_BOUSTROPHEDON and OUTLINE_WORKING_AREA and OUTLINE_OBSTACLE)

5. **Plan metadata**: Plan ships with metadata required by GUI display, post-mortem analysis, and verification.
   - Current: No metadata structure exists
   - Target: `PlanMetadata` struct with `float64 mow_angle_used_deg`, `uint32 outline_passes_used`, `float64 path_spacing_used`, `uint32[] processed_area_indices`, `uint32[] skipped_area_indices`, `string[] skip_reasons` (parallel to skipped_area_indices), `string[] warnings`, `Checkpoint checkpoint_seed` (initial checkpoint embedded in plan)
   - Acceptance: Metadata is non-null on success; `processed_area_indices.size() + skipped_area_indices.size()` equals total areas count; for each skipped area there is a parallel reason string

6. **Footprint-aware geometry**: All geometric checks use the rectangular robot footprint with `drive_axis_offset` as the rotation centre, not the tool plate.
   - Current: `outline_offset = mower_width/2 + safety` — tool-centric; chassis overhang is unmodelled
   - Target: Robot footprint is a 4-point polygon defined by `robot_length`, `robot_width`, `drive_axis_x_offset`, `drive_axis_y_offset` (all from `mowgli_robot.yaml`); collision checks call Boost.Geometry `intersection`/`covered_by` with the footprint polygon, swept across each pose; in-place yaw rotation sweeps the full footprint disc and validates that disc against allowed area
   - Acceptance: Unit test creates a working area where the tool fits but the chassis overhang would intersect an obstacle — planner rejects the plan with `error_code = FOOTPRINT_VIOLATION`; the legacy tool-centric check would have accepted it (regression guard)

7. **Outlines: Working Area outside-in, Obstacles inside-out**: Working-area outlines are generated from the polygon edge inward; obstacle outlines from the obstacle edge outward.
   - Current: `compute_outline_path` does outside-in for working areas only; obstacle outlines are intentionally skipped (#61 mitigation)
   - Target: For each working area, `outline_passes` (configurable, ≥ 1) parallel inward offsets at step `tool_width − strip_overlap`, pass-0 centerline at `outline_offset_robot + robot_width/2` (where outline_offset_robot is measured from polygon edge to nearest robot footprint edge); for each obstacle, `outline_passes` outward offsets by the same step, starting from the obstacle edge expanded by `outline_offset_robot + robot_width/2`; navigation areas have no outlines
   - Acceptance: Plan for a working area with 1 outline pass + 1 obstacle with 1 outline pass shows OUTLINE_WORKING_AREA (CCW) before OUTLINE_OBSTACLE (CCW around obstacle, blade on); footprint is fully inside working area minus obstacle along the entire outline path

8. **Boustrophedon AABB sweep with obstacle clipping (Iteration 1)**: Inner coverage uses simple axis-aligned bounding-box sweep with per-scan-line obstacle clipping; no BCD.
   - Current: Sweep exists but is tool-centric and uses fixed inset; obstacles are pre-inflated, scan-line endpoints clipped at first obstacle hit
   - Target: After picking mow angle, rotate the working area into scan-frame; for each scan line at `path_spacing` step, clip against (a) the rotated working-area polygon and (b) every rotated obstacle expanded outward by `outline_offset_robot + robot_width/2`; emit each clipped segment as one MOWING_BOUSTROPHEDON pair; alternate direction per scan line (boustrophedon); skip segments shorter than `robot_length` after applying narrow-area strategy
   - Acceptance: For a square working area with one circular obstacle, plan covers the area minus the obstacle band; consecutive swath directions alternate; no swath crosses an obstacle outline

9. **Mow-angle auto-rotation**: When `mow_angle_offset_deg = -1`, planner derives `new_angle = (last_completed_angle + angle_increment) mod 180°` from the persisted last-mowed-angle.
   - Current: `mow_angle_override_deg` is a fixed YAML value; auto-mode (NaN) calls `compute_optimal_mow_angle` which uses Minimum Bounding Rectangle, ignoring history
   - Target: Sentinel `-1` means "auto-rotate from last completed angle"; persisted last-mowed-angle is read from the area-level coverage state file (default `angle_increment = 30°`, configurable in `mowgli_robot.yaml`); angle is normalised to [0°, 180°)
   - Acceptance: Three sequential plans on the same unchanged area with `mow_angle_offset_deg = -1` produce angles that differ by exactly `angle_increment` mod 180°; first run with no history uses MBR-derived angle as the seed

10. **YAML checkpoint persistence**: Per-area coverage state is persisted to YAML files atomically at the end of every completed swath.
    - Current: Coverage state is in `mow_progress` grid_map layer (per-cell freshness float); a charge cycle keeps the grid but loses semantic context (open swath, current direction, last angle)
    - Target: For each working area, a sidecar YAML file `<areas_dir>/coverage_<area_index>.yaml` holds `current_outline_index`, `current_swath_index`, `swath_direction` (FORWARD|REVERSE), `last_completed_swath_index`, `next_open_swath_index`, `last_mow_angle_deg`, `last_swath_endpoint` (Pose); writes go through a temp file + `rename(2)` for atomicity
    - Acceptance: Killing the planner mid-plan and reading the YAML on disk yields a valid checkpoint with all fields populated; corrupted YAML triggers `error_code = RESUME_CHECKPOINT_INVALID` on the next plan request

11. **Resume after charging**: Re-planning with `resume_from_checkpoint = true` continues at the open swath within ≤ 5 cm + 5° yaw of where the previous plan was interrupted.
    - Current: No semantic resume — re-plan walks the strip layout from scratch and skips strips above the `is_strip_mowed` threshold
    - Target: Goal flag `resume_from_checkpoint = true` reads each area's YAML checkpoint, skips already-mowed swaths (emit them as TRANSIT or omit them entirely from the new plan), starts the next MOWING_BOUSTROPHEDON pose at the open-swath endpoint within ≤ 5 cm + 5° yaw of the persisted `last_swath_endpoint`, preserves the persisted `last_mow_angle_deg`
    - Acceptance: Test scenario — BT mows 3 swaths, action cancelled mid-swath-4 with RETURN_TO_DOCK, new goal with `resume_from_checkpoint = true` produces plan whose first MOWING_BOUSTROPHEDON pose lies ≤ 5 cm + 5° from the persisted endpoint; previously-mowed swaths 1–3 do not appear as MOWING_BOUSTROPHEDON in the new plan; mow-angle-of-the-row equals the persisted angle

12. **Pre-flight validation against 10 spec points + structured PlanError on failure**: All 10 validation points from the spec are checked before plan emission; failure produces a typed error, never a partial/unsafe plan.
    - Current: No pre-flight check; planner emits whatever it computes; failures surface mid-execution as Nav2/collision_monitor aborts
    - Target: Validation pipeline runs all 10 checks (footprint inside allowed area, no obstacle intersections, mowing only in working areas, navigation areas blade-off, outline offset against robot body, path spacing matches `tool_width − overlap`, narrow areas not driven, dock segments collision-free, checkpoint resumability, total distance computed); on failure returns `success=false` with `PlanError { error_code, failed_validation_point (1–10, 0=other), affected_area_indices[], affected_polygons[], human_readable }`; error codes: `NO_AREAS`, `AREA_TOO_NARROW`, `OBSTACLE_BLOCKS_AREA`, `DOCK_OUTSIDE_AREAS`, `FOOTPRINT_VIOLATION`, `OBSTACLE_OFFSET_FAILED`, `RESUME_CHECKPOINT_INVALID`, `INTERNAL`
    - Acceptance: For each error_code, a unit test triggers exactly that error path; in success path, all 10 validation points are evaluated (asserted via test hook that returns the validation report alongside the plan)

13. **Narrow-area handling: 3 strategies, operator-selectable per area**: Operator chooses `narrow_area_strategy` (SKIP / OUTLINE_ONLY / SPECIAL_PATTERN) per area in the GUI; planner applies the selected strategy to scan-line segments shorter than `2 × tool_width`.
    - Current: Narrow segments are silently dropped; no operator control
    - Target: `MapArea.msg` gains `uint8 narrow_area_strategy` (0=SKIP, 1=OUTLINE_ONLY, 2=SPECIAL_PATTERN); GUI Polygon-Editor shows a dropdown next to each area; SKIP omits the segment and emits a warning in metadata; OUTLINE_ONLY emits an extra outline pass that fills the narrow area; SPECIAL_PATTERN emits a single-pass centerline along the segment's long axis with footprint validation per pose
    - Acceptance: Three test cases — same narrow strip with each strategy — produce plans whose metadata.skip_reasons or warnings match the chosen strategy; SPECIAL_PATTERN plan validates the footprint stays inside the working area minus obstacles along the centerline

## Boundaries

**In scope:**
- New `mowgli_coverage_planner` ROS2 package, `coverage_planner_node` executable
- Rewrite of `PlanCoverage.action` (goal/result/feedback shapes per Requirement 2)
- New `CoverageWaypoint.msg` and `PlanMetadata.msg` and `PlanError.msg` and `Checkpoint.msg` definitions in `mowgli_interfaces`
- `MapArea.msg` extension: `uint8 narrow_area_strategy`
- New `GetAllAreas.srv` in `mowgli_interfaces` for the snapshot pull
- GUI: navigation-area toggle (already present, no work) + per-area narrow-area-strategy dropdown
- GUI: "Preview Plan" button that calls `PlanCoverage.action` and renders the result on the map
- BT: replace `GetNextStrip`/`TransitToStrip`/`FollowStrip`/`OutlineArea`/`GetNextUnmowedArea` with a single sequential plan-follower BT node that consumes `CoverageWaypoint[]`
- Footprint-aware Boost.Geometry collision checks
- Working-area outlines (outside-in) + obstacle outlines (inside-out) — both with footprint offset
- Boustrophedon AABB sweep with obstacle clipping
- Auto-mow-angle rotation via `angle_increment`
- YAML checkpoint persistence (atomic write)
- Resume-from-checkpoint with ≤ 5 cm + 5° tolerance
- Pre-flight validation pipeline (10 points)
- Structured `PlanError` on failure
- Narrow-area handling: SKIP / OUTLINE_ONLY / SPECIAL_PATTERN
- Unit tests + colcon build + E2E sim test (`make e2e-test`)
- Hardware smoke test on Pi5 in Eichenau garden: plan generated + validation passes + first complete swath mowed without #61/#64-class failures

**Out of scope:**
- Boustrophedon Cell Decomposition (BCD) for non-convex polygons — Iteration 2 phase, after this lands. Reason: significant complexity, not required for the rectangular gardens mowed today.
- Bezier / spline / curve smoothing of waypoints — straight segments and in-place yaw rotations only. Reason: explicit spec constraint ("Höhenpunkte, Bezier-Kurven und Splines dürfen nicht verwendet werden").
- Hard plan-generation-time budget — no upper bound; cancellation via `cancel_goal` is the operator escape hatch. Reason: user explicitly chose this in spec round 3.
- Global re-planning on dynamic obstacles — `collision_monitor` + Nav2's local planner handle dynamic obstacle avoidance without invalidating the global plan. Reason: spec calls for "lokal umfahren oder pausieren" not "globale Replanung".
- Continuous re-trigger on area edits — plan is generated on goal-receipt as a snapshot; if areas change, the operator must re-issue the goal. Reason: avoids race conditions during live editing.
- Removal-hardening / parallel-fallback period — pull-path artifacts (`get_next_strip` + `get_outline_path` + `get_coverage_status` services, the BT coverage nodes, and the legacy planner code in `map_server_node`) are deleted in this phase, not deprecated. Reason: user explicitly chose single-source-of-truth (Q1.1=a) over parallel-keep.
- Mag-fusion / localization phase 3 work — separate phase. Reason: blocked on AltIMU-10v6 hardware.
- LiDAR scan-match smart undock (#43) — separate phase. Reason: orthogonal to coverage planning.

## Constraints

- **Distro:** ROS2 Kilted with Cyclone DDS (per CLAUDE.md invariant 3); no FastRTPS.
- **Frames:** Plans live in `map` frame (X=east, Y=north, GPS-anchored). All `PoseStamped` waypoints have `header.frame_id = "map"`.
- **Robot frame for footprint:** `base_footprint` (REP-105). The 4-point footprint polygon is defined in `base_footprint` and transformed to `map` for collision checks per pose.
- **Coordinate precision:** float64 throughout (geometry_msgs default). Tolerances: positional 5 cm, angular 5°.
- **Plan size budget:** typically 50–500 waypoints for a 1000 m² area with 5 obstacles. No hard upper limit; > 5000 waypoints triggers a warning in metadata.
- **Plan-generation time:** no hard budget; action publishes `progress_percent` feedback; cancellation supported via `rclcpp_action::CancelGoal`.
- **Threading:** single-threaded executor for the action server is acceptable (planning is request/response, not real-time). No callback-group multi-threading required for Iteration 1.
- **Geometric library:** Boost.Geometry — already in the ROS2 stack, no new dependency. CGAL is forbidden (binary size + license).
- **Compatibility:** sparse plan must be consumable by Nav2 `FollowPath` action via FTCController for swaths and RPP for transits (matches CLAUDE.md invariant 8).
- **GUI bridge:** all new ROS2 surfaces (action, services, message types) must be reachable through `foxglove_bridge` over `rmw_cyclonedds`. Custom-service typesupport bug applies — actions go through cleanly, services may need topic-fallback if they hit the bug.
- **No modification of:** EKF localization, firmware, blade safety logic, dock_calibration handling.

## Acceptance Criteria

- [ ] `colcon build --packages-select mowgli_coverage_planner` succeeds on the Pi5 ARM image
- [ ] `ros2 action list` shows `/coverage_planner_node/plan_coverage` after node startup
- [ ] `PlanCoverage.action` accepts a goal with valid input and returns `success=true` with a non-empty sparse plan ≤ 500 waypoints for the standard 500 m² test area
- [ ] All 8 `PlanError.error_code` values have a triggering unit test in `test_coverage_planner.cpp`
- [ ] Plan output for a square 500 m² area with 1 obstacle and `path_spacing = 0.13 m` satisfies `50 ≤ plan.size() ≤ 200`
- [ ] Each waypoint's `segment_type` matches the spec table (regression test scans the entire plan and asserts allowed segment-types per index)
- [ ] Footprint-violation regression test: a working area where tool fits but rectangular chassis overhang clips an obstacle is rejected with `error_code = FOOTPRINT_VIOLATION`
- [ ] Auto-rotate test: three sequential plans with `mow_angle_offset_deg = -1` produce angles differing by `angle_increment` mod 180°
- [ ] Atomic checkpoint test: kill the planner mid-write, the YAML file on disk is either the previous version or the new one, never partial
- [ ] Resume test: cancel mid-swath-4, re-plan with `resume_from_checkpoint = true`, first MOWING_BOUSTROPHEDON pose ≤ 5 cm + 5° yaw of persisted endpoint
- [ ] Narrow-area strategy test: same narrow strip with SKIP / OUTLINE_ONLY / SPECIAL_PATTERN produces plans whose metadata reflects the chosen strategy
- [ ] E2E sim test (`make e2e-test`) passes with the new planner active and pull-path artifacts deleted
- [ ] Hardware smoke test on Pi5 in Eichenau garden: plan generated, all 10 validation points pass, robot mows ≥ 1 complete strip without #61-class (drive-through obstacle outline) or #64-class (10 cm BackUp abort) failures
- [ ] Pull-path artifacts (`get_next_strip` / `get_outline_path` / `get_coverage_status` services, `GetNextStrip`/`TransitToStrip`/`FollowStrip`/`OutlineArea`/`GetNextUnmowedArea` BT nodes, and the legacy planner code in `map_server_node`) are removed from the codebase
- [ ] GUI Preview button calls `PlanCoverage.action` and renders the sparse plan with segment-type colour coding (UNDOCK/DOCKING dock-orange, TRANSIT grey, OUTLINE green, MOWING_BOUSTROPHEDON blue, RETURN_TO_DOCK yellow)
- [ ] GUI per-area narrow-area-strategy dropdown writes the selected enum into `MapArea.narrow_area_strategy` and round-trips through the areas YAML

## Ambiguity Report

| Dimension          | Score | Min  | Status | Notes                                                  |
|--------------------|-------|------|--------|--------------------------------------------------------|
| Goal Clarity       | 0.90  | 0.75 | ✓      | Sparse plan + 5cm/5° resume tolerance + DoD locked     |
| Boundary Clarity   | 0.82  | 0.70 | ✓      | Pull-path deletion explicit; BCD/Bezier/no-budget out  |
| Constraint Clarity | 0.72  | 0.65 | ✓      | No hard time budget by design; Cyclone DDS + Boost.Geometry locked |
| Acceptance Criteria| 0.85  | 0.70 | ✓      | 8 error codes + Pi5 hardware smoke test required       |
| **Ambiguity**      | 0.17  | ≤0.20| ✓      |                                                        |

## Interview Log

| Round | Perspective    | Question summary                                            | Decision locked                                                        |
|-------|----------------|-------------------------------------------------------------|------------------------------------------------------------------------|
| 0     | (pre-spec)     | Six architecture questions in chat before invoking skill    | New node + Action + sequential BT + AABB-sweep + rect-footprint + YAML |
| 1     | Researcher     | Q1.1 Schicksal der Pull-Pfad-Artefakte?                    | (a) Komplett löschen — single source of truth                          |
| 1     | Researcher     | Q1.2 Navigation Area GUI + Datenfluss?                     | (i) GUI hat es bereits; (ii) Service-Snapshot via neuer GetAllAreas    |
| 1     | Researcher     | Q1.3 Wer triggert PlanCoverage.action?                     | (c) Beides — GUI Preview + BT actual run                               |
| 2     | Simplifier     | Q2.1 Plan-Granularität dense/sparse/hybrid?                | (a) Sparse — Nav2 FollowPath densifies                                  |
| 2     | Researcher     | Q2.2 Schema strukturierte Fehlermeldung?                   | Schema akzeptiert: 8 error codes, validation point, affected polygons  |
| 2     | Failure Analyst| Q2.3 Resume-Test Schwellen?                                 | ≤ 5 cm + 5° vom Endpunkt der unterbrochenen Bahn                        |
| 3     | Boundary Keeper| Q3.1 Narrow-Area Behandlung?                                | (c) Alle drei Strategien, Operator-Wahl per Area                       |
| 3     | Boundary Keeper| Q3.2 Plan-Generation-Time-Budget?                           | (c) Kein hartes Budget, cancel_goal supported                          |
| 3     | Boundary Keeper| Q3.3 Definition of Done?                                    | (b) Unit + E2E sim + 1 Hardware-Smoke-Strip im Eichenau-Garten         |

---

*Phase: 01-coverage-planner-rewrite*
*Spec created: 2026-04-28*
*Next step: /gsd-discuss-phase 1 — implementation decisions (package layout, BT plan-follower internals, GUI plan-render integration, etc.)*
