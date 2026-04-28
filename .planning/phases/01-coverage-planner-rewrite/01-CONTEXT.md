# Phase 1: Coverage Planner Rewrite - Context

**Gathered:** 2026-04-28
**Status:** Ready for planning

<domain>
## Phase Boundary

Phase delivers a new `mowgli_coverage_planner` ROS2 package owning a `coverage_planner_node` action server. The node emits sparse `PoseStamped[]` plans via `PlanCoverage.action` with footprint-aware geometry, narrow-area handling (SKIP / OUTLINE_ONLY / SPECIAL_PATTERN), atomic `key=value` checkpoint resume, and full pre-flight validation against 10 spec points. BT consumes the plan via a new monolithic `FollowCoveragePlan` `BT.CPP v4` `StatefulActionNode`, dispatching `segment_type` to Nav2 sub-actions internally. GUI gets a Preview button rendering the plan via a single GeoJSON FeatureCollection with `segment_type`-driven color encoding. Pull-path artifacts (services + 5 BT coverage nodes + legacy planner code in `map_server_node`) are deleted in this phase — no parallel fallback.

</domain>

<spec_lock>
## Requirements (locked via SPEC.md)

**13 requirements are locked.** See `01-SPEC.md` for full requirements, boundaries, and acceptance criteria.

Downstream agents MUST read `01-SPEC.md` before planning or implementing. Requirements are not duplicated here.

**In scope (from SPEC.md):**
- New `mowgli_coverage_planner` ROS2 package, `coverage_planner_node` executable
- Rewrite of `PlanCoverage.action` (goal/result/feedback per SPEC R-2)
- New `CoverageWaypoint.msg`, `PlanMetadata.msg`, `PlanError.msg`, `Checkpoint.msg` in `mowgli_interfaces`
- `MapArea.msg` extension: `uint8 narrow_area_strategy`
- New `GetAllAreas.srv` in `mowgli_interfaces`
- GUI: navigation-area toggle (already present, no work) + per-area narrow-area-strategy dropdown
- GUI: "Preview Plan" button calling `PlanCoverage.action`
- BT: replace 5 existing coverage nodes with single sequential plan-follower BT node
- Footprint-aware Boost.Geometry collision checks
- Working-area outlines (outside-in) + obstacle outlines (inside-out)
- Boustrophedon AABB sweep with obstacle clipping
- Auto-mow-angle rotation via `angle_increment`
- Atomic checkpoint persistence + resume within ≤ 5 cm + 5° tolerance
- Pre-flight validation pipeline (10 points)
- Structured `PlanError` on failure
- Narrow-area handling: SKIP / OUTLINE_ONLY / SPECIAL_PATTERN
- Unit tests + colcon build + E2E sim test
- Hardware smoke test on Pi5 in Eichenau garden

**Out of scope (from SPEC.md):**
- BCD for non-convex polygons — Iteration 2 phase
- Bezier/spline smoothing — straight + in-place yaw rotations only
- Hard plan-generation-time budget — no upper bound
- Global re-planning on dynamic obstacles — collision_monitor + Nav2 local planner handle these
- Continuous re-trigger on area edits — plan is a snapshot per Action goal
- Removal-hardening parallel-fallback period — pull-path is deleted, not deprecated
- Mag-fusion / localization phase 3 work
- LiDAR scan-match smart undock (#43)

</spec_lock>

<decisions>
## Implementation Decisions

### Package Layout & Code Organization
- **D-01:** New ROS2 package `mowgli_coverage_planner` lives in `ros2/src/`, contains `coverage_planner_node` executable, action server, validation pipeline, narrow-area strategies. Clean separation from `mowgli_map`.
- **D-02:** New header-only library package `mowgli_geometry` in `ros2/src/`. Contains shared geometry helpers (`offset_polygon_inward`, `point_in_polygon`, footprint-polygon helpers, PCA-axis derivation, `compute_optimal_mow_angle` via MBR, `convex_hull`). Both `coverage_planner_node` and `map_server_node` depend on it. Single source of truth, prevents API drift.

### BT plan-follower architecture
- **D-03:** Replace the 5 existing coverage BT nodes (`GetNextStrip`, `FollowStrip`, `TransitToStrip`, `OutlineArea`, `GetNextUnmowedArea`) with a single monolithic `FollowCoveragePlan` `BT.CPP v4` `StatefulActionNode`. The node holds the entire plan and dispatches per-waypoint based on `segment_type` to Nav2 sub-actions: `nav2_msgs/action/NavigateToPose` for `TRANSIT` / `UNDOCK` / `DOCK_APPROACH` / `RETURN_TO_DOCK` / `DOCKING`; `nav2_msgs/action/FollowPath` via FTCController for `OUTLINE_WORKING_AREA` / `OUTLINE_OBSTACLE` / `MOWING_BOUSTROPHEDON`; `mowgli_interfaces/srv/MowerControl` for blade enable/disable.
- **D-04:** Plan-Action triggered by a new BT node `PlanCoverageGoal` that runs once at the start of the AUTONOMOUS branch. Sends Action goal, blocks (typically 5-30 s) until result, writes plan to BT blackboard. `FollowCoveragePlan` reads from blackboard. Cancel path is BT-Halt. GUI Preview path uses the same Action via direct rosbridge call.

### Checkpoint persistence
- **D-05:** Format is simple `key=value` text, one field per line. No new build dependency (yaml-cpp is intentionally avoided in this stack per `hardware_bridge_node.cpp:99`). Custom parser ~30 lines C++, atomic write via temp file + `rename(2)`. **Note:** SPEC R-10 wording mentions "YAML files" — the implementation decision overrides this with key=value text. All other R-10 requirements (atomic, sidecar layout, full field set) preserved verbatim.
- **D-06:** Sidecar file per working area: `<areas_dir>/coverage_<area_index>.kv`. Lives next to `mow_progress.png`. Per-area independent corruption recovery, small (~200 B), easy to inspect. Atomic write per area-update.

### Robot footprint & blade-offset parameters
- **D-07:** Add new section `robot_geometry:` to existing `mowgli_robot.yaml`. Contains `robot_length`, `robot_width`, `drive_axis_x_offset`, `drive_axis_y_offset`, `blade_x_offset`, `blade_y_offset`, `tool_width` (relocated from current top level for grouping). Existing pattern (mowgli_robot.yaml is already the canonical robot-config file).
- **D-08:** `coverage_planner_node` reads `mowgli_robot.yaml` via standard ROS2 node-parameter loading. `nav2_params.yaml` `collision_monitor` section keeps its own `robot_width: 0.40` — must be **manually** kept in sync. Document in CLAUDE.md "Architecture Invariants" section: "robot_width in `mowgli_robot.yaml` `robot_geometry:` and `nav2_params.yaml` `collision_monitor:` must match." Single-source-of-truth via topic was rejected (collision_monitor has no native footprint-topic-subscribe).
- **D-09:** Default values are empirically measured at hardware bench against the YardForce 500 chassis. Operator measures once, writes to `mowgli_robot.yaml` with comment block describing measurement methodology (caliper, masking-tape baseline, drive-axle-to-front + drive-axle-to-rear lengths, blade-disc-center offsets). Validation: `robot_length > 0`, `robot_width > 0`, `tool_width > 0` else `error_code = INTERNAL` on plan request.

### Narrow-area SPECIAL_PATTERN geometry
- **D-10:** SPECIAL_PATTERN emits a single-pass centerline along the polygon's principal axis. PCA on polygon vertices via Eigen3 (already a transitive dep of nav2). Pass = single sequence of `MOWING_BOUSTROPHEDON` poses along the major-eigenvalue axis, footprint-validated per pose against the working area minus obstacles. Robust for L-shaped concave narrow strips that longest-edge alignment would mishandle.

### GUI plan-preview render
- **D-11:** Single GeoJSON FeatureCollection emitted by GUI when `PlanCoverage.action` returns. Each feature has `properties.segment_type ∈ {UNDOCK, TRANSIT, OUTLINE_WORKING_AREA, OUTLINE_OBSTACLE, MOWING_BOUSTROPHEDON, RETURN_TO_DOCK, DOCK_APPROACH, DOCKING}`. Two Mapbox layers atop this source: `coverage-plan-line` (LineString features) and `coverage-plan-points` (start/end markers). Color encoded via `["match", ["get", "segment_type"], "MOWING_BOUSTROPHEDON", "#1d4ed8", "OUTLINE_WORKING_AREA", "#16a34a", "OUTLINE_OBSTACLE", "#15803d", "TRANSIT", "#9ca3af", "UNDOCK", "#f97316", "DOCK_APPROACH", "#fbbf24", "DOCKING", "#b45309", "RETURN_TO_DOCK", "#facc15", "#9ca3af"]`. Existing `plan-preview-*` layers (added during #56) are deleted in this phase since they were tied to the old pull-based path.

### Areas snapshot retrieval
- **D-12:** New service `GetAllAreas` in `mowgli_interfaces`. Schema: empty request, response `MapArea[] areas`. `coverage_planner_node` calls once on Action goal-receipt for snapshot consistency. Existing per-index `GetMowingArea` is kept (used by GUI for individual area editing). foxglove_bridge typesupport-bug risk is bounded — the new service is only consumed internally by ROS2-to-ROS2 IPC, not bridged to the GUI.

### Claude's Discretion
- **Pre-flight validation pipeline architecture:** implement as `std::vector<std::unique_ptr<Validator>>` with each `Validator::check(const PlanContext&) -> ValidationResult` running in fixed order, fail-fast. Each validator gets its own unit test. (Internal architecture; user did not select for discussion.)
- **Action progress feedback granularity:** emit one `progress_percent` event per planning phase (`areas_loaded`, `outlines_generated`, `swaths_generated`, `validation_passed`) plus one final at result. ~5 events per plan, low overhead.
- **Eigen3 use for PCA (D-10):** leverage `Eigen::SelfAdjointEigenSolver<Matrix2d>` on the 2D covariance matrix of polygon vertices. (Implementation detail of D-10.)
- **Atomic-write helper:** common temp-file + `rename(2)` helper used by both checkpoint sidecars and any other small-file writes — promote to `mowgli_geometry` or `mowgli_coverage_planner::detail::atomic_write`.

### Folded Todos
None — this discussion stayed within Phase 1 scope.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Phase requirements
- `.planning/phases/01-coverage-planner-rewrite/01-SPEC.md` — Locked requirements (13), boundaries (in/out), acceptance criteria (16). MUST read before planning.
- `.planning/REQUIREMENTS.md` — Original user spec verbatim, source of truth for the 10 validation points + segment_type enum + checkpoint field set.

### Project invariants
- `CLAUDE.md` §"Architectural Invariants" #7 (cell-based multi-area strip coverage), #8 (FTCController for coverage paths), #1 (robot_localization is the sole localizer — coverage_planner does NOT publish TF or pose).
- `CLAUDE.md` §"What NOT to Do" — no MPPI for coverage, no continuous SLAM, no FastRTPS.

### ROS2 package conventions
- `.claude/rules/ros2.md` — Node patterns, QoS profiles, topic naming, launch files, testing, build system.

### Reference implementations (to read before building)
- `ros2/src/mowgli_map/src/map_server_node.cpp` — Existing tool-centric strip planner. Source of geometry helpers to be moved to `mowgli_geometry`. Reference for the obsolete pull-pattern that this phase deletes.
- `ros2/src/mowgli_map/include/mowgli_map/map_server_node.hpp` — Public surface (e.g., `compute_optimal_mow_angle`, `convex_hull`, `offset_polygon_inward`) — candidates for promotion to `mowgli_geometry`.
- `ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp` — Existing 5 coverage BT nodes — to be deleted in this phase.
- `ros2/src/mowgli_behavior/trees/main_tree.xml` — Current BT XML structure; coverage subtree gets rewritten with new `PlanCoverageGoal` + `FollowCoveragePlan` nodes.
- `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp:99` — Documents the no-yaml-cpp pattern (relevant to D-05).

### Configuration
- `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` — Today contains `tool_width`, `undock_*`, `dock_approach_distance`. Phase extends with `robot_geometry:` section per D-07.
- `ros2/src/mowgli_bringup/config/nav2_params.yaml` — Contains `collision_monitor` `robot_width: 0.40`. Manual sync target per D-08.

### Action / message interfaces
- `ros2/src/mowgli_interfaces/action/PlanCoverage.action` — Existing skeleton, fully rewritten per SPEC R-2.
- `ros2/src/mowgli_interfaces/msg/MapArea.msg` — Already has `is_navigation_area: bool`; phase adds `uint8 narrow_area_strategy` per SPEC R-13.

### GUI
- `gui/web/src/pages/MapPage.tsx` — Map render, existing `plan-preview-*` layers (delete + replace per D-11).

### Memory / process
- `~/.claude/projects/-Users-danny-smolinsky-jam-dev-mowglinext-danyial/memory/feedback_background_pipeline_deploy.md` — bg-monitor + auto-deploy workflow for `migrate/upstream-localization`.
- `~/.claude/projects/-Users-danny-smolinsky-jam-dev-mowglinext-danyial/memory/workflow_pi5_test_then_pr.md` — Pi5 hardware test before PR, mandatory for this phase per SPEC acceptance criterion.
- `~/.claude/projects/-Users-danny-smolinsky-jam-dev-mowglinext-danyial/memory/project_planner_geometry.md` — Old planner geometry formulas. SPEC supersedes; this memory will be updated/retired after Phase 1 lands.
- `~/.claude/projects/-Users-danny-smolinsky-jam-dev-mowglinext-danyial/memory/project_foxglove_typesupport_bug.md` — Relevant to D-12 (new service is internal, bug doesn't apply) and to potential GUI-side use of `PlanCoverage.action` (actions go through cleanly).

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `compute_optimal_mow_angle(polygon)` in `map_server_node.hpp` — Minimum Bounding Rectangle solver. Promote to `mowgli_geometry`. Used as initial seed when no last-completed-angle exists (D-10 + SPEC R-9).
- `convex_hull(points)` in `map_server_node.hpp` — Andrew's monotone chain. Promote to `mowgli_geometry`.
- `offset_polygon_inward(poly, inset)` in `map_server_node.hpp` — Minkowski offset for polygon inward shrink. Promote to `mowgli_geometry`. Used for outlines (working-area inside-out) per SPEC R-7. For obstacle outward outlines, call with negative inset.
- `point_in_polygon(pt, polygon)` in `map_server_node.hpp` — Ray casting. Promote to `mowgli_geometry`.
- `MapArea.msg`, `PlanningParams.msg`, `CoveragePath.msg` — Existing message types. `MapArea` extended with `narrow_area_strategy` per D-07-related SPEC R-13. The other two are unchanged.
- `SetPlanningParams.srv` — Live-tunable parameter setter (foxglove_bridge typesupport workaround). Survives this phase since GUI still tunes outline_passes etc. live.
- `BehaviorTree.CPP v4` infrastructure in `mowgli_behavior` — `bt_context.hpp`, ReactiveSequence pattern from `main_tree.xml`. Reuse for new BT nodes.
- `Nav2 FollowPath` action via FTCController, `Nav2 NavigateToPose` action via RPP — both already wired and used by current BT.

### Established Patterns
- **No yaml-cpp:** `hardware_bridge_node.cpp:99` documents the avoidance. Custom regex/scanner for areas.yaml. D-05 follows this pattern.
- **rclcpp + Cyclone DDS:** All nodes use `rclcpp::Node` (no lifecycle), Cyclone DDS only.
- **REP-105 frames:** `map → odom → base_footprint → base_link`. Plans live in `map`, footprint reasoning in `base_footprint`.
- **Service-based config + topic-based live-tune:** GUI publishes `PlanningParams` topic to map_server because foxglove_bridge can't relay the service version. coverage_planner_node will follow the same pattern if it needs live-tuning (probably none in iteration 1 — narrow-area-strategy is per-area in `MapArea`, not global).
- **Persistent areas:** `areas.yaml` written/read by `map_server_node::save_areas_to_file` / `load_areas_from_file` using a custom string format. `coverage_<idx>.kv` (D-06) uses similar atomic-write approach.
- **Gtest + ament_cmake:** Standard test layout per `.claude/rules/ros2.md`.

### Integration Points
- **Action client (GUI):** rosbridge layer in `gui/` will call `PlanCoverage.action` for the Preview button.
- **Action client (BT):** new `PlanCoverageGoal` BT node connects to `/coverage_planner_node/plan_coverage`.
- **Service client (planner):** `coverage_planner_node` calls `/map_server_node/get_all_areas` (new service per D-12) on every Action goal.
- **Service consumer (GUI):** GUI calls existing `/map_server_node/get_mowing_area` per area for editing — unchanged.
- **BT plan-follower:** `FollowCoveragePlan` calls `nav2_msgs/action/NavigateToPose`, `nav2_msgs/action/FollowPath`, and `mowgli_interfaces/srv/MowerControl`.
- **GeoJSON consumption (GUI):** `MapPage.tsx` adds new Mapbox source `coverage-plan-source` + 2 layers (`coverage-plan-line`, `coverage-plan-points`) per D-11. Existing `plan-preview-*` layers deleted.

</code_context>

<specifics>
## Specific Ideas

- **"Single source of truth for geometry"** — emphasized through D-02 (`mowgli_geometry` library) and D-12 (single `GetAllAreas` snapshot).
- **"Don't introduce yaml-cpp"** — D-05 follows the established `hardware_bridge_node.cpp:99` precedent. This is a deliberate Stack convention, not a casual choice.
- **"PCA-axis for narrow-areas"** (D-10) — robust against L-shaped concave narrow strips, where a longest-edge alignment would mishandle the geometry.
- **"Empirical measurement of footprint"** (D-09) — operator measures once at hardware bench and writes to `mowgli_robot.yaml` with methodology comments. No magic defaults that could hide a calibration error.
- **"Single-source-of-truth for plan rendering"** (D-11) — one GeoJSON FeatureCollection with `segment_type` properties, two Mapbox layers, color via `match` expression. Avoids the 8-layer-explosion of the alternative.

</specifics>

<deferred>
## Deferred Ideas

- **BCD (Boustrophedon Cell Decomposition)** for non-convex polygons — Iteration 2 phase, after this lands. SPEC explicitly excludes from Phase 1.
- **Operator-defined narrow-area centerline** (instead of PCA-derived) — Backlog / GUI work, not Phase 1.
- **Single-source-of-truth via topic** for footprint params (vs. manual sync between `mowgli_robot.yaml` and `nav2_params.yaml`) — Possible follow-up if drift becomes a real problem; collision_monitor doesn't natively support topic-based footprint, would need a custom bridge node.
- **Extending xacro/URDF to define footprint** instead of YAML — Correct robotics-style modeling but a separate refactor; out of scope for Phase 1.
- **Live-replanning on area edits** — Plan is a snapshot per Action goal; GUI re-issues a goal after edits. Continuous re-plan was rejected in spec-phase.
- **Per-segment-type GUI layer toggling** — `coverage-plan-line` is monolithic per D-11; if operator wants to hide TRANSIT segments etc., that's a future GUI feature.

</deferred>

---

*Phase: 01-coverage-planner-rewrite*
*Context gathered: 2026-04-28*
*Next step: /gsd-plan-phase 1 — wave-based plan with task breakdown driven by SPEC.md + this CONTEXT.md*
