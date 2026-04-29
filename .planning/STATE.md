---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: unknown
stopped_at: Completed 01-05
last_updated: "2026-04-29T06:55:06.062Z"
progress:
  total_phases: 1
  completed_phases: 0
  total_plans: 9
  completed_plans: 5
  percent: 56
---

# Project state

## Current phase

1 — Coverage Planner Rewrite (Waves 1-2 complete + Wave 3 half-complete — 5/9 plans done)

## Current Plan

05 — coverage_planner skeleton (COMPLETE — committed `4bce4424`, `72a4d925`, `14e51df4`; SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-05-SUMMARY.md`)

## Total Plans

9

## Resume point

- **Last completed step:** Plan 01-05 (coverage_planner skeleton) executed via `/gsd-execute-phase 1 --auto` (sequential mode). SUMMARY committed.
- **Next step:** Wave 3 second half — 01-06 (map_server cleanup) is unblocked. After that, Wave 4 opens with 01-07 (planner core, fills the PLAN-07-PLACEHOLDER block in `coverage_planner_node.cpp`) and 01-08 (BT integration).
- **Auto-chain flag persisted:** yes (`workflow._auto_chain_active=true` in `.planning/config.json`)
- **Wave 1 plans:** 01-01 ✅ COMPLETE, 01-03 ✅ COMPLETE
- **Wave 2 plans:** 01-02 ✅ COMPLETE, 01-04 ✅ COMPLETE
- **Wave 3 plans:** 01-05 ✅ COMPLETE, 01-06 (map_server cleanup)
- **Wave 4 plans:** 01-07 (planner core: validators + sweep + narrow strategies), 01-08 (BT integration)
- **Wave 5 plans:** 01-09 (E2E sim + Pi5 hardware smoke — operator-gated checkpoint)

## Last session

- **Last session:** 2026-04-29T06:54:11.769Z
- **Stopped at:** Completed 01-05
- **Resume file:** None
- **Blockers:** None

## Performance Metrics

| Phase | Plan | Duration | Tasks | Files |
|-------|------|----------|-------|-------|
| 01    | 01   | 12min    | 3     | 18    |
| 01    | 03   | 3min     | 2     | 2     |
| 01    | 02   | 9min     | 2     | 11    |
| 01    | 04   | 15min    | 2     | 7     |
| 01    | 05   | 8min     | 2     | 11    |

## Active branch

`migrate/upstream-localization` (HEAD `c778e40c` at .planning bootstrap time)

## Locked architectural decisions

These were locked in chat on 2026-04-28 before `/gsd-spec-phase` started — the spec-phase interview must NOT re-litigate them, only refine details below them.

1. **New separate ROS2 node:** `coverage_planner_node` lives in a new package (or in `mowgli_map`), parallel to `map_server_node`. The legacy strip planner inside `map_server_node` keeps running until the new planner is hardware-verified.
2. **Action-based delivery:** `PlanCoverage.action` (rclcpp_action) — supports feedback during long planning, supports cancellation, supports resume.
3. **Sequential plan execution by BT:** the BT consumes the plan as a sequential `PoseStamped` waypoint list. Lokale dynamische Hindernisse werden via `collision_monitor` + Nav2 lokalem Planer umfahren — KEIN globales Replan.
4. **Iteration 1 = simple AABB sweep with obstacle clipping** (footprint-aware). No BCD in this phase.
5. **Robot footprint = rectangular 4-point polygon with `drive_axis_offset`** as the rotation centre. All collision checks use the footprint, not the tool plate.
6. **Checkpoint persistence = atomic YAML file** next to `mow_progress.png`, written at the end of each completed swath.

## Open milestones (parking lot)

- Localization migration Phase 3 (mag fusion, blocked by AltIMU-10v6 hardware) — separate from this rewrite.
- BCD upgrade for complex polygons — future phase, after Iteration 1 lands.
- LiDAR scan-match smart undock (#43 local tracker) — separate from this rewrite.

## Decision log

| Date | Decision | Reason |
|---|---|---|
| 2026-04-28 | Bootstrap `.planning/` directly with this single phase, skip `/gsd-new-project` | Project context is already documented in CLAUDE.md + memory; full discovery would be overhead |
| 2026-04-28 | Six architectural decisions locked pre-spec (see above) | User explicitly answered all six in chat before invoking `/gsd-spec-phase` |
| 2026-04-28 | SPEC.md committed — 13 requirements locked, ambiguity 0.17 | `/gsd-spec-phase 1` end-to-end |
| 2026-04-28 | CONTEXT.md committed — 12 implementation decisions (D-01..D-12) + Claude's discretion items | `/gsd-discuss-phase 1 --chain` end-to-end; `--chain` triggers auto-advance to plan-phase |
| 2026-04-28 | D-05 supersedes SPEC R-10 wording on file format: key=value text instead of YAML | yaml-cpp deliberately avoided in stack (`hardware_bridge_node.cpp:99`); other R-10 acceptance bits (atomic, sidecar, full field set) preserved |
| 2026-04-29 | Plan 01-01: WriteCheckpoint.srv locks BT-delegates-IO-to-planner pattern | Resolves RESEARCH §10 Q1; planner owns filesystem I/O so atomic-write helper lives in one place |
| 2026-04-29 | Plan 01-01: PlanCoverage.action old BCD-style schema discarded entirely | No clients of old schema on dev branch yet; clean rewrite is safer than versioned shim |
| 2026-04-29 | Plan 01-01: Fixed 3 latent codegen bugs (firmware parser inline-comment + Go/TS missing mowgli_interfaces case branches) | Bugs were silently corrupting Emergency.h fields and would have blocked all downstream waves; in-scope per Rule 1 |
| 2026-04-29 | Plan 01-03: Invariant #15 wording references PolygonSlow polygon + dormant coverage_server.robot_width, not the fictitious collision_monitor.robot_width parameter | The plan-suggested referent does not exist in nav2_params.yaml — the live collision_monitor block uses an explicit polygon. Honest documentation > fictitious referent (Rule 1 fix). |
| 2026-04-29 | Plan 01-02: footprint_inside_polygon uses bg::covered_by, not bg::within | Coverage planning needs the footprint to be allowed to graze the working-area boundary on outline passes; bg::within would reject those poses. T-02-03 GIGO mitigation lives in Plan 05 ValidatorPipeline. |
| 2026-04-29 | Plan 01-02: PCA degenerate-rank fallback uses the polygon's longest edge, not the SelfAdjointEigenSolver eigenvector | Robust against numerical noise on rank-1 covariances and matches SPECIAL_PATTERN intent (centerline along the dominant geometric span). |
| 2026-04-29 | Plan 01-02: atomic_write returns false when fsync(parent_dir) fails | The file is in place at that point but durability across power loss is at risk. Failing loudly surfaces SD/ext4 health issues per RESEARCH §9.4 instead of silently degrading the checkpoint guarantee. |
| 2026-04-29 | Plan 01-04: roslib@^2.x adopted as the browser-side rosbridge client (v1.x ships only ROS1 actionlib via /goal /cancel /feedback /result topics — incompatible with rosbridge_v2 + ROS2 Kilted) | v2's `Action` class uses the `send_action_goal` / `cancel_action_goal` rosbridge_v2 protocol ops, which is what the ROS2 stack speaks. v2 bundles its own types in `dist/RosLib.d.ts`; the v1-only `@types/roslib` package was removed. |
| 2026-04-29 | Plan 01-04: Browser → rosbridge direct on `ws://<host>:9090`, not through the Go API server | The Go API has no generic action-relay endpoint, and adding one was outside this plan's GUI-only scope. Direct browser→rosbridge matches CLAUDE.md's documented rosbridge_server placement (docker/README.md:305). Operators must expose port 9090 from the mowgli-ros2 container. |
| 2026-04-29 | Plan 01-04: Cross-segment-type bridge segments emit a TRANSIT line | Keeps the polyline visually continuous across segment_type boundaries; the D-11 match expression renders bridges grey, matching operator intuition that the robot physically transits between segments even when both endpoints share a non-TRANSIT label. |
| 2026-04-29 | Plan 01-04: Removed the legacy coverageLineWidth zoom-interpolated tool_width memo | Tightly coupled to the deleted plan-preview-coverage layer; the new coverage-plan-line uses a fixed 2.5 px width per UI-SPEC. A tool_width-tracking band can be reintroduced in a future plan as a separate non-segment_type-coloured layer if operators want it back. |
| 2026-04-29 | Plan 01-05: PLAN-07-PLACEHOLDER marker block in `coverage_planner_node.cpp::execute()` | Plan 01-07 will literally `grep -n PLAN-07-PLACEHOLDER` to find the extension point. Action plumbing is end-to-end live for the empty-areas + invalid-geometry paths today; the placeholder short-circuits with ERROR_INTERNAL until Plan 01-07 lands the validator pipeline + plan builder. |
| 2026-04-29 | Plan 01-05: Checkpoint .kv body does NOT carry `area_index` — encoded only in the filename via `std::to_string` | Mitigates T-05-01 (path traversal) at the type-system level: `uint32 -> std::to_string` produces digit-only output, no `..` or slash escape possible. `read_checkpoint_file` fills `area_index` from the caller's argument. |
| 2026-04-29 | Plan 01-05: T-05-02 NaN guard lives in `write_checkpoint_file`, not `serialize_checkpoint` | `serialize` is reused by the round-trip test where we want it to faithfully echo whatever it gets. `write_checkpoint_file` is the single trust boundary on the BT->planner path; `std::isfinite` rejects NaN endpoint x/y + mow_angle_deg before serialising. |
| 2026-04-29 | Plan 01-05: Yaw serialised as a single scalar (`last_swath_endpoint_yaw=`) extracted via `tf2::getYaw` | Six decimals is enough resolution for the SPEC R-11 5cm/5deg tolerance, and the .kv stays human-inspectable in the field. Parser reconstructs the quaternion via `tf2::Quaternion::setRPY(0, 0, yaw)` so the Pose is fully populated when read back. |
| 2026-04-29 | Plan 01-05: Tests link against the STATIC `mowgli_coverage_planner_lib`, not the executable | Mirrors `mowgli_map`'s gtest pattern. `test_coverage_planner_skeleton` spins the node on a SingleThreadedExecutor in a worker thread and queries the action endpoint from a fresh client node — exercises the action plumbing end-to-end without re-running `main.cpp`. |
