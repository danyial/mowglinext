---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
status: unknown
stopped_at: Completed 01-02
last_updated: "2026-04-29T06:10:01.186Z"
progress:
  total_phases: 1
  completed_phases: 0
  total_plans: 9
  completed_plans: 3
  percent: 33
---

# Project state

## Current phase

1 — Coverage Planner Rewrite (Wave 1 + Wave-2 partial — 3/9 plans done)

## Current Plan

02 — mowgli_geometry header-only library (COMPLETE — committed `ae551459`, `63933306`, `351f4139`; SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-02-SUMMARY.md`)

## Total Plans

9

## Resume point

- **Last completed step:** Plan 01-02 executed via `/gsd-execute-phase 1 --auto` (sequential mode). SUMMARY committed.
- **Next step:** Plan 01-04 (GUI integration) is the remaining Wave 2 plan and is unblocked. After Wave 2 closes, Wave 3 opens 01-05 (coverage_planner skeleton) and 01-06 (map_server cleanup).
- **Auto-chain flag persisted:** yes (`workflow._auto_chain_active=true` in `.planning/config.json`)
- **Wave 1 plans:** 01-01 ✅ COMPLETE, 01-03 ✅ COMPLETE
- **Wave 2 plans:** 01-02 ✅ COMPLETE, 01-04 (GUI integration)
- **Wave 3 plans:** 01-05 (coverage_planner skeleton), 01-06 (map_server cleanup)
- **Wave 4 plans:** 01-07 (planner core: validators + sweep + narrow strategies), 01-08 (BT integration)
- **Wave 5 plans:** 01-09 (E2E sim + Pi5 hardware smoke — operator-gated checkpoint)

## Last session

- **Last session:** 2026-04-29T06:10:01.179Z
- **Stopped at:** Completed 01-02
- **Resume file:** None
- **Blockers:** None

## Performance Metrics

| Phase | Plan | Duration | Tasks | Files |
|-------|------|----------|-------|-------|
| 01    | 01   | 12min    | 3     | 18    |
| 01    | 03   | 3min     | 2     | 2     |
| 01    | 02   | 9min     | 2     | 11    |

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
