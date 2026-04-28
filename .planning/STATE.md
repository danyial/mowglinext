# Project state

## Current phase
1 — Coverage Planner Rewrite (spec)

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
