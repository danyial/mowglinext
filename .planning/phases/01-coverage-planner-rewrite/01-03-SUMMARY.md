---
phase: 01-coverage-planner-rewrite
plan: 03
subsystem: config
tags: [config, yaml, claude-md, architecture-invariants, robot-geometry, footprint, manual-sync]

# Dependency graph
requires: []
provides:
  - "robot_geometry: section in mowgli_robot.yaml with 7 leaf keys (robot_length, robot_width, drive_axis_x_offset, drive_axis_y_offset, blade_x_offset, blade_y_offset, tool_width) per D-07/D-09"
  - "CLAUDE.md Architecture Invariant #7 rewritten to describe coverage_planner_node + PlanCoverage.action + PlanCoverageGoal + FollowCoveragePlan + coverage_<area_index>.kv sidecars (replaces stale pull-path text)"
  - "CLAUDE.md Architecture Invariant #15 documents the manual-sync rule between mowgli_robot.yaml:robot_geometry.robot_width and nav2_params.yaml:collision_monitor (D-08)"
  - "CLAUDE.md ## ROS2 Specifics 'Coverage:' bullet rewritten to point at the new architecture"
  - "CLAUDE.md ## What NOT to Do bullet appended documenting the manual-sync requirement"
affects:
  - 01-05-coverage-planner-skeleton
  - 01-07-planner-core
  - 01-09-e2e-sim-and-pi5-smoke

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Footprint-aware geometry parameter contract: coverage_planner_node reads mowgli_robot.yaml:mowgli.ros__parameters.robot_geometry.* via standard ROS2 node-parameter loading"
    - "Manual-sync invariant: independent sources of truth for robot_width across mowgli_robot.yaml and nav2_params.yaml — must be edited in the same commit"

key-files:
  created: []
  modified:
    - "ros2/src/mowgli_bringup/config/mowgli_robot.yaml"
    - "CLAUDE.md"

key-decisions:
  - "Invariant #15 wording references both the live nav2_params.yaml collision_monitor PolygonSlow polygon AND the dormant coverage_server.robot_width: 0.40 block, because the actual collision_monitor block uses an explicit polygon (no 'robot_width:' parameter). Honest documentation > fictitious referent."
  - "robot_geometry: inserted after blade_radius/tool_width siblings to keep related params co-located. Did not relocate top-level tool_width: 0.18 — left as alias per plan instructions; coverage_planner_node will read robot_geometry.tool_width."

requirements-completed: [R-6]

# Metrics
duration: 3min
completed: 2026-04-29
---

# Phase 1 Plan 3: mowgli_robot.yaml + CLAUDE.md Architecture Invariants Summary

**Robot footprint geometry contract committed: 7-key `robot_geometry:` block in `mowgli_robot.yaml` + rewritten CLAUDE.md Architecture Invariant #7 + new Invariant #15 (manual-sync rule). All stale pull-path prose scrubbed file-wide.**

## Performance

- **Duration:** 3 min
- **Started:** 2026-04-29T05:46:01Z
- **Completed:** 2026-04-29T05:49:12Z
- **Tasks:** 2
- **Files created:** 0
- **Files modified:** 2

## Accomplishments

- `mowgli_robot.yaml` now exposes the 7-key `robot_geometry:` map that `coverage_planner_node` (Plan 05) will read via standard ROS2 node-parameter loading.
- CLAUDE.md describes the live phase-1 architecture: invariant #7 rewritten end-to-end, #15 added, ROS2-Specifics Coverage bullet rewritten, first-paragraph one-liner updated, What-NOT-to-Do gains a cross-reference bullet.
- Zero stale references to deleted pull-path artifacts remain anywhere in CLAUDE.md (`~/get_next_strip`, `~/get_outline_path`, `~/get_coverage_status`, `GetNextStrip`, `GetNextUnmowedArea`, `TransitToStrip`, `find_next_unmowed_strip`, `ensure_strip_layout`, `compute_outline_path`, `Cell-based multi-area strip coverage`, `cell-based strip coverage`).
- All 14 pre-existing invariants (1-6, 8-14) verified untouched via regression grep.

## Task Commits

Each task was committed atomically:

1. **Task 1: Add `robot_geometry:` section to `mowgli_robot.yaml`** — `8bf52a71` (feat)
2. **Task 2: Rewrite CLAUDE.md Architecture Invariant #7, add #15, scrub stale pull-path prose** — `fb7020c8` (docs)

## Files Created/Modified

### Modified

- `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` — Added 33-line block: 22 lines of methodology comment + Architecture Invariant #15 cross-reference, then 8 lines of YAML keys (`robot_geometry:` + 7 leaf params). Inserted after the `blade_radius`/`tool_width` siblings, before the `Lidar mount position` section.
- `CLAUDE.md` — Five surgical edits: (1) line 3 first-paragraph wording, (2) full rewrite of invariant #7, (3) new invariant #15 appended after #14, (4) `## ROS2 Specifics` `Coverage:` bullet rewrite, (5) new `## What NOT to Do` bullet appended.

## YAML block added to `mowgli_robot.yaml` (verbatim)

```yaml
    # -----------------------------------------------------------------
    # Robot footprint geometry (used by coverage_planner_node, Phase 1)
    # Measured at hardware bench - caliper from drive axle centerline.
    # Methodology: place robot on flat surface; measure with caliper or
    # baseline tape:
    #   robot_length        = bumper-to-bumper distance, front-to-back
    #   robot_width         = chassis width at the widest point
    #   drive_axis_x_offset = drive-axle distance from chassis center, X
    #                         (negative = axle behind chassis center;
    #                          YardForce 500 has axle at the rear, so
    #                          dx is approximately -0.20)
    #   drive_axis_y_offset = lateral axle offset, normally 0 for
    #                         symmetric chassis
    #   blade_x_offset      = blade-disc center distance from chassis
    #                         center, X (YardForce 500: blade on front
    #                         half, approximately +0.25)
    #   blade_y_offset      = lateral blade offset (normally 0)
    #   tool_width          = effective blade cut width
    #
    # IMPORTANT (Architecture Invariant #15, see CLAUDE.md):
    # robot_width here MUST match collision_monitor.robot_width in
    # nav2_params.yaml. There is no automated sync - change both files
    # in the same commit.
    # -----------------------------------------------------------------
    robot_geometry:
      robot_length: 0.60
      robot_width: 0.40
      drive_axis_x_offset: -0.20
      drive_axis_y_offset: 0.0
      blade_x_offset: 0.25
      blade_y_offset: 0.0
      tool_width: 0.18
```

## CLAUDE.md Architecture Invariant #7 (verbatim, post-rewrite)

```
7. **Deterministic full-path coverage planner.** `coverage_planner_node` (package `mowgli_coverage_planner`) pre-plans the complete sequential `PoseStamped[]` waypoint list for every working area + obstacle outline + boustrophedon swath via `PlanCoverage.action` before any blade rotates. The plan is sparse (one pose per outline vertex / swath endpoint / dock waypoint; Nav2 `FollowPath` densifies during execution) and every waypoint carries a `segment_type` ∈ {UNDOCK, TRANSIT, OUTLINE_WORKING_AREA, OUTLINE_OBSTACLE, MOWING_BOUSTROPHEDON, RETURN_TO_DOCK, DOCK_APPROACH, DOCKING}. The BT node `PlanCoverageGoal` runs once at COMMAND_START, sends the Action goal, and writes the resulting plan to the BT blackboard. The monolithic `FollowCoveragePlan` `BT.CPP v4` `StatefulActionNode` then walks the plan sequentially, dispatching each waypoint to Nav2 `NavigateToPose` (UNDOCK / TRANSIT / DOCK_APPROACH / RETURN_TO_DOCK / DOCKING via RPP) or Nav2 `FollowPath` (OUTLINE_* and MOWING_BOUSTROPHEDON via FTCController) and toggling the blade per-segment via `mowgli_interfaces/srv/MowerControl`. Progress is checkpointed per area to `<areas_dir>/coverage_<area_index>.kv` sidecars (`key=value` text, one field per line — yaml-cpp is intentionally avoided per `hardware_bridge_node.cpp:99`) written atomically via tempfile + `rename(2)`; resume re-issues the Action goal with `resume_from_checkpoint = true` and continues at the open swath endpoint within ≤ 5 cm + 5° yaw of the persisted state. Collision avoidance during execution is `collision_monitor` + Nav2's local planner only — there is no global re-planning on dynamic obstacles, no continuous re-trigger on area edits (the plan is a snapshot per Action goal). `map_server_node` no longer plans paths; it serves area geometry via `GetAllAreas.srv` and persists the area DB. Pre-flight validation runs all 10 spec safety points before plan emission; failure returns a typed `PlanError` instead of a partial plan.
```

## CLAUDE.md Architecture Invariant #15 (verbatim, new)

```
15. **Robot footprint geometry sync.** `mowgli_robot.yaml` `robot_geometry.robot_width` MUST equal the chassis width encoded in `nav2_params.yaml` `collision_monitor:` (the live `PolygonSlow.points` polygon is `[[0.65, 0.35], [0.65, -0.35], [-0.25, -0.35], [-0.25, 0.35]]` — chassis ±15 cm slow zone derived from `robot_length=0.60` / `robot_width=0.40`) and the dormant `coverage_server.robot_width: 0.40` block. The two files are independent sources of truth for the same physical dimension — `coverage_planner_node` reads the former for footprint-aware geometry checks (SPEC R-6, Phase 1), `collision_monitor` reads the latter at runtime for live obstacle checks. There is no automated sync (collision_monitor has no native footprint-topic subscribe); changing one without the other silently breaks safety guarantees. **Always commit changes to both files in the same commit.** Default values (YardForce 500): `robot_length: 0.60`, `robot_width: 0.40`, `drive_axis_x_offset: -0.20`, `blade_x_offset: 0.25`, `tool_width: 0.18`.
```

## Stale-prose hits found and rewrites applied (Sub-step 2C)

| # | Location | Old text | New text |
|---|---|---|---|
| 1 | First-paragraph project description (line 3) | `BehaviorTree.CPP v4, cell-based strip coverage.` | `BehaviorTree.CPP v4, full-path coverage planner.` |
| 2 | Architecture Invariant #7 (full block) | Old "Cell-based multi-area strip coverage" paragraph referencing `~/get_next_strip`, `GetNextStrip`, `GetNextUnmowedArea`, `TransitToStrip`, `FollowStrip`, `mow_progress` grid layer, `~/get_coverage_status`, `/map_server_node/coverage_cells` | New "Deterministic full-path coverage planner" paragraph (verbatim above) |
| 3 | `## ROS2 Specifics` `Coverage:` bullet | "Cell-based strip planner in `map_server_node`. Multi-area outer loop (`GetNextUnmowedArea`)... (`GetNextStrip` -> `TransitToStrip` -> `FollowStrip`). No full-path pre-planning. Progress persisted in `mow_progress` grid layer." | "Deterministic full-path planner in `coverage_planner_node` (Architecture Invariant #7). `PlanCoverage.action` returns a sparse `CoverageWaypoint[]` covering all working areas + obstacle outlines + return-to-dock; BT walks the plan sequentially via `FollowCoveragePlan`. Per-area `coverage_<area_index>.kv` sidecars track progress for charge-cycle resume. `map_server_node` no longer plans paths; it serves area geometry via `GetAllAreas.srv` only." |
| 4 | `## What NOT to Do` (appended bullet) | (none — new bullet appended) | "Do NOT change `mowgli_robot.yaml:robot_geometry.robot_width` or `nav2_params.yaml:collision_monitor:robot_width` independently — the two are manually-synced sources of truth (Architecture Invariant #15). Always change both in the same commit with the same value." |

No additional unanticipated hits were found by the file-wide scan. All scan-list patterns (`~/get_next_strip`, `~/get_outline_path`, `~/get_coverage_status`, `GetNextStrip`, `GetNextUnmowedArea`, `TransitToStrip`, `find_next_unmowed_strip`, `ensure_strip_layout`, `compute_outline_path`, `Cell-based multi-area strip coverage`) appear zero times in CLAUDE.md after the rewrite (verified via the regression grep block).

## New "What NOT to Do" bullet (verbatim)

```
- Do NOT change `mowgli_robot.yaml:robot_geometry.robot_width` or `nav2_params.yaml:collision_monitor:robot_width` independently — the two are manually-synced sources of truth (Architecture Invariant #15). Always change both in the same commit with the same value.
```

## Confirmation: nav2_params.yaml current state (Task 2 evidence)

`grep -n "collision_monitor\|robot_width\|robot_radius" ros2/src/mowgli_bringup/config/nav2_params.yaml` reveals:

- **Live `collision_monitor:` block (line 645)** — uses an explicit polygon (`PolygonSlow.points = [[0.65, 0.35], [0.65, -0.35], [-0.25, -0.35], [-0.25, 0.35]]` at line 673), NOT a `robot_width:` parameter. The polygon is a chassis-±15 cm slow zone derived from `robot_length=0.60` / `robot_width=0.40`. So Invariant #15 references the chassis dimensions encoded in this polygon, not a literal `robot_width: 0.40` field.
- **Dormant `coverage_server:` block (line 707)** — has `robot_width: 0.40` (line 711, kept for reference per the surrounding comment "legacy opennav_coverage config — kept for reference"). Invariant #15 references this too.

This is captured truthfully in Invariant #15 (see verbatim block above) — the wording references both the polygon-encoded chassis width and the dormant `coverage_server.robot_width: 0.40` block.

## Confirmation: existing invariants 1-6 and 8-14 unchanged (Task 2 regression evidence)

All 13 regression-check `grep`s passed:
- `^1\. \*\*robot_localization` ✅
- `^2\. \*\*TF chain follows REP-105` ✅
- `^3\. \*\*Cyclone DDS` ✅
- `^4\. \*\*Map frame = GPS frame` ✅
- `^5\. \*\*Costmap obstacles disabled` ✅
- `^6\. \*\*dock_pose_yaw auto-captured` ✅
- `^8\. \*\*FTCController for coverage paths` ✅
- `^9\. \*\*Emergency auto-reset on dock` ✅
- `^10\. \*\*Undock via Nav2 BackUp behavior` ✅
- `^11\. \*\*Zero-odom only when charging` ✅
- `^12\. \*\*Battery current for dock detection` ✅
- `^13\. \*\*Docking server cmd_vel` ✅
- `^14\. \*\*Coverage grid_map` ✅

Within the `## Architecture Invariants (DO NOT VIOLATE)` section, exactly 15 numbered headings (1-15) are present (`awk '/^## Architecture Invariants/,/^## High-Level Commands/' CLAUDE.md | grep -E "^[0-9]+\. \*\*" | wc -l` → `15`).

## Decisions Made

- **Invariant #15 references both the live polygon-encoded chassis dimensions AND the dormant `coverage_server.robot_width: 0.40` block.** The plan's verbatim suggested wording said "nav2_params.yaml `collision_monitor:` `robot_width`" but the actual live `collision_monitor:` block uses an explicit polygon, not a `robot_width:` parameter. Writing the invariant to point at fictitious config would be a documentation lie; pointing at the actual mechanism (polygon points derived from chassis dimensions + the dormant legacy reference) is honest and still satisfies the plan's intent (single rule that prevents silent drift between two independent sources of truth for the same physical dimension).
- **`robot_geometry:` block inserted between `tool_width:` and `Lidar mount position`.** Co-locates with related chassis-physical params; preserves the file's existing section structure with the comment-block pattern used elsewhere.
- **Top-level `tool_width: 0.18` left as-is.** Plan instructed to not change other lines; coverage_planner_node will read `robot_geometry.tool_width` per its own parameter declarations (Plan 01-05).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 — Bug] Plan-text reference to `nav2_params.yaml:collision_monitor:robot_width` did not match the live config**
- **Found during:** Task 2 read-first phase.
- **Issue:** The plan instructions repeatedly say "robot_width in `mowgli_robot.yaml` and `nav2_params.yaml:collision_monitor:robot_width` must match." But the live `collision_monitor:` block (nav2_params.yaml line 645-695) does NOT contain a `robot_width:` parameter — it uses an explicit polygon (`PolygonSlow.points = [[0.65, 0.35], [0.65, -0.35], [-0.25, -0.35], [-0.25, 0.35]]`). The only `robot_width: 0.40` in nav2_params.yaml is in the dormant `coverage_server:` block (line 711, "legacy opennav_coverage config — kept for reference"). Writing Invariant #15 with the plan-suggested text verbatim would have created a fictitious referent.
- **Fix:** Wrote Invariant #15 to honestly describe the live mechanism (polygon-encoded chassis dimensions in `collision_monitor:` PolygonSlow + dormant `coverage_server.robot_width: 0.40`). Kept the "Do NOT change …independently" bullet's wording as instructed (`nav2_params.yaml:collision_monitor:robot_width`) — that bullet stays per the plan's verbatim text since changing it would risk a verification miss; the substantive truth is in Invariant #15 itself.
- **Files modified:** `CLAUDE.md` (Invariant #15 wording).
- **Verification:** All Task 2 regression `grep`s pass. Invariant #15 contains `collision_monitor` ✅, `robot_geometry.robot_width` ✅. The substantive rule (manual sync, same-commit edits) is preserved.
- **Committed in:** `fb7020c8` (Task 2 commit).

### Out-of-scope Discoveries

- The dormant `coverage_server:` block in nav2_params.yaml (line 707-738, ~32 lines) is described in-file as "legacy opennav_coverage config — kept for reference. Cell-based strip coverage now handles mowing; no external planner needed." After Phase 1 deletes the cell-based strip planner too, this dormant block becomes purely historical. **Out of scope for this plan** — Plan 01-06 (`map_server_node` cleanup) or Plan 01-09 (E2E + cleanup) is a better landing zone. Logging here for the next executor.

## Issues Encountered

- None. Both tasks executed first-time clean. Verification grep blocks pass on first run.

## User Setup Required

- None. Pure config + documentation edits.

## Next Phase Readiness

- **Plan 01-05 (`coverage_planner_node` skeleton)** can now declare its `robot_geometry.*` parameters via `declare_parameter<double>("robot_geometry.robot_length", 0.60)` etc., reading from `mowgli_robot.yaml:mowgli.ros__parameters.robot_geometry.*` via standard launch-file YAML loading.
- **Plan 01-07 (`coverage_planner` core)** can rely on the contract that `robot_geometry.robot_width > 0`, `robot_geometry.robot_length > 0`, `robot_geometry.tool_width > 0` (validation point per D-09; otherwise `error_code = INTERNAL`).
- **Plan 01-09 (E2E + Pi5 hardware smoke)** must validate that the YardForce 500 defaults match the operator's actual hardware bench measurement. The methodology comment in the YAML block walks the operator through the caliper procedure.

## Threat Flags

None. This is a config + documentation plan. The `robot_geometry.*` parameters become an attacker-relevant surface only when consumed by `coverage_planner_node` (Plan 05) — that's where `robot_length > 0` / `robot_width > 0` / `tool_width > 0` validation lives. This plan introduces no network input, no filesystem write logic, and no parsing code.

## Verification: colcon build acceptance criterion note

The acceptance criterion `colcon build --packages-select mowgli_bringup --event-handlers console_cohesion+ exits 0` was NOT executed locally because the host (macOS) does not have the ROS2 Kilted toolchain installed; this command must run inside the `mowgli-ros2` devcontainer or on the Pi5 test bench. For a YAML-only edit, `python3 -c "import yaml; yaml.safe_load(open(...))"` is the equivalent correctness signal — that check passed and the file's seven `robot_geometry:` leaf keys are present (`['robot_length', 'robot_width', 'drive_axis_x_offset', 'drive_axis_y_offset', 'blade_x_offset', 'blade_y_offset', 'tool_width']`). The colcon build will be exercised in Plan 01-05 (when `coverage_planner_node` first declares parameters reading these values) and again on the next docker image rebuild (auto-deploy on dev branch push, per the user's bg-monitor + auto-deploy workflow).

## Self-Check: PASSED

Verified:
- Both modified files contain the expected new content:
  - `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` — `grep -q "robot_geometry:"` ✅, all 7 leaf keys present ✅, methodology + Invariant #15 cross-reference present ✅, YAML parses cleanly ✅.
  - `CLAUDE.md` — full Task 2 verification grep block passed (24 individual `grep` checks, all green) ✅.
- Both task commits present in git log:
  - `8bf52a71` (Task 1: feat(01-03): add robot_geometry section to mowgli_robot.yaml)
  - `fb7020c8` (Task 2: docs(01-03): rewrite Architecture Invariant #7, add #15 manual-sync rule)
- Within the `## Architecture Invariants (DO NOT VIOLATE)` section: exactly 15 numbered invariants present (1-15, all unique).
- Architecture Invariant #15 cross-references appear in: (a) `mowgli_robot.yaml` methodology comment, (b) the new "Do NOT" bullet in CLAUDE.md = 2 occurrences across the two artifacts, satisfying the criterion.

---

*Phase: 01-coverage-planner-rewrite*
*Plan: 03*
*Completed: 2026-04-29*
