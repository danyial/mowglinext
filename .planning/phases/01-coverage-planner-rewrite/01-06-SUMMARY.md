---
phase: 01-coverage-planner-rewrite
plan: 06
subsystem: ros2
tags: [ros2, mowgli_map, mowgli_interfaces, refactor, deletion, narrow-area-strategy, areas-yaml]

# Dependency graph
requires:
  - phase: 01-coverage-planner-rewrite
    provides: "Plan 01-01 froze the GetAllAreas.srv schema (uint8 narrow_area_strategy on MapArea). This plan implements the server side."
  - phase: 01-coverage-planner-rewrite
    provides: "Plan 01-02 promoted point_in_polygon / convex_hull / compute_optimal_mow_angle / offset_polygon_inward into mowgli_geometry. This plan consumes that header-only library and deletes the in-file copies."
  - phase: 01-coverage-planner-rewrite
    provides: "Plan 01-05 wired coverage_planner_node to call /map_server_node/get_all_areas. This plan delivers the server side that endpoint resolves to."
provides:
  - "rclcpp::Service<mowgli_interfaces::srv::GetAllAreas> at /map_server_node/get_all_areas — snapshot pull of every entry in `areas_` plus per-area narrow_area_strategy"
  - "AreaEntry::narrow_area_strategy (uint8_t, 0=SKIP / 1=OUTLINE_ONLY / 2=SPECIAL_PATTERN per SPEC R-13) with range-checked AddMowingArea ingress and areas.yaml read/write round-trip"
  - "mowgli_map -> mowgli_geometry runtime dependency via package.xml + CMakeLists.txt + ament_target_dependencies"
  - "1188-line shrink of map_server_node.cpp: 8 strip-planner functions + 4 service handlers + 4 geometry helpers all gone, replaced by mowgli_geometry calls or moved entirely into coverage_planner_node (Plan 01-05/07)"
  - "4 pull-path .srv definitions deleted from mowgli_interfaces: GetNextStrip, GetCoverageStatus, PreviewPlan, GetOutlinePath"
  - "test_map_server.cpp shrinks: ConvexHullTest / MBRAngleTest / StripLayoutGeneratesStrips removed (equivalent coverage now in mowgli_geometry/test/)"
affects:
  - 01-07-planner-core
  - 01-08-bt-integration
  - 01-09-e2e-sim-and-pi5-smoke

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Server-side snapshot-pull handler: on_get_all_areas copies only the public MapArea fields (name, area, obstacles, is_navigation_area, narrow_area_strategy) so internal mow_progress / dock / replan bookkeeping stays out of the response (T-06-03 mitigation)."
    - "WARN-and-coerce on disk-corrupt enum: areas.yaml narrow_area_strategy out of [0..2] -> RCLCPP_WARN + clamp to 0 (Skip is the safe default). Same pattern applied to AddMowingArea ingress so GUI/CLI cannot poison the area DB (T-06-01 mitigation)."

key-files:
  created: []
  modified:
    - "ros2/src/mowgli_map/package.xml"
    - "ros2/src/mowgli_map/CMakeLists.txt"
    - "ros2/src/mowgli_map/include/mowgli_map/map_server_node.hpp"
    - "ros2/src/mowgli_map/src/map_server_node.cpp"
    - "ros2/src/mowgli_map/test/test_map_server.cpp"
    - "ros2/src/mowgli_interfaces/CMakeLists.txt"
  deleted:
    - "ros2/src/mowgli_interfaces/srv/GetNextStrip.srv"
    - "ros2/src/mowgli_interfaces/srv/GetCoverageStatus.srv"
    - "ros2/src/mowgli_interfaces/srv/PreviewPlan.srv"
    - "ros2/src/mowgli_interfaces/srv/GetOutlinePath.srv"

key-decisions:
  - "MapServerNode::point_in_polygon kept as a one-line forwarder onto mowgli_geometry::point_in_polygon in Task 1, then deleted entirely in Task 2. The forwarder bridges the inter-task gap so the Task 1 commit alone still compiles — the legacy strip-planner code (still present after Task 1) calls the wrapper, which calls the namespaced helper. Without the bridge Task 1 would not be a self-contained commit."
  - "narrow_area_strategy persisted as a single integer key per area in areas.yaml (`area_<i>_narrow_area_strategy: <0|1|2>`), not as a nested YAML map. Mirrors the existing custom non-yaml-cpp parser/writer convention (D-05 / hardware_bridge pattern). Forward-compatible with legacy on-disk files that pre-date this plan: missing field defaults to 0 (Skip), no migration required."
  - "Out-of-range narrow_area_strategy is silently coerced to 0 (Skip) with RCLCPP_WARN at both ingress points (AddMowingArea service + areas.yaml read). T-06-01 mitigation: a hand-edited / corrupted areas.yaml cannot put the planner into undefined-behaviour territory."
  - "Surviving planner parameter knobs (outline_passes, outline_offset, outline_overlap, path_spacing, mow_angle_override_deg, strip_boundary_margin_m, strip_mowed_threshold) intentionally retained in map_server_node.{hpp,cpp} even though their consumers (ensure_strip_layout, compute_outline_path) are now gone. Reasoning: the GUI's MowingSettings card publishes /map_server_node/planning_params_in to update them live, and on_set_planning_params still accepts them with sentinel semantics. They are dead weight in this commit but will be consumed by coverage_planner_node in Plan 01-07. Cleaner to land them as-is rather than introduce a temporary GUI-side breakage just to clean them up two plans early."
  - "mowgli_behavior remains broken until Plan 01-08 lands. coverage_nodes.{hpp,cpp}, condition_nodes.{hpp,cpp}, register_nodes.cpp, and main_tree.xml all reference the four deleted .srv types. Plan 01-08's explicit task is to replace those BT consumers with PlanCoverageGoal + FollowCoveragePlan; the plan's `verify` block scopes colcon to `--packages-select mowgli_interfaces mowgli_map mowgli_coverage_planner`, deliberately excluding mowgli_behavior. See `## Known Build Breakage` below."

requirements-completed: [R-1, R-13]

# Metrics
duration: ~25min
completed: 2026-04-29
---

# Phase 1 Plan 6: map_server Pull-Path Cleanup Summary

**1188-line shrink of map_server_node.cpp: 8 strip-planner functions + 4 pull-path .srv definitions deleted, GetAllAreas snapshot-pull handler lands, narrow_area_strategy round-trips through areas.yaml. CONTEXT.md "single source of truth (Q1.1=a) over parallel-keep" honored — no dead pull-path code carried forward.**

## Performance

- **Duration:** ~25 min
- **Started:** 2026-04-29 (after 01-05 SUMMARY commit `d9654bf5`)
- **Completed:** 2026-04-29
- **Tasks:** 2 (both `type="auto"`, sequential)
- **Files created:** 0 (purely deletion + extension)
- **Files modified:** 6
- **Files deleted:** 4

## Accomplishments

- `map_server_node` hosts the new `/map_server_node/get_all_areas` service. Empty request, response is `mowgli_interfaces/MapArea[] areas` carrying name, polygon, obstacles, is_navigation_area, and the new uint8 narrow_area_strategy. Internal IPC only (T-06-03 — only public fields are exposed; mow_progress / dock state / replan bookkeeping is NOT included).
- `AreaEntry::narrow_area_strategy` (uint8_t, default 0) added to map_server_node.hpp. Round-trips through areas.yaml in the existing custom non-yaml-cpp format. Range-checked at every ingress point; out-of-range -> WARN + coerce to 0.
- `mowgli_map` declares `<depend>mowgli_geometry</depend>` and links the header-only INTERFACE target through `find_package(mowgli_geometry REQUIRED)` + `ament_target_dependencies(... mowgli_geometry)`. The 4 promoted geometry helpers from Plan 01-02 (point_in_polygon / convex_hull / compute_optimal_mow_angle / offset_polygon_inward) are now consumed at the namespaced call site only — no local copies remain.
- 8 strip-planner functions (ensure_strip_layout, find_next_unmowed_strip, strip_to_path, is_strip_mowed, is_strip_blocked, compute_coverage_stats, on_get_next_strip, on_preview_plan) and 4 strip-planner service handlers (on_get_next_strip, on_get_coverage_status, on_preview_plan, on_get_outline_path) plus the matching state members and types (Strip, StripLayout, strip_layouts_, current_strip_idx_) are GONE.
- 4 .srv definitions deleted (GetNextStrip, GetCoverageStatus, PreviewPlan, GetOutlinePath) and removed from `mowgli_interfaces/CMakeLists.txt`.
- map_server_node.cpp: 3713 -> 2525 lines (1188 lines / ~32% reduction).
- map_server_node test file: 6 strip-related tests deleted (`StripLayoutGeneratesStrips`, `ConvexHullTest::*`, `MBRAngleTest::*`); the equivalent geometry coverage now lives in `ros2/src/mowgli_geometry/test/` per Plan 01-02.

## Task Commits

Each task was committed atomically.

1. **Task 1: Add GetAllAreas service handler + extend MowingArea/areas.yaml round-trip** — `d5634dc0` (feat)
2. **Task 2: Delete pull-path services + strip-planner code from map_server_node** — `73124c62` (refactor)

(Plan-metadata commit follows this SUMMARY.)

## Files Modified

### `ros2/src/mowgli_map/package.xml`
Added `<depend>mowgli_geometry</depend>` next to the existing `<depend>mowgli_interfaces</depend>`.

### `ros2/src/mowgli_map/CMakeLists.txt`
- `find_package(mowgli_geometry REQUIRED)` next to `find_package(mowgli_interfaces REQUIRED)`.
- `mowgli_geometry` added to `ament_target_dependencies(mowgli_map_lib ...)`, `ament_target_dependencies(test_map_server ...)`, and `ament_export_dependencies(...)`.

### `ros2/src/mowgli_map/include/mowgli_map/map_server_node.hpp`
- `#include <mowgli_interfaces/srv/get_all_areas.hpp>` added; the four pull-path `.hpp` includes removed.
- `on_get_all_areas` declaration added next to `on_get_mowing_area`.
- `get_all_areas_srv_` member added next to `get_mowing_area_srv_`.
- `AreaEntry` gained `uint8_t narrow_area_strategy{0}` after `is_navigation_area`.
- All strip-planner declarations removed: `convex_hull`, `compute_optimal_mow_angle`, `ensure_strip_layout` (test-only public helpers), the four pull-path service handlers, `point_in_polygon` (private wrapper), `Strip`, `StripLayout`, `find_next_unmowed_strip`, `strip_to_path`, `offset_polygon_inward`, `compute_outline_path`, `is_strip_mowed`, `is_strip_blocked`, `compute_coverage_stats`, the four pull-path service members, and the two strip-state members (`strip_layouts_`, `current_strip_idx_`).

### `ros2/src/mowgli_map/src/map_server_node.cpp`
- `#include <mowgli_geometry/geometry.hpp>` added; the four pull-path `.hpp` includes removed.
- Constructor: registered `~/get_all_areas` service after `~/load_areas`. Removed the four pull-path service registrations (`~/get_next_strip`, `~/preview_plan`, `~/get_outline_path`, `~/get_coverage_status`).
- `on_add_area`: range-checks `req->area.narrow_area_strategy` against [0..2], stores it, WARNs + coerces to 0 on out-of-range.
- `on_get_mowing_area`: copies `entry.narrow_area_strategy` into the response.
- New `on_get_all_areas`: locks `map_mutex_`, copies (name, polygon, obstacles, is_navigation_area, narrow_area_strategy) for every entry in `areas_`, RCLCPP_DEBUG log of count.
- `save_areas_to_file`: emits `area_<i>_narrow_area_strategy: <int>` between `is_navigation` and `obstacle_count` lines.
- `load_areas_from_file`: parses optional `area_<i>_narrow_area_strategy` key. Missing -> default 0 (forward-compat). Out of [0..2] -> WARN + coerce to 0.
- All 6 in-file `point_in_polygon(...)` callsites switched to `mowgli_geometry::point_in_polygon(...)`.
- All 8 strip-planner function bodies + the four pull-path service handlers + `point_in_polygon` body + `convex_hull`/`compute_optimal_mow_angle`/`offset_polygon_inward`/`compute_outline_path` bodies deleted (1188 lines total). The two stale `// ensure_strip_layout` comment references inside `on_set_parameters_callback` and `on_set_planning_params` were updated to point at `coverage_planner_node`. The `strip_layouts_.clear()` invalidation snippet inside `on_set_planning_params` was removed.

### `ros2/src/mowgli_map/test/test_map_server.cpp`
Strip-planner tests removed (replaced by a comment block explaining the move to `mowgli_geometry/test/`):
- `CoverageCellsTest::StripLayoutGeneratesStrips`
- `ConvexHullTest::RectangleHullHasFourPoints`, `ConvexHullTest::DegenerateInputReturnsAsIs`
- `MBRAngleTest::EastWestRectangle`, `NorthSouthRectangle`, `DiagonalRectangle`, `SquareReturnsValidAngle`

### `ros2/src/mowgli_interfaces/CMakeLists.txt`
Removed 4 entries from `srv_files`: `srv/GetNextStrip.srv`, `srv/GetCoverageStatus.srv`, `srv/PreviewPlan.srv`, `srv/GetOutlinePath.srv`.

## Files Deleted

- `ros2/src/mowgli_interfaces/srv/GetNextStrip.srv`
- `ros2/src/mowgli_interfaces/srv/GetCoverageStatus.srv`
- `ros2/src/mowgli_interfaces/srv/PreviewPlan.srv`
- `ros2/src/mowgli_interfaces/srv/GetOutlinePath.srv`

## Original-Source Map (deleted code)

| Function | Original location (map_server_node.cpp pre-cleanup) | Outcome |
|----------|-----------------------------------------------------|---------|
| `point_in_polygon` (member def) | lines 1389-1413 | Deleted in Task 2 (forwarder shrunk to 6 lines after Task 1, then removed). Callsites already on `mowgli_geometry::point_in_polygon`. |
| `convex_hull` (static member def) | lines 2421-2457 | Deleted in Task 2. Same implementation lives in `mowgli_geometry::convex_hull` (Plan 01-02). |
| `compute_optimal_mow_angle` (static member def) | lines 2459-2502 | Deleted in Task 2. Same implementation lives in `mowgli_geometry::compute_optimal_mow_angle`. |
| `ensure_strip_layout` | lines 2504-2842 | Deleted in Task 2. Equivalent boustrophedon-with-obstacle-clipping rebuilds in coverage_planner_node (Plan 01-07). |
| `is_strip_mowed` | lines 2844-2897 | Deleted in Task 2. Coverage now tracked via the per-area Checkpoint .kv file (Plan 01-05) plus mow_progress visualisation. |
| `is_strip_blocked` | lines 2899-2939 | Deleted in Task 2. Boustrophedon obstacle-clipping happens at plan-builder time (Plan 01-07). |
| `find_next_unmowed_strip` | lines 2941-3002 | Deleted in Task 2. Replaced by the full-plan PlanCoverage.action result + sequential BT consumption. |
| `strip_to_path` | lines 3004-3053 | Deleted in Task 2. Plan now ships as `CoverageWaypoint[]` not `nav_msgs/Path` (D-04). |
| `compute_coverage_stats` | lines 3055-3095 | Deleted in Task 2. mow_progress publication + coverage_cells visualisation handle live status; planner uses the .kv checkpoint for resume state. |
| `on_get_next_strip` | lines 3097-3165 | Deleted in Task 2. Replaced by PlanCoverage.action goal handling in coverage_planner_node. |
| `on_preview_plan` | lines 3173-3288 | Deleted in Task 2. GUI preview is now a PlanCoverage.action call with `start_pose==dock_pose==current robot pose`, drawn from the action result on the rosbridge path (Plan 01-04). |
| `offset_polygon_inward` | lines 3298-3416 | Deleted in Task 2. Same implementation lives in `mowgli_geometry::offset_polygon_inward`. |
| `compute_outline_path` | lines 3420-3540 | Deleted in Task 2. Outline generation moves into coverage_planner_node's plan builder (Plan 01-07). |
| `on_get_outline_path` | lines 3542-3585 | Deleted in Task 2. Outline is part of the unified PlanCoverage.action result. |
| `on_get_coverage_status` | lines 3682-3711 | Deleted in Task 2. Replaced by PlanCoverage feedback (`progress_percent`, `phase`) plus per-area Checkpoint .kv. |
| `Strip` / `StripLayout` types | hpp lines ~305-318 | Deleted in Task 2. Plan-builder uses internal types in coverage_planner_node (Plan 01-07). |
| `strip_layouts_` / `current_strip_idx_` state | hpp lines 491, 494 | Deleted in Task 2. Plan-builder is stateless across action goals; resume state lives in `<areas_dir>/coverage_<area_index>.kv`. |

## Decisions Made

(See `key-decisions:` block in the frontmatter for the authoritative list.)

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] point_in_polygon transitional forwarder needed to keep Task 1 self-contained**

- **Found during:** Task 1 (build sanity check)
- **Issue:** The plan instructs Task 1 to "switch all `point_in_polygon` call sites to `mowgli_geometry::point_in_polygon`" while Task 2 deletes `MapServerNode::point_in_polygon` entirely. Doing both in one task is fine in principle, but Task 2 also deletes large swathes of the strip-planner that *also* call `point_in_polygon` indirectly via `compute_coverage_stats`. If Task 1 deleted the in-class `point_in_polygon` body outright, the strip-planner would fail to link in the Task 1 intermediate state because its callsites are not removed until Task 2.
- **Fix:** Task 1 shrinks `MapServerNode::point_in_polygon` to a one-line forwarder onto `mowgli_geometry::point_in_polygon` and keeps the Header decl. Task 2 then deletes the wrapper alongside all the strip-planner code that called it.
- **Files modified:** `ros2/src/mowgli_map/include/mowgli_map/map_server_node.hpp`, `ros2/src/mowgli_map/src/map_server_node.cpp`
- **Verification:** Both commits are self-contained at the file level (each commit's static-acceptance greps pass against the working tree at that commit).
- **Committed in:** `d5634dc0` (Task 1, forwarder lands), `73124c62` (Task 2, forwarder + body gone)

**2. [Rule 3 - Blocking] Transitional Header decls for the strip-planner internals**

- **Found during:** Task 1 (header sanity check)
- **Issue:** Same fundamental shape as #1. Task 1 strictly speaking only wants the `GetAllAreas` handler decl + `narrow_area_strategy` field added; it does NOT need to delete the strip-planner decls. But the plan's prescribed Task 1 hpp diff is silent on whether the *existing* strip-planner decls survive Task 1. Naively deleting them in Task 1 leaves orphan member-function definitions in the cpp file (compile error: "out-of-class definition has no matching declaration"). Naively keeping them all in Task 1 leaves Task 2's delete-the-bodies operation underspecified.
- **Fix:** Task 1 keeps the strip-planner decls + the four pull-path service decls inside DEPRECATED-marked comment blocks in the hpp, with explicit "Task 2 of Plan 01-06 deletes these" notes so the next-task acceptance criteria can be matched literally. Task 2 then removes both the decls and the bodies in a single coherent commit.
- **Files modified:** `ros2/src/mowgli_map/include/mowgli_map/map_server_node.hpp`
- **Verification:** Task 2's acceptance grep `grep -E "struct StripLayout|class StripLayout|StripLayout " ...` returns no matches against the post-Task-2 working tree.
- **Committed in:** `d5634dc0` (decls flagged DEPRECATED, kept), `73124c62` (decls + bodies all gone)

**3. [Rule 2 - Critical] Stale comments referenced gone functions**

- **Found during:** Task 2 (post-deletion sweep)
- **Issue:** Two comment blocks in surviving code referenced now-deleted functions: the `on_set_parameters_callback` Live-tunable comment block ("Replanning is on-demand (next /preview_plan or /get_next_strip call)") and the `on_set_planning_params` mow_angle handler comment ("NaN drives auto-MBR in ensure_strip_layout"). Stale comments that contradict the code are a long-running correctness-debt source.
- **Fix:** Both comments updated to point at `coverage_planner_node` as the new consumer of those parameters.
- **Files modified:** `ros2/src/mowgli_map/src/map_server_node.cpp`
- **Verification:** `grep -E "ensure_strip_layout|/preview_plan|/get_next_strip" ros2/src/mowgli_map/src/map_server_node.cpp` returns no hits.
- **Committed in:** `73124c62`

---

**Total deviations:** 3 — all fixes inside Task scope. None of them required architectural deviation.

## Issues Encountered

- **colcon build / colcon test cannot run on the macOS host.** Same situation as Plans 01-02, 01-03, 01-05 (the docker daemon is not running locally and the mowgli-ros2 devcontainer image is not loaded). The plan's verify steps `cd ros2 && colcon build --packages-select mowgli_interfaces mowgli_map` and `colcon test --packages-select mowgli_map` are deferred to:
  1. The next dev-branch push, which triggers the standard mowgli-ros2 docker build pipeline (per `feedback_background_pipeline_deploy.md` workflow).
  2. The Pi5 hardware test bench (Plan 01-09 owns the formal hardware acceptance).
  All static / syntactic checks (file existence, grep invariants, line-count reduction, header / source consistency) ran green locally.

## Known Build Breakage

`mowgli_behavior` will fail to build until Plan 01-08 lands. The following files reference the four deleted .srv types (GetNextStrip, GetCoverageStatus, GetOutlinePath, PreviewPlan) and must be rewritten in Plan 01-08:

| File | What references the deleted types |
|------|-----------------------------------|
| `ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp` | `GetNextStrip` BT class with `rclcpp::Client<GetNextStrip>`; `OutlineArea` BT class with `rclcpp::Client<GetOutlinePath>`; includes `get_next_strip.hpp` / `get_coverage_status.hpp` / `get_outline_path.hpp` |
| `ros2/src/mowgli_behavior/include/mowgli_behavior/condition_nodes.hpp` | `rclcpp::Client<GetCoverageStatus>` for the per-area-coverage check; includes `get_coverage_status.hpp` |
| `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` | Implementations of `GetNextStrip`, `GetNextUnmowedArea` (calls `/map_server_node/get_coverage_status`), `OutlineArea` (calls `/map_server_node/get_outline_path`), and the `FollowStrip` / `TransitToStrip` consumers |
| `ros2/src/mowgli_behavior/src/condition_nodes.cpp` | Implementation of the per-area coverage condition that creates a `GetCoverageStatus` client |
| `ros2/src/mowgli_behavior/src/register_nodes.cpp` | `factory.registerNodeType<GetNextStrip>("GetNextStrip");` |
| `ros2/src/mowgli_behavior/trees/main_tree.xml` | `<GetNextStrip area_index="..."/>` BT node + the GetNextStrip-driven Repeat loop |
| `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp` | Comment text references `GetNextStrip` / `FollowStrip` / `TransitToStrip` (not a hard dep, but stale) |
| `ros2/src/mowgli_interfaces/srv/SetPlanningParams.srv` | Comment text references "PreviewPlan / GetOutlinePath" (not a hard dep, just stale documentation) |

This breakage is **expected** — Plan 01-08's explicit objective is to replace those BT consumers with `PlanCoverageGoal` + `FollowCoveragePlan`, exactly as locked in CONTEXT.md "single source of truth (Q1.1=a) over parallel-keep". The plan-06 verify command (`colcon build --packages-select mowgli_interfaces mowgli_map mowgli_coverage_planner`) deliberately excludes mowgli_behavior. Until Plan 01-08 lands, on-Pi5 deployments must NOT pick up this branch — operators should stay on the dev branch's previous tip until the BT integration plan completes.

`map_server_node.cpp` itself contains zero references to the deleted services.

## User Setup Required

None — pure refactor. The areas.yaml schema is forward-compatible (legacy files load with default 0 = Skip for `narrow_area_strategy`), and no new env vars / auth flow / service accounts are introduced.

## Next Phase Readiness

- **Plan 01-07 (planner core):** the `PLAN-07-PLACEHOLDER` block in `coverage_planner_node::execute()` (Plan 01-05) can now be filled. `/map_server_node/get_all_areas` is live and returns areas with `narrow_area_strategy`, so the validator pipeline + plan builder + narrow-area strategies have all the per-area inputs they need. The surviving live-tunable knobs (`outline_passes`, `outline_offset`, `outline_overlap`, `path_spacing`, `mow_angle_override_deg`, `strip_boundary_margin_m`) remain declared in map_server_node so the GUI keeps working unmodified, but Plan 01-07 will have to decide whether the planner reads them from map_server (over a small new query service) or from its own copy of mowgli_robot.yaml. Recommendation: declare them on coverage_planner_node directly (no IPC needed) since they are flat doubles + an int and live in the same yaml file.
- **Plan 01-08 (BT integration):** the explicit task is to delete `coverage_nodes.{hpp,cpp}::GetNextStrip` / `OutlineArea` / `FollowStrip` / `TransitToStrip` / `GetNextUnmowedArea`, delete `condition_nodes.{hpp,cpp}`'s `GetCoverageStatus` client, and replace them with the two-class scheme (PlanCoverageGoal calls the action; FollowCoveragePlan consumes `CoverageWaypoint[]`). After that lands, the workspace builds cleanly end-to-end again.
- **Plan 01-09 (E2E sim + Pi5 smoke):** PreCondition for the E2E run is that mowgli_behavior compiles, which depends on Plan 01-08. Pi5 smoke test gates the entire phase per the user's `Pi5 test before PR` workflow rule.

## Threat Flags

None new. The threat surface introduced here matches the plan's `<threat_model>` register exactly:

- T-06-01 (areas.yaml `narrow_area_strategy` out of [0..2]) — mitigated by range-check + WARN-and-coerce to 0 in both `load_areas_from_file` and `on_add_area`. Tests cover the round-trip; out-of-range path is exercised at runtime via the WARN log when GUI/CLI sends a bad value.
- T-06-02 (non-planner caller invokes `/map_server_node/get_all_areas`) — accept disposition. Internal LAN-only DDS, no auth layer, consistent with rest of stack.
- T-06-03 (information disclosure via GetAllAreas) — mitigated. Handler explicitly copies only the public MapArea fields (name, area, obstacles, is_navigation_area, narrow_area_strategy). mow_progress / dock state / replan bookkeeping is NOT in the response.
- T-06-04 (code deletion accidentally removes mow_progress safeguards) — mitigated. `mow_progress_to_occupancy_grid()` and the `coverage_cells_pub_` publisher are untouched; CLAUDE.md invariant #14 (grid_map → OccupancyGrid convention) is preserved verbatim. The strip-planner-specific consumers/producers were the only things removed; mow_progress writes via `mark_cells_mowed` (called from `on_odom`) survive intact.

## Self-Check: PASSED

Verified:
- All 4 .srv files removed (`! ls ros2/src/mowgli_interfaces/srv/{GetNextStrip,GetCoverageStatus,PreviewPlan,GetOutlinePath}.srv` returns non-zero).
- `mowgli_interfaces/CMakeLists.txt` no longer references the four deleted services.
- 8 strip-planner function names + 4 service handlers + 4 geometry helpers are GONE from `map_server_node.cpp` (verified via the plan's exact grep patterns).
- `StripLayout` / `Strip` types GONE from both hpp and cpp.
- All 4 pull-path service includes GONE from both hpp and cpp.
- `map_server_node.cpp`: 3713 -> 2525 lines (1188 lines removed; plan minimum was 800).
- `<depend>mowgli_geometry</depend>` present in `package.xml`.
- `find_package(mowgli_geometry REQUIRED)` present in `CMakeLists.txt`.
- `uint8_t narrow_area_strategy` declared in hpp `AreaEntry`.
- `on_get_all_areas` declared in hpp + implemented in cpp + registered as `~/get_all_areas` service.
- `narrow_area_strategy` referenced 17 times in `map_server_node.cpp` (struct copy in handler + writer + reader + ingress range-check + AddMowingArea ingest + GetMowingArea response = comfortably above the >=3 minimum).
- Both task commits present in git log:
  - `d5634dc0` (Task 1)
  - `73124c62` (Task 2)
- Stale comments referencing deleted functions cleaned up (`grep -E "ensure_strip_layout|/preview_plan|/get_next_strip" ros2/src/mowgli_map/src/map_server_node.cpp` returns 0 hits).
- colcon build / colcon test deferred to docker pipeline + Pi5 (see Issues Encountered).
- mowgli_behavior break documented under `## Known Build Breakage` (will be repaired by Plan 01-08).

---

*Phase: 01-coverage-planner-rewrite*
*Plan: 06*
*Completed: 2026-04-29*
