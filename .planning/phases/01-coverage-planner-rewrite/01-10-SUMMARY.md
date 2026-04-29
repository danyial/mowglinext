---
phase: 01-coverage-planner-rewrite
plan: 10
subsystem: interface + planner-core
tags: [ros2, mowgli_interfaces, rosidl, codegen, coverage_planner, tdd, area_index, cr-01]

# Dependency graph
requires:
  - 01-01-mowgli-interfaces (CoverageWaypoint.msg exists)
  - 01-07-planner-core (PlanBuilder exists)
provides:
  - "uint32 area_index field on CoverageWaypoint.msg"
  - "firmware rosserial CoverageWaypoint.h regenerated with area_index"
  - "Go CoverageWaypoint struct with AreaIndex field"
  - "TypeScript CoverageWaypoint interface with area_index field"
  - "PlanBuilder stamps area_index on every emitted waypoint via stamp_and_push"
  - "kNoArea (UINT32_MAX) sentinel for non-area segments"
  - "3 TEST_F cases pinning per-segment-type area_index contract"
affects:
  - 01-11-bt-area-index (consumes area_index in dispatch_checkpoint_write)

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "stamp_and_push lambda: single push path for all per-area waypoints stamps both sequence_id and area_index atomically (T-10-04 mitigation)"
    - "kNoArea = UINT32_MAX sentinel: non-area segments (UNDOCK/RETURN_TO_DOCK/DOCK_APPROACH/DOCKING/start-pose TRANSIT) carry this value; BT in Plan 01-11 skips checkpoint writes for these"

key-files:
  modified:
    - "ros2/src/mowgli_interfaces/msg/CoverageWaypoint.msg — uint32 area_index field added after sequence_id"
    - "firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/CoverageWaypoint.h — regenerated; area_index in serialize() + deserialize() (this->area_index x4)"
    - "gui/pkg/msgs/mowgli/types_generated.go — CoverageWaypoint.AreaIndex uint32 added"
    - "gui/pkg/msgs/mowgli/services_generated.go — regenerated (removed deleted service types)"
    - "gui/web/src/types/ros.generated.ts — area_index?: number added to CoverageWaypoint"
    - "gui/web/src/types/ros.ts — synced from ros.generated.ts; Stamp/Header/TwistStamped helpers re-appended"
    - "ros2/src/mowgli_coverage_planner/src/plan_builder/plan_builder.cpp — kNoArea sentinel + mk_waypoint area_idx param + stamp_and_push lambda"
    - "gui/pkg/api/diagnostics.go — removed dead GetCoverageStatus call (Rule 1)"
    - "gui/pkg/api/mowglinext.go — replaced dead PreviewPlanRoute with 410 Gone stub (Rule 1)"
  created:
    - "ros2/src/mowgli_coverage_planner/test/test_plan_builder_area_index.cpp — 3 TEST_F cases (RED/GREEN)"
    - "ros2/src/mowgli_coverage_planner/test/CMakeLists.txt — ament_add_gtest entry added"

key-decisions:
  - "stamp_and_push lambda over inline assignments: eliminates the N-callsite risk where any new emit path forgets area_index. Single lambda is the only push path for all per-area waypoints."
  - "kNoArea = UINT32_MAX: consistent with the BT contract in Plan 01-11 — BT only writes checkpoints when area_index != UINT32_MAX, so non-area waypoints never produce a coverage_*.kv file."
  - "Navigation-area centroid TRANSIT carries idx (not kNoArea): preserves the canonical owner-index for diagnostics and BT-side filtering even though no checkpoint will ever be written for it."
  - "Resume snap executes before stamp_and_push: the snap overwrites the MOWING_BOUSTROPHEDON pose, then stamp_and_push consumes it. SPEC R-11 5cm/5deg tolerance holds by construction."

# Metrics
duration: 6min
completed: 2026-04-29
---

# Phase 1 Plan 10: CoverageWaypoint area_index + PlanBuilder stamping Summary

**Added uint32 area_index to CoverageWaypoint.msg, regenerated all four binding chains (firmware/Go/TS), and introduced stamp_and_push lambda in PlanBuilder — single-source-of-truth for area_index stamping on every emitted waypoint. Planner-side half of CR-01 (R-9/R-11 gap closure).**

## Performance

- **Duration:** 6 min
- **Started:** 2026-04-29T10:03:57Z
- **Completed:** 2026-04-29T10:09:57Z
- **Tasks:** 2 (Task 1: interface + bindings; Task 2: TDD RED + GREEN)
- **Commits:** 4 (Task 1 feat, Task 2 RED test, Task 2 GREEN impl, docs metadata)

## Accomplishments

- `CoverageWaypoint.msg` now carries `uint32 area_index` at a stable position (after `sequence_id`, before `speed`). The multi-line block comment uses `#` continuation lines which `sync_ros_lib.py`'s comment-strip logic handles correctly.
- All four code-generation chains updated: firmware rosserial (4× `this->area_index` references — 2 serialize + 2 deserialize), Go struct (`AreaIndex uint32 json:"area_index"`), TS generated interface (`area_index?: number`), and `ros.ts` aggregator with helpers preserved.
- `PlanBuilder::build` now stamps `area_index` on every waypoint via a `stamp_and_push` lambda. The lambda is the sole push path for all per-area waypoints (outlines, obstacle outlines, boustrophedon sweep). UNDOCK/TRANSIT-start/RETURN_TO_DOCK/DOCK_APPROACH/DOCKING carry `kNoArea` (UINT32_MAX). Navigation-area centroid TRANSIT carries `idx`.
- TDD RED→GREEN sequence in git log: `36bbbb30` (failing test) → `bd6fda58` (implementation).
- `go build ./...` and `tsc --noEmit` both exit 0.
- colcon build + colcon test deferred to docker/dev-branch CI (macOS host has no ROS toolchain — matches established pattern from Plans 01-07 and 01-09).

## Task Commits

1. **Task 1: CoverageWaypoint.msg + bindings** — `c329a81b` (feat)
2. **Task 2 RED: failing area_index test** — `36bbbb30` (test)
3. **Task 2 GREEN: stamp area_index in PlanBuilder** — `bd6fda58` (feat)

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] Dead Go API callers referencing deleted service types**
- **Found during:** Task 1 Go build verification (`go build ./...`)
- **Issue:** `gui/pkg/api/diagnostics.go:184` called `mowgli.GetCoverageStatusReq/Res` and `gui/pkg/api/mowglinext.go:52` called `mowgli.PreviewPlanReq/Res`. Both types were in `services_generated.go` until this plan's regeneration correctly removed them (the backing `.srv` files `GetCoverageStatus.srv` and `PreviewPlan.srv` were deleted in Plan 01-06). The stale generated file had been masking these dead callers.
- **Fix:** `diagnostics.go` — replaced the coverage status loop with a comment explaining the service is gone and that coverage data will be wired via PlanCoverage.action feedback in a future plan. `mowglinext.go` — replaced `PreviewPlanRoute` implementation with a 410 Gone response (same pattern as the legacy SLAM endpoints). Removed the now-unused `mowgli` import from `diagnostics.go` and the `strconv` import from `mowglinext.go`.
- **Files modified:** `gui/pkg/api/diagnostics.go`, `gui/pkg/api/mowglinext.go`
- **Committed in:** `c329a81b` (Task 1 commit)

## Verification Status

### Host-side (completed this plan)
- `grep -c '^uint32 area_index' CoverageWaypoint.msg` → 1 PASS
- `grep -c 'this->area_index' CoverageWaypoint.h` → 4 PASS (serializer + deserializer wired)
- `grep -c 'AreaIndex' types_generated.go` → 2 PASS
- `grep -c 'area_index' ros.generated.ts` → 2 PASS
- `grep -c 'area_index' ros.ts` → 2 PASS
- `export type Stamp/Header/TwistStamped` helpers intact in ros.ts PASS
- `cd gui && go build ./...` → 0 PASS
- `cd gui/web && tsc --noEmit` → 0 PASS
- `grep -c 'wp\.area_index' plan_builder.cpp` → 2 PASS
- `grep -c 'kNoArea' plan_builder.cpp` → 8 PASS
- RED commit `36bbbb30` present, GREEN commit `bd6fda58` present PASS

### Deferred to docker/dev-branch CI
- `colcon build --packages-select mowgli_interfaces mowgli_coverage_planner`
- `colcon test --packages-select mowgli_coverage_planner --ctest-args -R 'test_plan_builder_area_index|test_resume|test_auto_rotate|test_aabb_sweep|test_outline_generator|test_validation_pipeline|test_narrow_area_strategies|test_segment_type_invariants|test_checkpoint'`

Run command:
```bash
docker exec mowgli-ros2 bash -c '
  source /opt/ros/kilted/setup.bash && cd /ros2_ws &&
  colcon build --packages-select mowgli_interfaces mowgli_coverage_planner \
    --event-handlers console_cohesion+ &&
  colcon test --packages-select mowgli_coverage_planner \
    --ctest-args -R "test_plan_builder_area_index|test_resume|test_auto_rotate" \
    --event-handlers console_cohesion+'
```

## Threat Flags

None — `area_index` is an operator-visible uint32 diagnostic field. +4 bytes per waypoint × ≤500 waypoints = ≤2 KB plan size increase (T-10-03: accepted). The stamp_and_push lambda eliminates T-10-04 (single push path).

## Self-Check: PASSED

- `ros2/src/mowgli_interfaces/msg/CoverageWaypoint.msg` — FOUND
- `firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/CoverageWaypoint.h` — FOUND
- `gui/pkg/msgs/mowgli/types_generated.go` — FOUND (AreaIndex)
- `gui/web/src/types/ros.ts` — FOUND (area_index + helpers)
- `ros2/src/mowgli_coverage_planner/src/plan_builder/plan_builder.cpp` — FOUND (kNoArea + stamp_and_push)
- `ros2/src/mowgli_coverage_planner/test/test_plan_builder_area_index.cpp` — FOUND (3 TEST_F)
- Commit `c329a81b` (Task 1) — FOUND
- Commit `36bbbb30` (RED) — FOUND
- Commit `bd6fda58` (GREEN) — FOUND

---

*Phase: 01-coverage-planner-rewrite*
*Plan: 10*
*Completed: 2026-04-29*
