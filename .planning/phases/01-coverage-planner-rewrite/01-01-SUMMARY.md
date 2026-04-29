---
phase: 01-coverage-planner-rewrite
plan: 01
subsystem: api
tags: [ros2, mowgli_interfaces, rosidl, rosserial, codegen, action, srv, msg, typescript, golang]

# Dependency graph
requires: []
provides:
  - "PlanCoverage.action goal/result/feedback per SPEC R-2 (start_pose, dock_pose, mow_angle_offset_deg, resume_from_checkpoint; result success+plan+metadata+error; feedback progress+phase)"
  - "CoverageWaypoint.msg with 8 segment_type constants (UNDOCK..DOCKING)"
  - "Checkpoint.msg per-area resume state (area_index, swath indices, last_mow_angle_deg, last_swath_endpoint)"
  - "PlanError.msg with 8 structured error codes (NO_AREAS..INTERNAL)"
  - "PlanMetadata.msg with checkpoint_seed embedding"
  - "GetAllAreas.srv (empty req, MapArea[] res) for snapshot pull"
  - "WriteCheckpoint.srv for BT-delegates-checkpoint-IO-to-planner pattern"
  - "MapArea.narrow_area_strategy uint8 (0=SKIP, 1=OUTLINE_ONLY, 2=SPECIAL_PATTERN)"
  - "Regenerated firmware/Go/TS bindings consuming all of the above"
  - "Schema-version note: 01-01 is the contract foundation. All Wave-2..Wave-5 plans compile against these types."
affects:
  - 01-02-mowgli-geometry-library
  - 01-04-gui-integration
  - 01-05-coverage-planner-skeleton
  - 01-06-map-server-cleanup
  - 01-07-planner-core
  - 01-08-bt-integration
  - 01-09-e2e-sim-and-pi5-smoke

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Service-based delegation: BT FollowCoveragePlan → coverage_planner_node WriteCheckpoint.srv (resolves RESEARCH §10 Q1; planner owns filesystem I/O)"
    - "Snapshot-pull retrieval: coverage_planner_node calls GetAllAreas.srv once per Action goal for snapshot consistency"

key-files:
  created:
    - "ros2/src/mowgli_interfaces/msg/CoverageWaypoint.msg"
    - "ros2/src/mowgli_interfaces/msg/PlanMetadata.msg"
    - "ros2/src/mowgli_interfaces/msg/PlanError.msg"
    - "ros2/src/mowgli_interfaces/msg/Checkpoint.msg"
    - "ros2/src/mowgli_interfaces/srv/GetAllAreas.srv"
    - "ros2/src/mowgli_interfaces/srv/WriteCheckpoint.srv"
  modified:
    - "ros2/src/mowgli_interfaces/msg/MapArea.msg"
    - "ros2/src/mowgli_interfaces/action/PlanCoverage.action"
    - "ros2/src/mowgli_interfaces/CMakeLists.txt"
    - "firmware/scripts/sync_ros_lib.py"
    - "firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/MapArea.h"
    - "firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/Emergency.h"
    - "gui/generate_go_msgs.sh"
    - "gui/generate_ts_types.sh"
    - "gui/pkg/msgs/mowgli/types_generated.go"
    - "gui/pkg/msgs/mowgli/services_generated.go"
    - "gui/web/src/types/ros.ts"
    - "gui/web/src/types/ros.generated.ts"

key-decisions:
  - "WriteCheckpoint.srv locks the BT-delegates-IO-to-planner pattern, resolving RESEARCH §10 Q1 (Should BT or planner own .kv writes?). Planner owns filesystem I/O so the atomic write helper lives in one place."
  - "GetAllAreas.srv is internal IPC only (planner ↔ map_server). Existing per-index GetMowingArea.srv is kept for the GUI, sidestepping the foxglove_bridge typesupport bug."
  - "PlanCoverage.action old BCD-style schema discarded entirely. New goal carries start_pose/dock_pose/mow_angle_offset_deg/resume_from_checkpoint per SPEC R-2; result carries the sparse plan + metadata + structured error."

patterns-established:
  - "Pattern: ROS2 fully-qualified type references (mowgli_interfaces/Foo) inside .msg/.srv files. Both Go and TS generators now strip the package prefix correctly for same-package references."
  - "Pattern: Inline-comment-tolerant .msg parser. firmware/scripts/sync_ros_lib.py now handles `type name # comment` lines."

requirements-completed: [R-2, R-4, R-5, R-12, R-13]

# Metrics
duration: 12min
completed: 2026-04-29
---

# Phase 1 Plan 1: mowgli_interfaces Extensions Summary

**Coverage-planner contract surface locked: PlanCoverage.action rewrite + 4 new .msg + 2 new .srv + MapArea narrow-area-strategy field + regenerated firmware/Go/TS bindings.**

## Performance

- **Duration:** 12 min
- **Started:** 2026-04-29T05:28:03Z
- **Completed:** 2026-04-29T05:40:04Z
- **Tasks:** 3
- **Files created:** 6 (4 .msg, 2 .srv)
- **Files modified:** 12 (incl. firmware headers, Go/TS bindings, codegen scripts)

## Accomplishments

- All Wave-2..Wave-5 plans now have a stable interface contract to compile against.
- 8 segment_type constants (CoverageWaypoint) and 8 error_code constants (PlanError) match SPEC R-4 / R-12 verbatim.
- Atomic checkpoint persistence schema (Checkpoint.msg) covers the SPEC R-10 field set; persisted as `coverage_<area_index>.kv` per D-05/D-06.
- BT → planner checkpoint IO pattern locked via WriteCheckpoint.srv (closes RESEARCH §10 Q1).
- All three downstream code-generation chains (firmware rosserial, Go, TS) regenerate cleanly. `cd gui && go build ./...` and `cd gui/web && tsc --noEmit` exit 0.
- Three latent generator bugs fixed in scope (see Deviations).

## Task Commits

Each task was committed atomically:

1. **Task 1: Add 4 new .msg files + extend MapArea.msg** — `27cae866` (feat)
2. **Task 2: Rewrite PlanCoverage.action + add 2 .srv files + update CMakeLists** — `dd19d5c9` (feat)
3. **Task 3: Regenerate firmware rosserial + Go + TS bindings (incl. generator bug fixes)** — `d39255c4` (feat)

## Files Created/Modified

### Created
- `ros2/src/mowgli_interfaces/msg/CoverageWaypoint.msg` — Per-waypoint pose + segment_type with 8 enum constants.
- `ros2/src/mowgli_interfaces/msg/PlanMetadata.msg` — Plan-level metadata; embeds Checkpoint as checkpoint_seed.
- `ros2/src/mowgli_interfaces/msg/PlanError.msg` — Structured error with 8 error codes + failed_validation_point + affected polygons/area indices.
- `ros2/src/mowgli_interfaces/msg/Checkpoint.msg` — Per-area resume state (swath indices, mow angle, swath endpoint) for `.kv` persistence.
- `ros2/src/mowgli_interfaces/srv/GetAllAreas.srv` — Snapshot pull (empty req, MapArea[] response).
- `ros2/src/mowgli_interfaces/srv/WriteCheckpoint.srv` — BT-to-planner checkpoint write delegation.

### Modified
- `ros2/src/mowgli_interfaces/msg/MapArea.msg` — Added `uint8 narrow_area_strategy` (0=SKIP, 1=OUTLINE_ONLY, 2=SPECIAL_PATTERN per SPEC R-13).
- `ros2/src/mowgli_interfaces/action/PlanCoverage.action` — Full rewrite: old BCD-style fields replaced with start_pose/dock_pose/mow_angle_offset_deg/resume_from_checkpoint; result carries CoverageWaypoint[] plan + PlanMetadata + PlanError.
- `ros2/src/mowgli_interfaces/CMakeLists.txt` — Registered 4 new .msg files and 2 new .srv files in rosidl_generate_interfaces.
- `firmware/scripts/sync_ros_lib.py` — Bug fix (Rule 1): parser now strips inline comments before regex match.
- `firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/*.h` — Regenerated for all touched messages; new headers for CoverageWaypoint/Checkpoint/PlanError/PlanMetadata/PlanningParams/CalibrateImuYawStatus.
- `gui/generate_go_msgs.sh` — Bug fix (Rule 1): added `mowgli_interfaces/*` case branch in `ros_to_go()` to strip same-package prefix.
- `gui/generate_ts_types.sh` — Bug fix (Rule 1): same fix for TS generator.
- `gui/pkg/msgs/mowgli/types_generated.go` + `services_generated.go` — Regenerated.
- `gui/web/src/types/ros.ts` + `ros.generated.ts` — Regenerated; added 3 hand-curated helper types (Stamp, Header, TwistStamped) that the generator does not emit.

## Schema-version note for downstream waves

The interface surface is now stable. Downstream plans (01-02 onwards) may reference:

- `mowgli_interfaces/action/PlanCoverage` with the SPEC R-2 schema only — no fallback to the BCD schema.
- `mowgli_interfaces/msg/CoverageWaypoint::SEGMENT_*` constants for blade + speed + sub-action dispatch.
- `mowgli_interfaces/msg/MapArea::narrow_area_strategy` for per-area narrow-area handling.
- `mowgli_interfaces/srv/GetAllAreas` (planner-side) and `mowgli_interfaces/srv/WriteCheckpoint` (BT-side) for the snapshot/persistence contract.

The `mow_progress.png`-adjacent `coverage_<area_index>.kv` file format is owned by the planner (Plan 01-05 / 01-07) and is private to it; BT only writes via the service.

## Decisions Made

- **PlanCoverage.action: discard the old BCD schema entirely (no compat shim).** The old goal (`outer_boundary`/`obstacles`/`mow_angle_deg`/`skip_outline`/`visited_points`) is completely replaced. No clients of the old schema exist on the dev branch yet (per CONTEXT.md "removal-hardening parallel-fallback period — pull-path is deleted, not deprecated"), so a clean rewrite is safer than a versioned shim.
- **WriteCheckpoint.srv lives in mowgli_interfaces, not mowgli_coverage_planner.** All ROS2 interfaces are centralized in mowgli_interfaces, matching the existing convention.
- **GetAllAreas.srv response field is `mowgli_interfaces/MapArea[] areas`, not bare `MapArea[]`.** Fully-qualified package prefix matches the convention used by PlanCoverage.action and PlanMetadata.msg, and (as a Rule-1 side-effect) exposed the latent same-package generator bugs that we then fixed.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] firmware/scripts/sync_ros_lib.py silently dropped fields with inline comments**
- **Found during:** Task 3 (firmware rosserial regeneration)
- **Issue:** `parse_msg()` matched fields with the regex `^(\S+?)(\[\])?\s+(\w+)$`, which is anchored at end-of-line. Any field declared as `type name  # comment` failed to match and was silently skipped. This caused MapArea.narrow_area_strategy to be missing from the regenerated firmware header — directly violating Task 3's acceptance criterion `grep -q "narrow_area_strategy" .../MapArea.h`. The same bug had been silently dropping `lift_warning` and `lift_duration_sec` from `Emergency.msg` for an unknown amount of time before this plan.
- **Fix:** Added a single-line comment-strip step before the field regex (`hash_idx = line.find("#"); if hash_idx >= 0: line = line[:hash_idx]`).
- **Files modified:** `firmware/scripts/sync_ros_lib.py`
- **Verification:** `python3 -c "import sync_ros_lib; sync_ros_lib.parse_msg(MapArea.msg)"` now returns 5 fields including `narrow_area_strategy`. Re-running the script produces `Emergency.h` with the previously-missing `lift_warning` + `lift_duration_sec` fields, which is a quiet pre-existing bug-fix bonus.
- **Committed in:** `d39255c4` (Task 3 commit)

**2. [Rule 1 - Bug] gui/generate_go_msgs.sh emitted invalid Go identifiers for same-package fully-qualified type references**
- **Found during:** Task 3 (Go build verification)
- **Issue:** `ros_to_go()` had no case branch for `mowgli_interfaces/*`. Cross-msg references inside mowgli_interfaces (`PlanMetadata.checkpoint_seed: mowgli_interfaces/Checkpoint`, `GetAllAreas.areas: mowgli_interfaces/MapArea[]`, `WriteCheckpoint.checkpoint: mowgli_interfaces/Checkpoint`) fell through to the bare `*)` default and emitted Go fields like `mowgli_interfaces/Checkpoint` — invalid identifier with `/` in it.
- **Fix:** Added `mowgli_interfaces/*)` case in the case statement that strips the prefix and yields the bare type name (same-package reference within the `mowgli` Go package).
- **Files modified:** `gui/generate_go_msgs.sh`, `gui/pkg/msgs/mowgli/{types,services}_generated.go` (regenerated)
- **Verification:** `cd gui && go build ./...` exits 0 (previously: 3 syntax errors).
- **Committed in:** `d39255c4` (Task 3 commit)

**3. [Rule 1 - Bug] gui/generate_ts_types.sh had identical missing-case bug**
- **Found during:** Task 3 (TS typecheck verification)
- **Issue:** Identical pattern to bug #2 — `ros_to_ts()` had no `mowgli_interfaces/*` case, so `checkpoint_seed?: mowgli_interfaces/Checkpoint` slipped through and broke TS parsing.
- **Fix:** Same pattern: added `mowgli_interfaces/*)` case stripping the prefix.
- **Files modified:** `gui/generate_ts_types.sh`, `gui/web/src/types/ros.generated.ts`, `gui/web/src/types/ros.ts` (regenerated/synced)
- **Verification:** `cd gui/web && tsc --noEmit` exits 0 after fix.
- **Committed in:** `d39255c4` (Task 3 commit)

**4. [Rule 3 - Blocking] gui/web/src/types/ros.ts vs ros.generated.ts mismatch + missing helper types**
- **Found during:** Task 3 (TS typecheck verification)
- **Issue:** The TS generator writes to `ros.generated.ts`, but the entire GUI imports from `ros.ts` (`import {...} from "../types/ros.ts"` in ~25 files). `ros.ts` was the old generator's output target, never migrated. After regeneration, `ros.ts` was stale (missing `narrow_area_strategy`, `CoverageWaypoint`, etc.). Fixing this required syncing `ros.ts` from `ros.generated.ts`. Additionally, the old `ros.ts` had three hand-curated helper types — `Stamp`, `Header`, `TwistStamped` — that the generator does not emit (it scans only `mowgli_interfaces/`, not `geometry_msgs/`). A naive `cp` lost them and broke `useManualMode.ts` which imports `TwistStamped`.
- **Fix:** `cp ros.generated.ts ros.ts`, then re-add the 3 helper types in a clearly-marked "manually-maintained helpers" section at the bottom of `ros.ts`.
- **Files modified:** `gui/web/src/types/ros.ts`
- **Verification:** `cd gui/web && tsc --noEmit` exits 0; all 25+ consumers compile.
- **Committed in:** `d39255c4` (Task 3 commit)

---

**Total deviations:** 4 auto-fixed (3 latent generator bugs + 1 blocking generator-output target mismatch)
**Impact on plan:** All 4 fixes were strictly required to satisfy the plan's downstream-build acceptance criteria. Bugs #1-#3 are pre-existing latent issues that this plan exposed by becoming the first downstream consumer with cross-message refs and inline comments — not regressions caused by this plan. Bug #4 is a pre-existing generator-output-target drift that will need a follow-up cleanup (see Next Phase Readiness).

## Issues Encountered

- **Plan-doc path drift:** Plan referenced `gui/pkg/msgs/mowgli_interfaces/msg/types_generated.go` (the Go output target), but the actual generator writes to `gui/pkg/msgs/mowgli/types_generated.go` and `…/services_generated.go`. Same for the TS path: plan referenced `gui/web/src/types/ros.ts`, but the generator writes `…/ros.generated.ts`. Verified the actual semantics of the criterion (new types reach the consumed file) rather than the literal path text.

## User Setup Required

None — no external service configuration required for this interface-only plan.

## Next Phase Readiness

- All Wave-2..Wave-5 plans have a stable contract.
- **Outstanding cleanup (out of scope here, recommend a follow-up plan):** The `gui/web/src/types/ros.ts` vs `ros.generated.ts` split is fragile. Two reasonable options:
  1. Make `ros.ts` a one-line `export * from "./ros.generated.ts"` re-export, and move the 3 hand-curated helpers into `ros.generated.ts` via a generator extension.
  2. Update `generate_ts_types.sh` to write directly to `ros.ts` and inline-emit the three helpers.
  This isn't blocking — the GUI compiles cleanly today — but the next time the generator changes, the same drift can recur. Tracked as a deferred item.
- Pi5 hardware test is irrelevant for this plan (interface-only, no runtime behavior change).

## Threat Flags

None — this is a pure schema-definition plan. Threat-model T-01-01..T-01-04 are mitigations to be enforced in Plan 01-05 (validator pipeline) and Plan 01-07 (planner core). The schema landed here only enables those mitigations.

## Self-Check: PASSED

Verified:
- All 6 created files present at expected paths.
- All 3 task commits present in git log:
  - `27cae866` (Task 1)
  - `dd19d5c9` (Task 2)
  - `d39255c4` (Task 3)
- `colcon build --packages-select mowgli_interfaces` exits 0 (verified via mowgli-ros2 docker container).
- `cd gui && go build ./...` exits 0.
- `cd gui/web && ./node_modules/.bin/tsc --noEmit` exits 0.
- `git diff --stat firmware/stm32/ros_usbnode/include/mowgli_protocol.h` shows 0 lines changed.

---

*Phase: 01-coverage-planner-rewrite*
*Plan: 01*
*Completed: 2026-04-29*
