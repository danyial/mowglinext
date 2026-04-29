---
phase: 01-coverage-planner-rewrite
plan: 05
subsystem: ros2
tags: [ros2, mowgli_coverage_planner, rclcpp_action, checkpoint, kv, atomic-write, gtest, plan-07-placeholder]

# Dependency graph
requires:
  - phase: 01-coverage-planner-rewrite
    provides: "Plan 01-01 froze PlanCoverage.action / GetAllAreas.srv / WriteCheckpoint.srv / Checkpoint.msg / PlanError.msg / CoverageWaypoint.msg / PlanMetadata.msg interfaces. This plan implements the action server that serves them."
  - phase: 01-coverage-planner-rewrite
    provides: "Plan 01-02 shipped mowgli_geometry header-only library with FootprintParams + atomic_write. on_write_checkpoint delegates the 4-step atomic write to mowgli_geometry::atomic_write."
  - phase: 01-coverage-planner-rewrite
    provides: "Plan 01-03 added the robot_geometry: section to mowgli_robot.yaml with 7 leaf params. coverage_planner_node declares those params with matching defaults."
provides:
  - "ros2/src/mowgli_coverage_planner/ — new package with the coverage_planner_node executable (rclcpp_action server)"
  - "rclcpp_action server at /coverage_planner_node/plan_coverage; rejects concurrent goals via planning_active_ atomic; runs execute() on a detached worker thread per RESEARCH §6.1"
  - "GetAllAreas client at /map_server_node/get_all_areas (5 s timeout, cancel-aware poll loop) — populates PlanContext.areas in the snapshot pull"
  - "WriteCheckpoint service at /coverage_planner_node/write_checkpoint backed by checkpoint_io.cpp + mowgli_geometry::atomic_write"
  - "Checkpoint .kv key=value serializer / parser per RESEARCH §7.1: 9 keys, FORWARD|REVERSE direction tokens, 6-decimal floats, area_index encoded only in the filename"
  - "PLAN-07-PLACEHOLDER marker in execute() — Plan 01-07 replaces it with the validator pipeline + plan builder"
  - "8 gtest TEST cases in test_checkpoint covering round-trip, REVERSE direction token, garbage / missing-key / non-numeric / unknown-direction parser rejection, atomic-write happy path (no .tmp orphan, 9 lines), NaN endpoint rejection (T-05-02), missing-file -> nullopt, corrupted-file -> nullopt (gateway to ERROR_RESUME_CHECKPOINT_INVALID)"
  - "2 gtest TEST_F cases in test_coverage_planner_skeleton: node-construction smoke + /coverage_planner_node/plan_coverage action endpoint live within 2 s (R-1 + R-2 acceptance)"

affects:
  - 01-06-map-server-cleanup
  - 01-07-planner-core
  - 01-08-bt-integration
  - 01-09-e2e-sim-and-pi5-smoke

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "rclcpp_action server skeleton: handle_goal REJECT-on-busy + ACCEPT_AND_EXECUTE / handle_cancel ACCEPT / handle_accepted spawns detached std::thread for execute(). Matches RESEARCH §6.1 verbatim."
    - "Service-poll-without-spin in execute(): async_send_request + future.wait_for + cancel-check loop with 5 s deadline. Mirrors coverage_nodes.cpp:64-86 idiom; replaces the legacy GetNextStrip pattern."
    - "Plan-context phase pipeline: PlanContext struct passed phase-by-phase through execute(). Plan 01-07 will extend with intermediate per-area outline / sweep state."
    - "BT-delegates-IO-to-planner: WriteCheckpoint.srv handler builds <areas_dir>/coverage_<area_index>.kv and atomic_writes via mowgli_geometry. BT never touches the filesystem (Q1 lock from RESEARCH §10)."
    - "PLAN-07-PLACEHOLDER marker pattern: explicit grep-able TODO block in execute() so Plan 01-07's task can locate the extension point cleanly."

key-files:
  created:
    - "ros2/src/mowgli_coverage_planner/package.xml"
    - "ros2/src/mowgli_coverage_planner/CMakeLists.txt"
    - "ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/coverage_planner_node.hpp"
    - "ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/plan_context.hpp"
    - "ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/checkpoint_io.hpp"
    - "ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp"
    - "ros2/src/mowgli_coverage_planner/src/main.cpp"
    - "ros2/src/mowgli_coverage_planner/src/checkpoint_io.cpp"
    - "ros2/src/mowgli_coverage_planner/test/CMakeLists.txt"
    - "ros2/src/mowgli_coverage_planner/test/test_checkpoint.cpp"
    - "ros2/src/mowgli_coverage_planner/test/test_coverage_planner_skeleton.cpp"
  modified: []

key-decisions:
  - "execute() short-circuits with ERROR_INTERNAL inside an explicit PLAN-07-PLACEHOLDER block. The action plumbing is end-to-end live (the GUI / BT will receive a structured failure rather than a timeout) until Plan 01-07 lands the validator pipeline + plan builder. The marker is a literal grep target, documented in Task 1 acceptance criteria."
  - "Checkpoint .kv body does NOT carry area_index — that field is encoded only in the filename. T-05-01 (path traversal) is mitigated by the uint32 -> std::to_string interpolation, which produces digit-only output. The parser leaves area_index zero; the file-level read_checkpoint_file fills it from the caller's argument."
  - "swath_direction serialises as FORWARD/REVERSE string tokens (not numeric uint8) so the .kv file is human-inspectable in the field. Parser maps unknown tokens to nullopt (test_checkpoint.RejectsUnknownSwathDirectionToken)."
  - "Yaw is serialised as a single scalar (last_swath_endpoint_yaw=) extracted via tf2::getYaw. Parser reconstructs the quaternion via tf2::Quaternion::setRPY(0, 0, yaw) so the round-trip preserves orientation while keeping the file shape simple."
  - "T-05-02 NaN guard lives in write_checkpoint_file (rejects non-finite endpoint x/y + mow_angle_deg before serialising) rather than in serialize_checkpoint, because serialize is also used by the round-trip test where we want it to faithfully echo whatever it gets. The write_checkpoint_file layer is the single trust boundary on the BT->planner path."
  - "Tests compile against the static lib (mowgli_coverage_planner_lib), not the executable, mirroring mowgli_map's pattern. test_coverage_planner_skeleton spins the node on a SingleThreadedExecutor in a worker thread and queries the action endpoint from a fresh client node."

requirements-completed: [R-1, R-2, R-10, R-11, R-12]

# Metrics
duration: 8min
completed: 2026-04-29
---

# Phase 1 Plan 5: mowgli_coverage_planner Skeleton Summary

**Coverage-planner skeleton lands: rclcpp_action server + GetAllAreas client + WriteCheckpoint service handler + 9-key Checkpoint .kv I/O + 10 unit tests, all wired to the locked Plan 01-01 contract surface and backed by Plan 01-02's atomic_write helper. Plan 01-07 has a clean PLAN-07-PLACEHOLDER block to fill with validators + plan builder.**

## Performance

- **Duration:** 8 min
- **Started:** 2026-04-29T06:43:12Z
- **Completed:** 2026-04-29T06:51:19Z
- **Tasks:** 2 (1 + 1 TDD-paired)
- **Files created:** 11 (package.xml, CMakeLists.txt, 3 headers, 3 sources, test/CMakeLists.txt, 2 test files)
- **Files modified:** 0 (purely additive)

## Accomplishments

- New ROS2 package `mowgli_coverage_planner` ready to build (`colcon build --packages-select mowgli_coverage_planner` will exercise on the next docker rebuild / Pi5 deploy — see "Issues Encountered").
- `coverage_planner_node` exposes the full action / service / client surface that Plan 01-08's BT integration and Plan 01-04's GUI hook will talk to:
  - rclcpp_action server `/coverage_planner_node/plan_coverage` accepting `PlanCoverage.action`.
  - rclcpp::Client for `/map_server_node/get_all_areas` (Plan 01-06 will land the server side).
  - rclcpp::Service `/coverage_planner_node/write_checkpoint` for BT-delegates-IO-to-planner.
- Action plumbing is end-to-end functional for the empty-areas + invalid-geometry paths today:
  - `robot_geometry.robot_length / robot_width / tool_width <= 0` -> ERROR_INTERNAL on every plan request.
  - GetAllAreas timeout -> ERROR_INTERNAL.
  - Empty area inventory -> ERROR_NO_AREAS.
  - Cancel during fetch_all_areas -> goal_handle->canceled() within the poll-loop iteration.
- Checkpoint .kv format implemented per RESEARCH §7.1:
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
  ```
  9 lines, no trailing area_index (filename-encoded), no yaml-cpp.
- 10 gtest cases land (test_checkpoint x8 + test_coverage_planner_skeleton x2), exceeding the plan's "≥ 4 tests" minimum.

## Task Commits

Each task was committed atomically; Task 2 used the TDD RED -> GREEN gate sequence.

1. **Task 1: Bootstrap package + action-server skeleton + GetAllAreas client + parameter validation** — `4bce4424` (feat)
2. **Task 2 RED: failing tests for checkpoint .kv + skeleton** — `72a4d925` (test)
3. **Task 2 GREEN: implement Checkpoint .kv I/O + wire WriteCheckpoint handler** — `14e51df4` (feat)

## Files Created

### Build glue

- `ros2/src/mowgli_coverage_planner/package.xml` — buildtool_depend ament_cmake; runtime depends on rclcpp / rclcpp_action / mowgli_interfaces / mowgli_geometry / nav_msgs / nav2_msgs / geometry_msgs / tf2 / tf2_geometry_msgs / eigen; test_depend on ament_cmake_gtest + lint_auto/common.
- `ros2/src/mowgli_coverage_planner/CMakeLists.txt` — `mowgli_coverage_planner_lib` STATIC library (so gtest can link without spinning up the executable) + `coverage_planner_node` executable + BUILD_TESTING add_subdirectory(test). Links `mowgli_geometry` (plain target name, matching the geometry library's export form).
- `ros2/src/mowgli_coverage_planner/test/CMakeLists.txt` — 2 `ament_add_gtest` entries (test_checkpoint linked against the lib only; test_coverage_planner_skeleton additionally pulls rclcpp / rclcpp_action via `ament_target_dependencies`).

### Headers

- `ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/coverage_planner_node.hpp` — `class CoveragePlannerNode : public rclcpp::Node` declaring the action server / GetAllAreas client / WriteCheckpoint service members and the parameter-loading helpers.
- `ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/plan_context.hpp` — `RobotGeometry` (FootprintParams + outline/sweep knobs) + `PlanContext` struct passed phase-by-phase through `execute()`. Plan 01-07 extends with intermediate per-area state (rotated polygons, swath segments, narrow-area fallout).
- `ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/checkpoint_io.hpp` — declares `serialize_checkpoint`, `parse_kv`, `write_checkpoint_file`, `read_checkpoint_file` plus a header comment block summarising the threat model (T-05-01 .. T-05-04).

### Sources

- `ros2/src/mowgli_coverage_planner/src/main.cpp` — standard `rclcpp::init` -> `rclcpp::spin` -> `rclcpp::shutdown` boilerplate.
- `ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp` — constructor declares all 12 parameters + creates the action server / service / client, `execute()` runs the phase pipeline (geometry validate -> areas_loaded feedback -> fetch_all_areas with cancel poll -> empty-areas guard -> PLAN-07-PLACEHOLDER tail), `on_write_checkpoint` delegates to `write_checkpoint_file`.
- `ros2/src/mowgli_coverage_planner/src/checkpoint_io.cpp` — implements all four helpers per RESEARCH §7.1 + §7.2 (atomic_write via mowgli_geometry, yaw via tf2::getYaw / tf2::Quaternion::setRPY).

### Tests

- `ros2/src/mowgli_coverage_planner/test/test_checkpoint.cpp` — 8 TEST / TEST_F cases.
- `ros2/src/mowgli_coverage_planner/test/test_coverage_planner_skeleton.cpp` — 2 TEST_F cases.

## Plan-07 Extension Point

The `execute()` body in `coverage_planner_node.cpp` contains:

```cpp
  // 5. PLAN-07-PLACEHOLDER -----------------------------------------------------
  //    Plan 01-07 replaces everything from here through the goal_handle->succeed
  //    call below with the validator pipeline + outline generator + boustrophedon
  //    sweep + narrow-area strategies + plan-metadata population. Until then we
  //    short-circuit with ERROR_INTERNAL ...
  {
    result->success = false;
    PlanError err;
    err.error_code = PlanError::ERROR_INTERNAL;
    err.human_readable =
        "Plan builder not yet implemented (Plan 01-07 lands this)";
    result->error = err;
    goal_handle->succeed(result);
    planning_active_.store(false);
    return;
  }
  // ----------------------------------------------- PLAN-07-PLACEHOLDER end ---
```

Plan 01-07's task is to replace the body between the `// 5.` line and the `// ----- PLAN-07-PLACEHOLDER end ---` line with the validator pipeline + outline generator + boustrophedon sweep + narrow-area strategies + checkpoint resume + PlanMetadata population. Everything around it (action plumbing, GetAllAreas pull, robot-geometry validation, ERROR_NO_AREAS guard, planning_active_ teardown) stays as-is.

## Test Coverage Map

### test_checkpoint (8 cases)

| Case | Behaviour | Threat model |
|------|-----------|--------------|
| `CheckpointSerialize.RoundTrip` | Populated Checkpoint -> serialize -> parse_kv -> all fields equal within 1e-5; yaw round-trips through quaternion <-> scalar conversion | — |
| `CheckpointSerialize.ReverseDirectionToken` | swath_direction = REVERSE serialises to "swath_direction=REVERSE" and parses back to SWATH_DIRECTION_REVERSE | — |
| `CheckpointParse.RejectsGarbage` | parse_kv("garbage with no equals") returns nullopt | T-05-03 |
| `CheckpointParse.RejectsMissingRequiredKey` | Stripping last_mow_angle_deg line -> parse_kv returns nullopt | T-05-03 (gateway to ERROR_RESUME_CHECKPOINT_INVALID) |
| `CheckpointParse.RejectsNonNumericFloat` | "last_mow_angle_deg=not-a-number" -> parse_kv returns nullopt | T-05-03 |
| `CheckpointParse.RejectsUnknownSwathDirectionToken` | "swath_direction=SIDEWAYS" -> parse_kv returns nullopt | T-05-03 |
| `CheckpointFsTest.AtomicWriteAndReadBack` | write_checkpoint_file -> file exists, no .tmp orphan, exactly 9 newlines; read_checkpoint_file round-trips identical | T-05-04 (atomic_write) |
| `CheckpointFsTest.RejectsNonFiniteCheckpointValues` | NaN endpoint x -> write_checkpoint_file returns false, no file or .tmp created | T-05-02 |
| `CheckpointFsTest.ReadCheckpointMissingFileReturnsNullopt` | read_checkpoint_file on absent file -> nullopt | T-05-03 |
| `CheckpointFsTest.ReadCheckpointCorruptedFileReturnsNullopt` | Hand-written garbage file -> read_checkpoint_file returns nullopt | T-05-03 (gateway to ERROR_RESUME_CHECKPOINT_INVALID) |

(Two of these are TEST_F variants under `CheckpointFsTest` which provides a per-test temp directory — counted as separate cases by the test runner.)

### test_coverage_planner_skeleton (2 cases)

| Case | Behaviour |
|------|-----------|
| `CoveragePlannerSkeletonTest.NodeConstructsCleanly` | Node constructor with full parameter overrides finishes without throwing; `get_name()` is "coverage_planner_node". |
| `CoveragePlannerSkeletonTest.ActionEndpointIsAvailable` | An external client node calls `rclcpp_action::create_client<PlanCoverage>(..., "/coverage_planner_node/plan_coverage")` and reaches `action_server_is_ready()` within 2 s. |

## Decisions Made

(See `key-decisions:` block in the frontmatter for the authoritative list. Highlights:)

- **PLAN-07-PLACEHOLDER as a literal grep target** — Plan 01-07's executor will literally `grep -n PLAN-07-PLACEHOLDER ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp` to find the extension point. The marker also appears in this SUMMARY's frontmatter (`provides:`) so `gsd-sdk` queries surface it.
- **area_index lives only in the filename, never in the .kv body** — single-source-of-truth for the per-area key, removes round-trip ambiguity, and disarms one entire path-traversal class via `std::to_string` digit-only output (T-05-01).
- **Yaw scalar over full quaternion in the .kv** — six decimals of yaw is enough resolution for the 5° / 5 cm SPEC R-11 tolerance, and the file stays human-inspectable. Quaternion is reconstructed via `tf2::Quaternion::setRPY(0, 0, yaw)` so the Pose is fully populated when read back.
- **T-05-02 lives in write_checkpoint_file, not serialize_checkpoint** — serialize is reused by the round-trip test where we want it to faithfully echo whatever it gets. The write layer is the trust boundary on the BT->planner edge.
- **Empty checkpoint_io.cpp stub in Task 1** — keeps the Task-1 build link green even though `add_library(... src/checkpoint_io.cpp)` is listed there (Task 2 fills the file). Same pattern Plan 01-02 used for `test/CMakeLists.txt`.
- **Tests link against the STATIC lib, not the executable** — mirrors mowgli_map's gtest pattern; lets the action-endpoint test spin a real node under a SingleThreadedExecutor without re-running main.cpp.
- **mowgli_geometry linked as plain `mowgli_geometry`, not the namespaced `mowgli_geometry::mowgli_geometry`** — matches the export form chosen by Plan 01-02 (`ament_export_targets(mowgli_geometry HAS_LIBRARY_TARGET)` without a namespace); same pattern Plan 01-02's own `test/CMakeLists.txt` uses.

## Deviations from Plan

None — plan executed exactly as written. Each plan-prescribed acceptance grep passed (full output captured during Task 1 + Task 2 verification runs):

- `<depend>mowgli_geometry</depend>` and `<depend>mowgli_interfaces</depend>` present in package.xml.
- `rclcpp_action::create_server<PlanCoverage>`, `create_client<mowgli_interfaces::srv::GetAllAreas>`, `create_service<mowgli_interfaces::srv::WriteCheckpoint>`, `ERROR_NO_AREAS`, `ERROR_INTERNAL`, `PLAN-07-PLACEHOLDER` all present in coverage_planner_node.cpp.
- No TF publication in the node source.
- `serialize_checkpoint`, `parse_kv`, `write_checkpoint_file`, `read_checkpoint_file` all declared in checkpoint_io.hpp.
- `atomic_write` and `tf2::getYaw`/`tf2_geometry_msgs` used in checkpoint_io.cpp.
- The `Not implemented yet` Task-1 stub message no longer appears in coverage_planner_node.cpp.

## Issues Encountered

- **colcon build / colcon test cannot run on the macOS host.** Same situation as Plans 01-02 and 01-03 (the docker daemon is not running locally and the mowgli-ros2 devcontainer image is not loaded). The plan's verify steps `cd ros2 && colcon build --packages-select mowgli_coverage_planner` and `colcon test --packages-select mowgli_coverage_planner` are deferred to:
  1. The next dev-branch push, which triggers the standard mowgli-ros2 docker build pipeline (per `feedback_background_pipeline_deploy.md` workflow).
  2. The Pi5 hardware test bench (Plan 01-09 owns the formal hardware acceptance).
  All static / syntactic checks (file existence, grep invariants, header / source consistency, test count) ran green locally. No additional action required from the executor; the Pi5 test bench step is gated by Plan 01-09.

## User Setup Required

None — the package adds no new external services, no env vars, and no auth-gated endpoints.

## Next Phase Readiness

Plan 01-05 closes Wave 3's planner-side bootstrapping. Downstream plans:

- **Plan 01-06 (map_server cleanup):** can now `find_package(mowgli_coverage_planner)`'s exported target (well — actually it doesn't need the planner; it just needs to add `GetAllAreas.srv` server-side and delete the legacy pull-path artifacts). The planner side is ready to call the new service.
- **Plan 01-07 (planner core):** the literal `PLAN-07-PLACEHOLDER` block in `execute()` is the precise extension point. `PlanContext` is the structured slot for the validator outputs + intermediate sweep state. The action plumbing, parameter loading, GetAllAreas pull, ERROR_NO_AREAS / ERROR_INTERNAL paths, and checkpoint I/O helpers are all ready to be called from the new validator pipeline + plan builder.
- **Plan 01-08 (BT integration):** `FollowCoveragePlan` writes checkpoints by calling `/coverage_planner_node/write_checkpoint` — that endpoint is now live and persists `<areas_dir>/coverage_<area_index>.kv` atomically.
- **Plan 01-09 (E2E sim + Pi5 smoke):** will exercise the full action -> plan -> BT path end-to-end; this plan is a prerequisite.

## TDD Gate Compliance

The plan flagged Task 2 as `tdd="true"`. Both gates verified in git log:

1. **RED gate:** `72a4d925 test(01-05): add failing tests for checkpoint .kv I/O + skeleton` — checkpoint_io.cpp at this commit is still the empty Task-1 stub, so the test_checkpoint translation unit fails to link (every test references one of the four undefined symbols).
2. **GREEN gate:** `14e51df4 feat(01-05): implement Checkpoint .kv I/O + wire WriteCheckpoint handler` — checkpoint_io.cpp now defines all four helpers; coverage_planner_node.cpp wires `on_write_checkpoint` to `write_checkpoint_file`.

No REFACTOR commit was needed (clean implementations, no cleanup pass required).

## Threat Flags

None new. The threat surface introduced here matches the plan's `<threat_model>` register exactly:

- T-05-01 (path traversal via area_index) — mitigated by uint32 -> std::to_string; tested implicitly by every CheckpointFsTest case.
- T-05-02 (NaN poses / out-of-range angles) — mitigated by `std::isfinite` guard in `write_checkpoint_file`; covered by `RejectsNonFiniteCheckpointValues`.
- T-05-03 (disk-level corruption) — mitigated by `parse_kv` returning `nullopt` on any failure; covered by 4 separate test cases.
- T-05-04 (mid-write crash) — mitigated by `mowgli_geometry::atomic_write` (Plan 01-02's 4-step pattern: write + fsync(fd) + rename + fsync(parent_dir)).
- T-05-05 (concurrent goals) — mitigated by `planning_active_` atomic guard; `handle_goal` REJECTs while in flight.
- T-05-06 (mow_angle_offset_deg out of range) — accept-disposition; Plan 01-07's validator pipeline range-checks.

No new attacker-relevant surface introduced beyond the plan's register.

## Self-Check: PASSED

Verified:
- All 11 created files present at expected paths (verified via `ls`).
- All 3 task commits present in git log:
  - `4bce4424` (Task 1: feat(01-05): bootstrap mowgli_coverage_planner package skeleton)
  - `72a4d925` (Task 2 RED: test(01-05): add failing tests for checkpoint .kv I/O + skeleton)
  - `14e51df4` (Task 2 GREEN: feat(01-05): implement Checkpoint .kv I/O + wire WriteCheckpoint handler)
- All Task 1 acceptance grep checks pass (action create_server, GetAllAreas client, WriteCheckpoint service, ERROR_NO_AREAS, ERROR_INTERNAL, PLAN-07-PLACEHOLDER present; no TF publication).
- All Task 2 acceptance grep checks pass (4 declarations in checkpoint_io.hpp, atomic_write + tf2 used in checkpoint_io.cpp, on_write_checkpoint stub message removed).
- Test counts: 10 TEST/TEST_F macros across the 2 test translation units (>= 4 per plan minimum).
- colcon build / colcon test deferred to docker pipeline + Pi5 (see Issues Encountered).

---

*Phase: 01-coverage-planner-rewrite*
*Plan: 05*
*Completed: 2026-04-29*
