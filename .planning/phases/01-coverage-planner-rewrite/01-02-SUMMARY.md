---
phase: 01-coverage-planner-rewrite
plan: 02
subsystem: api
tags: [ros2, mowgli_geometry, header-only, eigen, boost-geometry, footprint, pca, atomic-write, geometry, gtest]

# Dependency graph
requires:
  - phase: 01-coverage-planner-rewrite
    provides: "Plan 01-01 froze the coverage-planner contract surface (PlanCoverage.action, CoverageWaypoint, Checkpoint, etc.). Plan 01-02 doesn't depend on those types directly but lives downstream of the locked contract."
provides:
  - "Header-only ROS2 package mowgli_geometry that downstream packages can find_package() and link via target_link_libraries(... mowgli_geometry)"
  - "geometry.hpp: 4 helpers PROMOTED VERBATIM from map_server_node.cpp — point_in_polygon, convex_hull, compute_optimal_mow_angle, offset_polygon_inward (with the closing-vertex dedup + CCW/CW shoelace winding fix from the live #50-phase-2 incident on 2026-04-27)"
  - "footprint.hpp: footprint_polygon (4 CCW corners at pose), rotation_sweep_radius, disc_inside_polygon, footprint_inside_polygon (Boost.Geometry covered_by), footprint_disjoint_obstacles (Boost.Geometry disjoint)"
  - "pca.hpp: pca_principal_axis via Eigen3 SelfAdjointEigenSolver<Matrix2d> with degenerate-rank fallback to longest-edge atan2 — backbone for the SPECIAL_PATTERN narrow-area centerline (D-10)"
  - "atomic_write.hpp: 4-step power-loss-safe write (open + write + fsync(fd) + rename + fsync(parent_dir)) used by the BT-delegates-IO-to-planner checkpoint flow locked in Plan 01-01"
  - "FootprintParams struct (robot_length / robot_width / drive_axis_x_offset / drive_axis_y_offset) consumed by Plan 05 coverage_planner_node"
  - "to_bg adapter pattern (geometry_msgs::Polygon → bg::model::polygon<bg::model::d2::point_xy<double>>) — first introduction of Boost.Geometry as a C++ library to the codebase"
affects:
  - 01-04-gui-integration  # uses footprint params indirectly via Plan 05
  - 01-05-coverage-planner-skeleton  # main consumer of the entire library
  - 01-06-map-server-cleanup  # will switch to #include <mowgli_geometry/geometry.hpp> + delete the local copies
  - 01-07-planner-core  # validators + sweep call into footprint.hpp / pca.hpp / atomic_write.hpp
  - 01-08-bt-integration  # FollowCoveragePlan owns no IO; checkpoint flows via WriteCheckpoint.srv to coverage_planner_node which uses atomic_write

# Tech tracking
tech-stack:
  added:
    - "Boost.Geometry (header-only, via Boost::headers / find_package(Boost) — first C++ use of bg::model::polygon, bg::within, bg::covered_by, bg::disjoint, bg::correct in the codebase)"
  patterns:
    - "Header-only INTERFACE library: ament_cmake target with target_include_directories + ament_export_targets(... HAS_LIBRARY_TARGET); install(DIRECTORY include/ DESTINATION include) only — no compiled artefact"
    - "to_bg adapter: convert geometry_msgs::Polygon to BgPolygon and call bg::correct() to enforce closing point + winding before any predicate"
    - "Degenerate-rank fallback for PCA: SelfAdjointEigenSolver smallest-eigenvalue threshold 1e-9 (RESEARCH §3.4 Assumption A3) → longest-edge atan2"
    - "Atomic write 4-step pattern: open(O_WRONLY|O_CREAT|O_TRUNC) + write(retry on EINTR) + fsync(fd) + rename + fsync(parent_dir). Step 4 is non-negotiable on ext4 / SD per LWN.net citation"

key-files:
  created:
    - "ros2/src/mowgli_geometry/CMakeLists.txt"
    - "ros2/src/mowgli_geometry/package.xml"
    - "ros2/src/mowgli_geometry/include/mowgli_geometry/geometry.hpp"
    - "ros2/src/mowgli_geometry/include/mowgli_geometry/footprint.hpp"
    - "ros2/src/mowgli_geometry/include/mowgli_geometry/pca.hpp"
    - "ros2/src/mowgli_geometry/include/mowgli_geometry/atomic_write.hpp"
    - "ros2/src/mowgli_geometry/test/CMakeLists.txt"
    - "ros2/src/mowgli_geometry/test/test_offset_polygon_inward.cpp"
    - "ros2/src/mowgli_geometry/test/test_footprint_check.cpp"
    - "ros2/src/mowgli_geometry/test/test_pca_axis.cpp"
    - "ros2/src/mowgli_geometry/test/test_atomic_write.cpp"
  modified: []

key-decisions:
  - "All 4 promoted geometry helpers are byte-equivalent to the originals in map_server_node.cpp (lines 1389-1413, 2421-2502, 3298-3416) — modulo namespacing under mowgli_geometry and `inline` linkage. Plan 06 can swap the local copies for the namespaced ones without changing call sites. The closing-vertex dedup + shoelace winding detection from the live #50-phase-2 fix (2026-04-27) is preserved verbatim; the same regression-guard test now lives in the package."
  - "footprint_inside_polygon uses bg::covered_by (boundary contact accepted) instead of bg::within (strict interior). Coverage planning needs the footprint to be allowed to graze the working-area boundary along outline passes; bg::within would reject those poses. T-02-03 mitigation in Plan 05 ValidatorPipeline covers the NaN / GIGO path."
  - "atomic_write reports false when fsync(parent_dir) fails. The file is already in place at that point — only durability across power loss is at risk — but failing loudly to the caller surfaces the SD / ext4 health issue in RESEARCH §9.4 instead of silently degrading the checkpoint guarantee."
  - "PCA degenerate-rank fallback uses the polygon's longest edge (atan2 of the longest closing-traversal edge) rather than SelfAdjointEigenSolver's principal eigenvector. For the y=x collinear case the solver still returns a valid axis, but the longest-edge form is robust against numerical noise and matches the SPECIAL_PATTERN intent (centerline along the dominant geometric span)."
  - "test/CMakeLists.txt placeholder created in Task 1 so the BUILD_TESTING add_subdirectory(test) call doesn't error before tests exist. Replaced in Task 2."

patterns-established:
  - "Header-only ament_cmake INTERFACE library: target_link_libraries(target INTERFACE ...) + ament_export_targets(... HAS_LIBRARY_TARGET) + install(DIRECTORY include/ ...). Downstream find_package(mowgli_geometry) + target_link_libraries(consumer mowgli_geometry) gets the include path AND the transitive Eigen3 / Boost / geometry_msgs deps for free."
  - "to_bg(geometry_msgs::Polygon) → BgPolygon adapter. Call bg::correct() after appending so the polygon has a valid closing point + CCW winding regardless of the source representation. Reusable across mowgli_coverage_planner (Plan 05) and any future Boost.Geometry consumer."
  - "Test for the closing-vertex-dedup edge case as a regression guard against #50-phase-2 (2026-04-27) — the test_offset_polygon_inward.ClosingVertexDeduplicated case feeds the same input shape that triggered the live X-shape outline bug and asserts the result has 4 vertices, not 3."

requirements-completed: [R-6, R-7, R-9, R-13, R-10]

# Metrics
duration: 9min
completed: 2026-04-29
---

# Phase 1 Plan 2: mowgli_geometry Header-Only Library Summary

**Single source of truth for polygon geometry: 4 production-tested helpers promoted verbatim from map_server_node.cpp + 4 new helpers (footprint, PCA, disc-in-polygon, atomic write) backed by 18 gtest assertions across 4 executables, all passing.**

## Performance

- **Duration:** 9 min
- **Started:** 2026-04-29T05:56:11Z
- **Completed:** 2026-04-29T06:06:07Z
- **Tasks:** 2 (1 + 1 TDD-paired)
- **Files created:** 11 (4 headers, 4 tests, 1 package.xml, 2 CMakeLists.txt)
- **Files modified:** 0

## Accomplishments

- New ROS2 package `mowgli_geometry` builds cleanly (`colcon build --packages-select mowgli_geometry` exits 0) and exports a header-only INTERFACE target via `find_package(mowgli_geometry)`.
- All 4 promoted geometry helpers (`point_in_polygon`, `convex_hull`, `compute_optimal_mow_angle`, `offset_polygon_inward`) are byte-equivalent to the original `map_server_node.cpp` implementations (lines 1389-1413, 2421-2502, 3298-3416) modulo namespacing + `inline` linkage. The closing-vertex dedup + shoelace winding fix from the live #50-phase-2 incident is preserved verbatim and now has a dedicated regression test.
- All 4 new helpers (`footprint_polygon`, `disc_inside_polygon`, `pca_principal_axis`, `atomic_write`) implement the formulas from RESEARCH §3.1, §3.4, §7.2 plus the Boost.Geometry pattern from §5.2-§5.3.
- 4 gtest executables (`test_offset_polygon_inward`, `test_footprint_check`, `test_pca_axis`, `test_atomic_write`) covering 18 individual assertions: `100% tests passed, 0 tests failed out of 7` (4 gtest + 3 linters).
- Plan 06 unblocked: `map_server_node.cpp` can now `#include <mowgli_geometry/geometry.hpp>` and delete its local copies without changing any call site.

## Task Commits

Each task was committed atomically; Task 2 used the TDD RED → GREEN gate sequence.

1. **Task 1: Bootstrap mowgli_geometry package + promote 4 functions verbatim** — `ae551459` (feat)
2. **Task 2 RED: failing tests for footprint, pca, atomic_write** — `63933306` (test)
3. **Task 2 GREEN: implement footprint, pca, atomic_write helpers** — `351f4139` (feat)

**Plan metadata commit:** `<recorded after final commit>` (docs: complete plan)

## Files Created/Modified

### Created

- `ros2/src/mowgli_geometry/package.xml` — package metadata (geometry_msgs + eigen depends, ament_cmake_gtest test_depend).
- `ros2/src/mowgli_geometry/CMakeLists.txt` — header-only INTERFACE library, exports `mowgli_geometry` target with transitive `Eigen3::Eigen` + Boost + geometry_msgs deps.
- `ros2/src/mowgli_geometry/include/mowgli_geometry/geometry.hpp` — 4 promoted helpers (`point_in_polygon`, `convex_hull`, `compute_optimal_mow_angle`, `offset_polygon_inward`).
- `ros2/src/mowgli_geometry/include/mowgli_geometry/footprint.hpp` — `FootprintParams` struct + `footprint_polygon`, `rotation_sweep_radius`, `disc_inside_polygon`, `footprint_inside_polygon`, `footprint_disjoint_obstacles` (Boost.Geometry).
- `ros2/src/mowgli_geometry/include/mowgli_geometry/pca.hpp` — `pca_principal_axis` via Eigen3 `SelfAdjointEigenSolver<Matrix2d>` + longest-edge fallback.
- `ros2/src/mowgli_geometry/include/mowgli_geometry/atomic_write.hpp` — 4-step atomic write (open + write + fsync(fd) + rename + fsync(parent_dir)).
- `ros2/src/mowgli_geometry/test/CMakeLists.txt` — wires 4 `ament_add_gtest` entries against the INTERFACE target.
- `ros2/src/mowgli_geometry/test/test_offset_polygon_inward.cpp` — 4 cases: shrink, grow (negative inset), closing-vertex dedup regression, zero-inset no-op.
- `ros2/src/mowgli_geometry/test/test_footprint_check.cpp` — 9 cases: identity pose, 90° yaw, sweep radius, footprint inside / off-edge, disc fits / extends past, disjoint clear / intersecting / empty list.
- `ros2/src/mowgli_geometry/test/test_pca_axis.cpp` — 5 cases: horizontal, vertical, 45° tilted rectangles, collinear (degenerate-rank fallback), n<3 returns 0.
- `ros2/src/mowgli_geometry/test/test_atomic_write.cpp` — 5 cases: happy path + no .tmp orphan, overwrite, multi-line round-trip, non-existent dir → false, empty content.

### Modified

None — Plan 02 is purely additive. `map_server_node.cpp` switching to the new header lives in Plan 06.

## Promoted Functions — Original Source Map

| Function                     | Original location (map_server_node.cpp) | Linkage change      | Notes                                                                          |
| ---------------------------- | --------------------------------------- | ------------------- | ------------------------------------------------------------------------------ |
| `point_in_polygon`           | lines 1389-1413                         | static → inline     | Ray casting; returns false for n<3.                                            |
| `convex_hull`                | lines 2421-2457                         | member → inline     | Andrew's monotone chain, O(n log n).                                           |
| `compute_optimal_mow_angle`  | lines 2459-2502                         | member → inline     | MBR over convex-hull edges. Returns angle in radians.                          |
| `offset_polygon_inward`      | lines 3298-3416                         | const member → inline | Vertex-bisector inward Minkowski offset. Includes closing-vertex dedup + shoelace winding fix from #50-phase-2 (2026-04-27). |

## New Helpers — Test Coverage Map

| Helper                          | Header           | Test file                          | Coverage                                                                                |
| ------------------------------- | ---------------- | ---------------------------------- | --------------------------------------------------------------------------------------- |
| `footprint_polygon`             | footprint.hpp    | test_footprint_check.cpp           | Identity pose, 90° yaw rotation                                                         |
| `rotation_sweep_radius`         | footprint.hpp    | test_footprint_check.cpp           | YardForce 500 dimensions: r = hypot(0.50, 0.20) ≈ 0.5385 m                              |
| `disc_inside_polygon`           | footprint.hpp    | test_footprint_check.cpp           | Small centred disc fits, large disc extends past edge, n<3 polygon rejected             |
| `footprint_inside_polygon`      | footprint.hpp    | test_footprint_check.cpp           | Centred footprint inside, off-edge footprint outside (uses bg::covered_by)              |
| `footprint_disjoint_obstacles`  | footprint.hpp    | test_footprint_check.cpp           | Distant obstacles clear, intersecting obstacle rejected, empty list returns true        |
| `pca_principal_axis`            | pca.hpp          | test_pca_axis.cpp                  | Horizontal 5x0.5, vertical 0.5x5, 45°-tilted rect, y=x collinear (longest-edge fallback), n<3 returns 0 |
| `atomic_write`                  | atomic_write.hpp | test_atomic_write.cpp              | Happy path + no .tmp orphan, overwrite, multi-line content, non-existent dir → false, empty content |
| `offset_polygon_inward` (regress) | geometry.hpp     | test_offset_polygon_inward.cpp     | Shrink 10x10 → 8x8, grow (inset=-1) → 12x12, closing-vertex dedup, zero-inset no-op    |

## Boost / Eigen3 Link Pattern

`mowgli_geometry/CMakeLists.txt`:

```cmake
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(mowgli_geometry INTERFACE)
target_include_directories(mowgli_geometry INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowgli_geometry INTERFACE Eigen3::Eigen)
target_compile_features(mowgli_geometry INTERFACE cxx_std_17)
ament_target_dependencies(mowgli_geometry INTERFACE geometry_msgs Boost)

ament_export_targets(mowgli_geometry HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs Eigen3 Boost)
```

Downstream packages add `<depend>mowgli_geometry</depend>` to their package.xml + `find_package(mowgli_geometry REQUIRED)` and `target_link_libraries(consumer mowgli_geometry)` to inherit the include path AND the transitive Eigen3 / Boost / geometry_msgs deps.

## Decisions Made

- **`covered_by` over `within` for footprint-inside-polygon:** boundary contact is allowed during outline passes; strict interior would reject valid poses where the footprint grazes the polygon edge.
- **`atomic_write` reports `fsync(parent_dir)` failure as false:** the file is in place at that point but durability across power loss is at risk. Failing loudly to the caller exposes SD/ext4 health issues that RESEARCH §9.4 documents.
- **PCA degenerate fallback uses the polygon's longest edge:** robust against numerical noise on rank-1 covariances and matches the SPECIAL_PATTERN intent of "centerline along the dominant span".
- **Empty `test/CMakeLists.txt` placeholder during Task 1:** keeps the main `add_subdirectory(test)` call valid before the test files exist; replaced in Task 2.

## Deviations from Plan

None — plan executed exactly as written. Each plan-prescribed acceptance criterion was met:

- `colcon build --packages-select mowgli_geometry` exits 0 (verified).
- `colcon test --packages-select mowgli_geometry` reports 100% pass / 0 fail (verified).
- `grep "<name>mowgli_geometry</name>" package.xml` succeeds.
- `grep "INTERFACE" CMakeLists.txt` succeeds.
- `grep "Eigen3::Eigen" CMakeLists.txt` succeeds.
- 4 inline functions found in geometry.hpp.
- `namespace mowgli_geometry` present.
- `install/mowgli_geometry/include/mowgli_geometry/geometry.hpp` exists.
- `install/mowgli_geometry/share/mowgli_geometry/cmake/mowgli_geometryConfig.cmake` exists.
- `Eigen::SelfAdjointEigenSolver` present in pca.hpp.
- `fsync(dir_fd)` present in atomic_write.hpp.
- `boost::geometry` present in footprint.hpp.
- `rotation_sweep_radius` present in footprint.hpp.
- `CollinearFallsBackToLongestEdge` test enforces the degenerate-rank fallback path.
- `test_atomic_write` asserts no `.tmp` orphan after the success path.

## Issues Encountered

- **CMake deprecation warning** during build:
  ```
  ament_target_dependencies() is deprecated. Use target_link_libraries()
  with modern CMake targets instead.
  ```
  Comes from `ament_cmake_target_dependencies.cmake` itself (kilted-side change). The plan explicitly prescribes `ament_target_dependencies(... INTERFACE geometry_msgs Boost)` so the warning was kept rather than diverging from the locked plan. Building / linking still succeeds. Followup: a future cleanup wave may switch to `target_link_libraries(... INTERFACE ${geometry_msgs_TARGETS})` once all packages can co-migrate.
- **GCC 10.1 ABI note** during `test_offset_polygon_inward.cpp` compilation:
  ```
  parameter passing for argument of type 'std::pair<double, double>'
  when C++17 is enabled changed to match C++14 in GCC 10.1
  ```
  This is a non-fatal note about ABI compatibility, not a warning about correctness. No action needed.

## User Setup Required

None — header-only library, no runtime config, no env vars, no service accounts.

## Next Phase Readiness

Plan 02 closes the geometry foundation that Plans 04-08 depend on. No further work required before:

- **Plan 04 (GUI integration):** indirectly affected via Plan 05's footprint params surfacing to the GUI. No direct mowgli_geometry use.
- **Plan 05 (coverage_planner_node skeleton):** main consumer. Will `find_package(mowgli_geometry)` and use all 4 headers.
- **Plan 06 (map_server cleanup):** switches `map_server_node.cpp` to `#include <mowgli_geometry/geometry.hpp>` and deletes the 4 in-file copies. Deletions can proceed without behaviour changes since the promoted functions are byte-equivalent.
- **Plan 07 (planner core):** `ValidatorPipeline`, `BoustrophedonSweeper`, narrow-area strategies all call into `footprint.hpp` / `pca.hpp`.
- **Plan 08 (BT integration):** `FollowCoveragePlan` delegates checkpoint IO to `coverage_planner_node` (per Plan 01-01 `WriteCheckpoint.srv`) which uses `atomic_write`.

## TDD Gate Compliance

The plan flagged Task 2 as `tdd="true"`. Both gates verified in git log:

1. **RED gate:** `63933306 test(01-02): add failing tests for footprint, pca, atomic_write` — colcon test reported 17 failures across 3 stubbed test executables (footprint, pca, atomic_write).
2. **GREEN gate:** `351f4139 feat(01-02): implement footprint, pca, atomic_write helpers` — colcon test reported 100% pass, 0 fail.

No REFACTOR commit was needed (clean implementations, no cleanup pass required).

## Self-Check: PASSED
