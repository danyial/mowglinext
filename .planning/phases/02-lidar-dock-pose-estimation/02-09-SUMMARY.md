---
phase: 02-lidar-dock-pose-estimation
plan: 09
subsystem: build-integration-gap-closure
status: code-only-complete; phase-end-podman-build-pending
autonomous: true
gap_closure: true
tags: [gap-closure, kinematic-icp-cmake, kiss-icp-headers, path-a, regression-smoke-test]
dependency-graph:
  requires:
    - "02-02-SUMMARY: mowgli_lidar_docking_lib STATIC target (the lib whose linkage we are fixing)"
    - "ros2/src/kinematic_icp/cpp/kinematic_icp/ — PRBonn submodule cpp tree (read by add_subdirectory)"
    - "ros2/src/kinematic_icp/cpp/kinematic_icp/kiss_icp/kiss-icp.cmake — pins kiss_icp v1.2.0 via FetchContent (inherited transitively, NOT redeclared in this package)"
    - "02-01-PROBE.md A1 verdict: kiss_icp::VoxelHashMap::AddPoints is public (struct default access) — used by the smoke gtest"
    - "02-01-PROBE.md A2 verdict: kiss_icp::VoxelHashMap::GetClosestNeighbor is public + returns std::tuple<Eigen::Vector3d, double> — used by the smoke gtest"
  provides:
    - "kinematic_icp_pipeline STATIC linkage in mowgli_lidar_docking_lib (PUBLIC, propagates kinematic_icp/ + kiss_icp/ INTERFACE_INCLUDE_DIRECTORIES via the chain to kiss_icp_pipeline -> kiss_icp_core)"
    - "test_kinematic_icp_headers_reachable regression smoke gtest (2 TEST cases, fails to compile if Task 1's CMake edits regress)"
  affects:
    - "ros2/src/mowgli_lidar_docking/CMakeLists.txt (add_subdirectory + explicit kinematic_icp_pipeline link added)"
    - "ros2/src/mowgli_lidar_docking/test/CMakeLists.txt (new ament_add_gtest registration appended)"
    - "Unblocks: mowgli_behavior, mowgli_simulation, mowgli_bringup (the 3 packages that were 'not processed' in the prior podman build)"
    - "Unblocks: R-2, R-3, R-7, R-8, R-12, R-13 — every requirement whose gtest binary lives in or downstream of mowgli_lidar_docking"
tech-stack:
  added:
    - "CMake add_subdirectory pattern with EXCLUDE_FROM_ALL + scoped binary directory for in-tree submodule cpp lib reuse"
    - "Regression-smoke gtest pattern: link only against the package lib (not the underlying cpp target) so the test fails loudly on any future linkage regression"
  patterns:
    - "Path A (self-add_subdirectory) over Path B (patch upstream submodule) and Path C (vendor headers) — keeps mowgli_lidar_docking self-contained, no PRBonn fork burden"
    - "Inherit kiss_icp version pin transitively from kinematic_icp's own kiss-icp.cmake — no separate FetchContent_Declare in this package, so version drift is impossible by construction"
key-files:
  created:
    - "ros2/src/mowgli_lidar_docking/test/test_kinematic_icp_headers_reachable.cpp"
  modified:
    - "ros2/src/mowgli_lidar_docking/CMakeLists.txt"
    - "ros2/src/mowgli_lidar_docking/test/CMakeLists.txt"
decisions:
  - "Path A (self-add_subdirectory + explicit kinematic_icp_pipeline link) chosen per 02-VERIFICATION.md GAP-02 § 'Suggested fix paths'. Path B (patch the kinematic_icp submodule and ament_export the cpp targets) rejected because it would require a forever-diff against PRBonn upstream + .gitmodules URL change to a fork. Path C (vendor headers) rejected because it loses upstream fix tracking and the matcher needs the kinematic_icp pipeline LIBRARY (not just headers) to compile."
  - "Inherit kiss_icp v1.2.0 transitively via the kinematic_icp cpp top-level CMakeLists' include(kiss_icp/kiss-icp.cmake) call. Do NOT add a separate FetchContent_Declare(kiss_icp ...) block in mowgli_lidar_docking — duplicating it would risk version skew between the two consumers of the same lib in the same workspace."
  - "Keep find_package(Sophus REQUIRED), find_package(kinematic_icp REQUIRED), ament_target_dependencies(... kinematic_icp), and ament_export_dependencies(... kinematic_icp) — defence-in-depth for ROS-wrapper symbols and for downstream packages (mowgli_behavior) that consume mowgli_lidar_docking_lib via ament_target_dependencies(... mowgli_lidar_docking)."
  - "Link the new regression smoke gtest ONLY against mowgli_lidar_docking_lib (NOT against kinematic_icp_pipeline directly). Linking it directly to kinematic_icp_pipeline would mask any future regression in the lib's own PUBLIC propagation of include paths — the test must fail at the same translation-unit boundary as a real consumer would."
  - "Collapse the add_subdirectory call onto a single line so the verification gate `grep -n 'add_subdirectory.*kinematic_icp/cpp/kinematic_icp'` matches in one pass. Multi-line CMake function calls are valid syntactically but make grep-based gates noisy."
  - "Do NOT instantiate kinematic_icp::pipeline::KinematicICP in the smoke test. Its constructor takes a non-trivial Config struct that can change across PRBonn releases; pinning the full ctor path would couple the smoke test to upstream's internal API. Plan 02-04's test_kinematic_icp_dock_matcher already exercises the full constructor path. The smoke test only needs to assert HEADER REACHABILITY (the GAP-02 symptom)."
metrics:
  duration_min: ~12
  completed_date: 2026-04-30
---

# Phase 2 Plan 09: GAP-02 closure (mowgli_lidar_docking ↔ kinematic_icp + kiss_icp cpp linkage) Summary

Single-plan gap closure rewriting `mowgli_lidar_docking/CMakeLists.txt` to pull the kinematic_icp cpp tree in via `add_subdirectory` (Path A) and link explicitly against the resulting `kinematic_icp_pipeline` STATIC target. Adds a 2-case regression smoke gtest that fails to compile loudly if any future "simplification" reverts the linkage. PRBonn submodule (`ros2/src/kinematic_icp/`) is byte-identical to HEAD; package.xml + src/ + include/ unchanged.

## Built

### 1. CMakeLists.txt rewrite (Task 1, commit `7470b60d`)

Three concrete edits to `ros2/src/mowgli_lidar_docking/CMakeLists.txt`:

1. **Replaced the `# Dependencies` comment block** with the new GAP-02 rationale block. The block names the failing headers verbatim, references VERIFICATION.md GAP-02, identifies Path A as the chosen fix, and explains why `find_package(kinematic_icp REQUIRED)` and `ament_target_dependencies(... kinematic_icp)` are kept (defence-in-depth for the ROS-wrapper symbols that may surface in future plans). The block contains the substring `GAP-02` (3 occurrences) so future grep-based code-archaeology surfaces this fix immediately.

2. **Injected the `add_subdirectory` call** between `find_package(kinematic_icp REQUIRED)` and `add_library(mowgli_lidar_docking_lib STATIC ...)`:

   ```cmake
   add_subdirectory(${CMAKE_SOURCE_DIR}/../kinematic_icp/cpp/kinematic_icp ${CMAKE_CURRENT_BINARY_DIR}/kinematic_icp_cpp_for_mowgli EXCLUDE_FROM_ALL)
   ```

   - `EXCLUDE_FROM_ALL`: the cpp tree currently declares only library targets (`kinematic_icp_pipeline`, `kinematic_icp_registration`, `kinematic_icp_threshold` plus transitive kiss_icp libs), but EXCLUDE_FROM_ALL is defence-in-depth against future upstream changes that might add example/test executables.
   - Scoped binary dir `kinematic_icp_cpp_for_mowgli` prevents collision with any future in-tree add_subdirectory of the same source tree.
   - `${CMAKE_SOURCE_DIR}/..` resolves to `ros2/src/` regardless of where colcon places the build dir, so the path is identical on macOS dev hosts and Pi5 ARM containers.

3. **Added `kinematic_icp_pipeline` to the `target_link_libraries(mowgli_lidar_docking_lib PUBLIC ...)` block** with an inline comment explaining why this is GAP-02's load-bearing edit (PUBLIC chain to `kiss_icp_pipeline` → `kiss_icp_core` propagates the kiss_icp INTERFACE_INCLUDE_DIRECTORIES, which is what makes `kiss_icp/core/VoxelHashMap.hpp` reachable from `confidence_metrics.cpp`).

Defence-in-depth elements left intact:
- `find_package(Sophus REQUIRED)` (line 69) — kept for self-documenting dep graph.
- `find_package(kinematic_icp REQUIRED)` (line 71) — kept for ament discovery.
- `ament_target_dependencies(... kinematic_icp)` (line 159) — kept for ROS-wrapper symbols that may be needed by future plans.
- `ament_export_dependencies(... kinematic_icp)` (line 233) — kept so downstream packages (mowgli_behavior Plan 02-06 BT nodes consuming `mowgli_lidar_docking_lib` via `ament_target_dependencies(... mowgli_lidar_docking)`) still pull the ament wrapper deps.

### 2. Regression smoke gtest (Task 2, commit `10f722a6`)

**`ros2/src/mowgli_lidar_docking/test/test_kinematic_icp_headers_reachable.cpp`** — new file with two TEST cases:

- **`KissIcpVoxelHashMapConstructible`** — instantiates `kiss_icp::VoxelHashMap` with the public ctor signature documented verbatim in 02-01-PROBE.md (`(double voxel_size, double max_distance, unsigned int max_points_per_voxel)`), exercises the public `Empty()` getter, then `AddPoints` (PROBE.md A1) on three points along the +X axis, then `GetClosestNeighbor` (PROBE.md A2) for a query point between two of them. Asserts the squared-distance return is `0.0025` ± `1e-6` and that the returned neighbor matches one of the two seeded points.
- **`KinematicIcpHeaderCompiles`** — the `#include <kinematic_icp/pipeline/KinematicICP.hpp>` directive at the top of the file IS the test. The TEST body adds a `static_assert` on pointer size and a `SUCCEED()` line so the test produces a real PASS line in `colcon test` output. We deliberately do NOT instantiate `kinematic_icp::pipeline::KinematicICP` here — Plan 02-04's `test_kinematic_icp_dock_matcher` already exercises the full ctor path, and pinning the ctor in the smoke test would couple it to upstream's internal Config struct (which can change across PRBonn releases).

**`ros2/src/mowgli_lidar_docking/test/CMakeLists.txt`** — appended a new `ament_add_gtest(test_kinematic_icp_headers_reachable ...)` block after the existing `test_dock_scan_match_node` registration. Linked ONLY against `mowgli_lidar_docking_lib` (NOT against `kinematic_icp_pipeline` directly — that would mask Task 1 regressions, the whole point of the smoke test is to verify the lib propagates the include paths).

## GAP-02 closure

### Verbatim build log (from `deferred-items.md` § "From phase-end podman build (post Plan 02-08)" Bug 2)

```
fatal error: kinematic_icp/pipeline/KinematicICP.hpp: No such file or directory
fatal error: kiss_icp/core/VoxelHashMap.hpp: No such file or directory
Failed   <<< mowgli_lidar_docking [8.16s, exited with code 2]
```

Build summary from the failing run:

```
Summary: 9 packages finished [1min 25s]
  1 package failed: mowgli_lidar_docking
  9 packages had stderr output: kinematic_icp mowgli_coverage_planner mowgli_geometry
                                 mowgli_hardware mowgli_lidar_docking mowgli_localization
                                 mowgli_map mowgli_monitoring mowgli_nav2_plugins
  3 packages not processed (mowgli_behavior, mowgli_simulation, mowgli_bringup)
```

### Path chosen: A — self-add_subdirectory

Per 02-VERIFICATION.md GAP-02 § "Suggested fix paths". Selected because:

- **Self-contained.** No fork burden against PRBonn upstream. No `.gitmodules` URL change.
- **No version drift.** kinematic_icp's cpp top-level CMakeLists already does `include(kiss_icp/kiss-icp.cmake)` which `FetchContent`s kiss_icp v1.2.0. By NOT declaring a separate `FetchContent_Declare(kiss_icp ...)` block in mowgli_lidar_docking, we cannot drift from the kinematic_icp submodule's pinned version.
- **Acceptable build cost.** ~3-5 min added to `podman build` per the VERIFICATION.md analysis (the cpp tree is now compiled twice in the same colcon workspace, once per consuming package). Acceptable per CONTEXT.md "Code-only Commits, podman-Build am Phasen-Ende" — the build runs once at phase end.

Path B (patch the kinematic_icp submodule + ament_export the cpp targets) and Path C (vendor minimum kiss_icp / kinematic_icp headers into `mowgli_lidar_docking/third_party/`) were both rejected per the plan's `<decision_rationale>` section.

### CMake target-redefinition risk mitigation

Both `kinematic_icp` (the ROS package) and `mowgli_lidar_docking` (this fix) trigger `add_subdirectory(... cpp/kinematic_icp)` from different parent CMake contexts during a single colcon workspace build. Each colcon package configures + builds in its OWN build directory (`ros2/build/<package>/`), so target names like `kinematic_icp_pipeline` exist independently in each build tree — there is NO cross-package redefinition. Within `mowgli_lidar_docking`'s own build tree, `EXCLUDE_FROM_ALL` + the scoped binary directory `kinematic_icp_cpp_for_mowgli` are defence-in-depth against future upstream changes that might add executables or against any future in-tree add_subdirectory of the same source tree.

### `ros2/src/kinematic_icp/` was NOT modified

Acceptance criterion 5 of Task 1 explicitly requires `git status --porcelain ros2/src/kinematic_icp/` to return empty. Verified at commit `10f722a6` HEAD time:

```
$ git status --porcelain ros2/src/kinematic_icp/
(empty)
```

The PRBonn submodule is byte-identical to HEAD. Path B is explicitly rejected.

## Phase-end podman build status

**`DEFERRED-TO-PHASE-END-BUILD`** per CONTEXT.md "Code-only Commits, podman-Build am Phasen-Ende". This plan does not run the build itself; the operator runs:

```bash
cd ros2 && podman build --target build -t mowgli-phase2:test .
```

after merging this branch. Expected outcome:

- `Successfully tagged mowgli-phase2:test`
- All 13 packages in the `Finished <<<` list
- 0 packages failed; 0 packages not processed
- The 3 previously-blocked downstream packages (`mowgli_behavior`, `mowgli_simulation`, `mowgli_bringup`) reach `Finished <<<` for the first time on this branch

After the build is green the operator runs `02-08-PI5-CHECKLIST.md` on Pi5 for the 5-of-5 hardware UAT, then commits `02-08-PI5-RESULTS.md`. From the dev container, `/gsd-verify-phase 2` flips `02-VERIFICATION.md` rows for R-2, R-3, R-7, R-8, R-9, R-10, R-11, R-12, R-13 to `✓ VERIFIED` (or operator-evidence equivalents for the 🔧 hardware rows). Phase 2 closes.

The regression smoke gtest will run inside the operator's podman build via the second `colcon test` block in Stage 4 of `ros2/Dockerfile`. Expected output:

```
1 test passed, 0 failed
test_kinematic_icp_headers_reachable
  KinematicIcpHeadersReachable.KissIcpVoxelHashMapConstructible    PASSED
  KinematicIcpHeadersReachable.KinematicIcpHeaderCompiles          PASSED
```

If the test fails to compile with the original `No such file or directory` errors, that is by design — it means a future revert of Task 1's CMake edits is in flight.

## Verified-good packages preserved

The 9 packages that built green pre-fix (per the `deferred-items.md` build log) are still expected to build green post-fix. This plan made ZERO source modifications outside `mowgli_lidar_docking`:

1. `mowgli_geometry` — Plan 02-01 key=value parser shared header
2. `mowgli_interfaces` — with the new `DockMatchConfidence.msg` from Plan 02-01
3. `mowgli_nav2_plugins` — unchanged since Phase 1
4. `mowgli_localization` — Plan 02-03 `dock_scan_capture` library + Plan 02-05 cascade
5. `mowgli_coverage_planner` — Phase 1 outputs
6. `mowgli_hardware` — Plan 02-01 inline-parser → shared-header migration
7. `mowgli_monitoring` — Phase 1 outputs
8. `mowgli_map` — same Plan 02-01 migration
9. `kinematic_icp` — PRBonn upstream submodule (byte-identical, see GAP-02 closure section)

Sanity diff against the plan-authoring commit (`e5cdcd4a docs(02-09): gap-closure plan`) at `HEAD`:

```
$ git diff --stat e5cdcd4a..HEAD -- \
    ros2/src/mowgli_lidar_docking/package.xml \
    ros2/src/mowgli_lidar_docking/src/ \
    ros2/src/mowgli_lidar_docking/include/
(empty)
```

`package.xml` + every `.cpp`/`.hpp` under `src/` and `include/` are byte-identical to HEAD. Only `CMakeLists.txt` and `test/CMakeLists.txt` were modified, and `test/test_kinematic_icp_headers_reachable.cpp` was created. The build-graph change is surgical.

## Decisions Made

(See `decisions:` in frontmatter for the canonical list — the 6 decisions below mirror it for human-readable Markdown.)

- **Path A over Path B and Path C.** Self-add_subdirectory is the smallest, most-self-contained fix that does not require carrying a forever-diff against PRBonn upstream nor losing upstream fix tracking by vendoring headers.
- **Transitive kiss_icp inheritance, not redeclaration.** No separate `FetchContent_Declare(kiss_icp ...)` in this package — kinematic_icp's cpp top-level CMakeLists already declares it at v1.2.0 via `include(kiss_icp/kiss-icp.cmake)`. Inheriting transitively makes version drift impossible by construction.
- **Defence-in-depth ament dependencies preserved.** `find_package(Sophus REQUIRED)`, `find_package(kinematic_icp REQUIRED)`, `ament_target_dependencies(... kinematic_icp)`, and `ament_export_dependencies(... kinematic_icp)` are all kept on disk even though the cpp library now flows through `target_link_libraries(... kinematic_icp_pipeline)`. They cost nothing and protect against future evolution of the ROS-wrapper layer or downstream `mowgli_behavior` consumption patterns.
- **Test target links only against `mowgli_lidar_docking_lib`.** The smoke test must fail at the same translation-unit boundary a real consumer would. Linking it directly to `kinematic_icp_pipeline` would mask any future regression in the lib's own PUBLIC propagation.
- **Single-line `add_subdirectory` call.** Collapsed onto one line so the verification-gate pattern `grep -n 'add_subdirectory.*kinematic_icp/cpp/kinematic_icp'` matches in a single pass. CMake accepts both forms; this form keeps grep-based gates clean.
- **No instantiation of `kinematic_icp::pipeline::KinematicICP` in the smoke test.** The ctor takes a non-trivial Config struct whose internals can change across PRBonn releases. Plan 02-04's `test_kinematic_icp_dock_matcher` exercises the full ctor path; this smoke test only needs to assert header REACHABILITY (the GAP-02 symptom).

## Deviations from Plan

None — Tasks 1 and 2 executed exactly per the plan specification. The single-line collapse of the `add_subdirectory` call (vs. the multi-line block in the plan's `<action>` block) is a presentation refinement that preserves identical CMake semantics; documented in the Decisions section above.

## Deferred verify steps

- `cd ros2 && podman build --target build -t mowgli-phase2:test .` — DEFERRED-TO-PHASE-END-BUILD per CONTEXT.md "Code-only Commits, podman-Build am Phasen-Ende"
- `colcon test --packages-select mowgli_lidar_docking --ctest-args -R test_kinematic_icp_headers_reachable` — DEFERRED-TO-PHASE-END-BUILD (runs inside the built podman image)
- Pi5 5-of-5 hardware UAT per `02-08-PI5-CHECKLIST.md` — operator-gated; chained on a green phase-end podman build

## Self-Check: PASSED

All expected files present on disk:
- `ros2/src/mowgli_lidar_docking/CMakeLists.txt` (modified)
- `ros2/src/mowgli_lidar_docking/test/CMakeLists.txt` (modified)
- `ros2/src/mowgli_lidar_docking/test/test_kinematic_icp_headers_reachable.cpp` (created)
- `.planning/phases/02-lidar-dock-pose-estimation/02-09-SUMMARY.md` (this file)

Both task commits present in `git log`:
- `7470b60d` — Task 1 (CMakeLists.txt rewrite)
- `10f722a6` — Task 2 (regression smoke gtest)

All Task 1 acceptance greps (1-9) pass. All Task 2 acceptance greps (1-7) pass. Plan-level static-grep gate from `<verification>` section gate 1 outputs `GREP_GATES: PASS`. `git status --porcelain ros2/src/kinematic_icp/` returns empty. `git status --porcelain ros2/src/mowgli_lidar_docking/package.xml` returns empty. No source files under `ros2/src/mowgli_lidar_docking/src/` or `ros2/src/mowgli_lidar_docking/include/` modified.

Phase-end podman build (gates 2 + 3 from the plan's `<verification>` section) is `DEFERRED-TO-PHASE-END-BUILD` per CONTEXT.md. Operator runs `cd ros2 && podman build --target build -t mowgli-phase2:test .` after this branch lands on `feat/mag-pipeline-resurrect`.
