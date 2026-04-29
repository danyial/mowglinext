---
phase: 02
plan: 02
subsystem: lidar-dock-pose-estimation
tags: [wave-1, package-bootstrap, idock-matcher, pcd-io, confidence-metrics, key-value-loaders, gtest]
requires:
  - mowgli_geometry::key_value_parser (parse_yaml_double / parse_yaml_string / parse_yaml_int)
  - mowgli_geometry::atomic_write
  - mowgli_interfaces::msg::DockMatchConfidence
  - kinematic_icp submodule populated (Plan 02-01)
  - kiss_icp v1.2.0 transitive via FetchContent (struct default-public access — PROBE.md A1+A2)
provides:
  - mowgli_lidar_docking_lib (STATIC ament library)
  - mowgli_lidar_docking::IDockMatcher abstract base + MatchResult struct (D-16 contract)
  - mowgli_lidar_docking::save_dock_scan_pcd / save_dock_scan_pcd_atomic / load_dock_scan_pcd (D-06 + R-11)
  - mowgli_lidar_docking::compute_confidence (production, kiss_icp::VoxelHashMap)
  - mowgli_lidar_docking::compute_confidence_brute_force (test-only)
  - mowgli_lidar_docking::is_trusted (SPEC R-3 trust gate)
  - mowgli_lidar_docking::DockApproach + load/save_dock_approach_yaml (D-04)
  - mowgli_lidar_docking::DockScanMeta + load/save_dock_scan_meta_yaml + dock_scan_meta_age_exceeds (D-17, T-02-06)
  - 15 gtest cases across 5 binaries (frozen contract surface for Plans 02-03 / 02-04 / 02-06)
affects:
  - none (this plan only adds a new package; no existing files modified beyond the new submodule's package.xml/CMakeLists)
tech-stack:
  added:
    - PCL (libpcl-all-dev) — pulled via package.xml + find_package(PCL)
    - Sophus (libsophus-dev) — pulled via package.xml + find_package(Sophus)
  patterns:
    - STATIC lib + later-executable split (mirrors mowgli_coverage_planner Plan 01-05)
    - Abstract IDockMatcher base + locally-defined MockMatcher in tests (D-16, dependency injection without #ifdef)
    - Atomic PCD save via render-to-string + mowgli_geometry::atomic_write (R-11)
    - timegm-based real day arithmetic for age threshold (T-02-06; lex compare on raw ISO strings is the documented failure mode we explicitly avoid)
    - Forward declaration of kiss_icp::VoxelHashMap in confidence_metrics.hpp so kiss_icp/core/VoxelHashMap.hpp is only pulled into the .cpp
key-files:
  created:
    - ros2/src/mowgli_lidar_docking/package.xml
    - ros2/src/mowgli_lidar_docking/CMakeLists.txt
    - ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/idock_matcher.hpp
    - ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/dock_scan_io.hpp
    - ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/dock_approach_loader.hpp
    - ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/dock_scan_meta_loader.hpp
    - ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/confidence_metrics.hpp
    - ros2/src/mowgli_lidar_docking/src/dock_scan_io.cpp
    - ros2/src/mowgli_lidar_docking/src/dock_approach_loader.cpp
    - ros2/src/mowgli_lidar_docking/src/dock_scan_meta_loader.cpp
    - ros2/src/mowgli_lidar_docking/src/confidence_metrics.cpp
    - ros2/src/mowgli_lidar_docking/test/CMakeLists.txt
    - ros2/src/mowgli_lidar_docking/test/test_dock_scan_io.cpp
    - ros2/src/mowgli_lidar_docking/test/test_confidence_metrics.cpp
    - ros2/src/mowgli_lidar_docking/test/test_dock_approach_loader.cpp
    - ros2/src/mowgli_lidar_docking/test/test_dock_scan_meta_loader.cpp
    - ros2/src/mowgli_lidar_docking/test/test_idock_matcher_mock.cpp
  modified: []
decisions:
  - "Plan 02-01 PROBE.md A1+A2 confirmed kiss_icp::VoxelHashMap exposes AddPoints + GetClosestNeighbor publicly via struct default access; production code path taken with NO fallback path"
  - "GetClosestNeighbor returns std::tuple<Eigen::Vector3d, double> (the second element is the squared distance to the nearest neighbour) — confidence_metrics.cpp uses that squared distance directly instead of recomputing (p-nn).squaredNorm()"
  - "CMake target resolution: ament_target_dependencies(... mowgli_geometry kinematic_icp) — namespaced IMPORTED targets like `mowgli_geometry::mowgli_geometry` and `kinematic_icp::pipeline` are NOT explicitly named in target_link_libraries; ament_target_dependencies propagates include paths + link order via the standard ROS2 idiom (mirrors mowgli_coverage_planner Plan 01-05 line 65-74). Plan 02-04 inherits this pattern; if it needs a per-component target later it should re-probe rather than assume"
  - "PCD atomic write renders the canonical PCD v0.7 ASCII layout to a std::string and routes through mowgli_geometry::atomic_write; PCL's savePCDFile owns its own fd and has no public hook to redirect to a writable string buffer in the Kilted apt build"
  - "Age threshold uses timegm-based real day arithmetic, not lex compare on raw ISO strings (T-02-06 mitigation; Plan 02-02 RESEARCH §A5 lex-compare is the documented failure mode we explicitly avoid)"
metrics:
  duration_minutes: 32
  tasks_completed: 2
  files_touched: 17
  test_cases_added: 15
  commits: 2
  completed_date: "2026-04-29"
---

# Phase 02 Plan 02: Wave 1 mowgli_lidar_docking Bootstrap Summary

**One-liner:** Wave 1 lands the new mowgli_lidar_docking ROS2 package
with a STATIC library, the IDockMatcher abstract base (D-16), PCL ASCII
PCD save/load (D-06) routed through mowgli_geometry::atomic_write (R-11),
the kiss_icp-backed compute_confidence helper (production path per
PROBE.md A1+A2), the dock_approach.yaml + dock_scan_meta.yaml flat
key=value loaders (D-04 + D-17), and 15 gtest cases across 5 binaries —
zero TF / publisher / subscriber code, contract surface frozen for
Plans 02-03 / 02-04 / 02-06.

## Built

This plan covers two atomic tasks landed in two commits on
`feat/mag-pipeline-resurrect`:

| Task | Description | Commit |
| ---- | ----------- | ------ |
| 1 | Bootstrap package: package.xml + CMakeLists.txt + IDockMatcher contract + dock_scan_io + dock_approach_loader + dock_scan_meta_loader + their .cpp implementations | `70785508` |
| 2 | confidence_metrics.{hpp,cpp} (production + brute-force overloads) + test/CMakeLists.txt + 5 gtest binaries with 15 cases | `0d0ae778` |

## Key files

### Created

#### Package skeleton (Task 1)

- `ros2/src/mowgli_lidar_docking/package.xml` — 13 `<depend>` entries
  covering ROS2 (rclcpp, sensor_msgs, geometry_msgs, tf2,
  tf2_geometry_msgs, pcl_conversions, pcl_ros, laser_geometry),
  PCL (libpcl-all-dev), Sophus, mowgli_geometry, mowgli_interfaces,
  kinematic_icp (in-tree submodule), eigen.
- `ros2/src/mowgli_lidar_docking/CMakeLists.txt` — STATIC
  `mowgli_lidar_docking_lib` with 4 sources (dock_scan_io.cpp,
  confidence_metrics.cpp, dock_approach_loader.cpp,
  dock_scan_meta_loader.cpp). Exports
  `export_mowgli_lidar_docking_lib` via `ament_export_targets`.
  Mirrors mowgli_coverage_planner Plan 01-05 layout (lib + later
  executable split — Plan 02-04 will add `add_executable` for
  dock_scan_match_node).

#### Public headers (Task 1)

- `include/mowgli_lidar_docking/idock_matcher.hpp` — D-16 abstract
  base. `MatchResult` struct (pose_in_map: Sophus::SE3d,
  inlier_ratio: double, rmse_m: double, valid: bool) + `IDockMatcher`
  pure-virtual `Match(live_frame, lidar_to_base)`.
- `include/mowgli_lidar_docking/dock_scan_io.hpp` — three entry
  points (`save_dock_scan_pcd`, `save_dock_scan_pcd_atomic`,
  `load_dock_scan_pcd`) operating on
  `std::vector<Eigen::Vector3d>`.
- `include/mowgli_lidar_docking/dock_approach_loader.hpp` — D-04
  schema. Reader is `inline` (header-only); rejects `source` other
  than "lidar"/"tf" (T-02-01 mitigation). Writer in .cpp.
- `include/mowgli_lidar_docking/dock_scan_meta_loader.hpp` — D-17
  schema (10 fields). Reader `inline`; writer + age-check helper in
  .cpp. `dock_scan_meta_age_exceeds` declared here for tests.

#### Implementations (Tasks 1 + 2)

- `src/dock_scan_io.cpp` — PCL ASCII save (binary=false per D-06) +
  hand-rolled PCD v0.7 ASCII renderer for the atomic variant routed
  through `mowgli_geometry::atomic_write`. Load via
  `pcl::io::loadPCDFile<pcl::PointXYZ>`.
- `src/dock_approach_loader.cpp` — atomic 5-line key=value writer.
- `src/dock_scan_meta_loader.cpp` — atomic 10-line key=value writer;
  `dock_scan_meta_age_exceeds` parses the YYYY-MM-DD prefix via
  `timegm` (UTC-aware, no locale dependency) — T-02-06 mitigation.
- `include/mowgli_lidar_docking/confidence_metrics.hpp` (Task 2) —
  forward-declares `kiss_icp::VoxelHashMap` so the kiss_icp header
  is only pulled into the .cpp. `ConfidenceResult` struct +
  production `compute_confidence` + test-only
  `compute_confidence_brute_force` + `is_trusted`.
- `src/confidence_metrics.cpp` (Task 2) — production overload calls
  `voxel_map.GetClosestNeighbor(p)`, destructures the
  `std::tuple<Eigen::Vector3d, double>` return, uses the squared
  distance directly. Empty frame and zero-inlier paths return
  `{0.0, +inf}`.

#### Tests (Task 2)

- `test/CMakeLists.txt` — 5 `ament_add_gtest` registrations.
- `test/test_dock_scan_io.cpp` (3 cases) — `PCDRoundTrip` (10-point
  square+diagonals, 1e-4 m precision), `PCDAtomicWrite` (no .tmp
  orphan check), `PCDLoadMissing` (out_points unchanged on failure).
- `test/test_confidence_metrics.cpp` (4 cases) —
  `HighConfidenceOnIdenticalCloud` (200-pt square outline, ratio≥0.95
  / RMSE≤1e-6), `LowConfidenceOnNoise` (80% replaced with random
  faraway points → ratio<0.30, !trusted),
  `FlippedYawHallucination` (180°-rotated L-shape — Pitfall 1
  documentation case for the metric, asserts !trusted),
  `EmptyFrame` (no division-by-zero, rmse=+inf, !trusted).
- `test/test_dock_approach_loader.cpp` (3 cases) — `RoundTrip`
  (1e-5 float precision), `InvalidSource` (T-02-01 rejected),
  `MissingField` (whole file rejected).
- `test/test_dock_scan_meta_loader.cpp` (4 cases) — `RoundTrip`,
  `MissingField`, `AgeExceeds_TrueAt8Days`, `AgeExceeds_FalseAt6Days`.
- `test/test_idock_matcher_mock.cpp` (1 case) —
  `MockReturnsFixedResult`. MockMatcher : public IDockMatcher
  derived locally; dispatched via abstract base pointer (D-16).

### Modified

None. This plan is purely additive — only a new package directory.

## CMake Target Resolution (per Task 1 Step 0 probe)

The plan instructed running a Step-0 CMake target probe before authoring
`target_link_libraries(...)` to determine whether namespaced IMPORTED
targets like `mowgli_geometry::mowgli_geometry` and
`kinematic_icp::pipeline` exist. The host environment cannot run colcon
(macOS, no devcontainer running), so the on-disk source-of-truth probe
was used:

| Probe | Source-of-truth | Verdict |
| ----- | --------------- | ------- |
| `mowgli_geometry` exports | `ros2/src/mowgli_geometry/CMakeLists.txt:43-49` | INTERFACE library + `ament_export_targets(mowgli_geometry HAS_LIBRARY_TARGET)` + `ament_export_include_directories(include)`. The IMPORTED target name is `mowgli_geometry::mowgli_geometry`. |
| `kinematic_icp` exports | Not introspectable on-host (FetchContent-driven build path) | Cannot pre-resolve a per-component CMake target name without a colcon build. The PRBonn upstream changes its export layout across releases. |

**Decision:** use `ament_target_dependencies(mowgli_lidar_docking_lib PUBLIC
mowgli_geometry kinematic_icp ...)` for both deps, mirroring
`mowgli_coverage_planner/CMakeLists.txt:65-74` (Plan 01-05). This idiom
propagates include paths + link order through the standard ROS2
mechanism without hand-naming namespaced IMPORTED targets that may not
exist on every checkout. PCL is linked the conventional way via
`${PCL_LIBRARIES}` in `target_link_libraries`. Sophus + Eigen are
header-only IMPORTED targets so their `Sophus::Sophus` /
`Eigen3::Eigen` names are stable and used directly.

If Plan 02-04 (which adds `kinematic_icp_dock_matcher.cpp` to the same
lib) needs to reference a per-component kinematic_icp target — e.g.
`kinematic_icp::pipeline` — that plan's executor should re-probe
inside the devcontainer (`find ros2/install/kinematic_icp -name
'*.cmake'`) and amend `target_link_libraries` accordingly. The current
`ament_target_dependencies(... kinematic_icp)` line is the safest base.

## Production code path on both kiss_icp probes (per PROBE.md)

PROBE.md A1+A2 confirmed kiss_icp v1.2.0 declares `VoxelHashMap` as a
`struct` with default-public access; `AddPoints(const
std::vector<Eigen::Vector3d>&)` and `GetClosestNeighbor(const
Eigen::Vector3d&) const → std::tuple<Eigen::Vector3d, double>` are
both directly usable. **No fallback / inline voxel-bucket lookup
required.** The production path landed verbatim:

```cpp
const auto [neighbor, d2] = voxel_map.GetClosestNeighbor(p);
if (d2 <= max_d2) { ++inliers; sse += d2; }
```

The returned `double` is the squared distance — using it directly
saves a redundant `(p - neighbor).squaredNorm()` recompute and keeps
the math identical to whatever future kiss_icp internal optimization
might do behind the scenes.

## 15 test cases — full enumeration

| # | Binary | Case | Asserts |
| -- | ------ | ---- | ------- |
| 1 | test_dock_scan_io | PCDRoundTrip | save → load round-trips 10 points within 1e-4 m |
| 2 | test_dock_scan_io | PCDAtomicWrite | atomic save round-trips + no .tmp orphan after success |
| 3 | test_dock_scan_io | PCDLoadMissing | load on /nonexistent returns false; out_points unchanged |
| 4 | test_confidence_metrics | HighConfidenceOnIdenticalCloud | identical cloud → ratio≥0.95, RMSE≤1e-6, trusted=true |
| 5 | test_confidence_metrics | LowConfidenceOnNoise | 80% noise → ratio<0.30, trusted=false |
| 6 | test_confidence_metrics | FlippedYawHallucination | 180°-rotated L-shape → trusted=false (Pitfall 1) |
| 7 | test_confidence_metrics | EmptyFrame | empty frame → ratio=0, rmse=+inf, !trusted |
| 8 | test_dock_approach_loader | RoundTrip | 5 fields round-trip within 1e-5 (floats) / exact (strings) |
| 9 | test_dock_approach_loader | InvalidSource | source=garbage → load returns nullopt (T-02-01) |
| 10 | test_dock_approach_loader | MissingField | missing yaw key → whole file rejected |
| 11 | test_dock_scan_meta_loader | RoundTrip | 10 fields round-trip |
| 12 | test_dock_scan_meta_loader | MissingField | missing dock_scan_pcd_path → nullopt |
| 13 | test_dock_scan_meta_loader | AgeExceeds_TrueAt8Days | 8-day delta vs threshold=7 → true |
| 14 | test_dock_scan_meta_loader | AgeExceeds_FalseAt6Days | 6-day delta vs threshold=7 → false |
| 15 | test_idock_matcher_mock | MockReturnsFixedResult | MockMatcher : IDockMatcher returns operator-supplied result via abstract base ptr (D-16) |

All randomness uses `std::mt19937{42}` (fixed seed) → deterministic
across CI runs, per CLAUDE.md "no flaky tests" rule.

## Contracts every downstream plan can rely on

**Plan 02-03 (Python dock_scan_capture extension to
calibrate_imu_yaw_node):**
- Will write the `dock_scan.pcd` to disk using PCL's Python bindings
  (or via a one-shot subprocess invocation of a C++ helper); meta
  goes via `dock_scan_meta_loader::save_dock_scan_meta_yaml`. The
  10-field schema is frozen.

**Plan 02-04 (C++ KinematicIcpDockMatcher + dock_scan_match_node):**
- Will derive `class KinematicIcpDockMatcher : public IDockMatcher`;
  contract is the `MatchResult Match(live_frame, lidar_to_base)`
  pure-virtual.
- Will call `mowgli_lidar_docking::compute_confidence(scan, voxel_map,
  max_d)` once per /scan_kicp callback to populate
  `mowgli_interfaces::msg::DockMatchConfidence` (D-03).
- Will call `mowgli_lidar_docking::is_trusted(r, min_inlier,
  max_rmse)` for the SPEC R-3 trust gate that drives the `trusted`
  field of DockMatchConfidence and decides whether to publish on
  `/dock_match/pose`.
- Will read `dock_scan_meta.yaml` at startup via
  `load_dock_scan_meta_yaml`, then mtime-watch the file for the
  Plan 02-06 auto-refresh trigger.

**Plan 02-06 (BT nodes RecordDockApproachPose + ApproachDock +
FineDock):**
- `RecordDockApproachPose` writes via `save_dock_approach_yaml`
  (atomic) and triggers `save_dock_scan_pcd_atomic` for the auto-refresh
  path (R-11).
- `ApproachDock` reads via `load_dock_approach_yaml`.
- BT unit tests construct local MockMatcher : IDockMatcher (the
  pattern proven in test 15) to assert trust-gating behavior without
  standing up kiss_icp.

## Deferred verify steps

The orchestrator runs the actual ROS2 build at end-of-phase via podman
inside the devcontainer. The host (macOS) has no colcon and no
devcontainer running. Each command below is the verbatim verify step
the plan specified that this executor could not run; the phase-end
build is expected to exercise all of them in a single batched pass.

- **Task 1 — colcon build of mowgli_lidar_docking package skeleton**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2 && \
    colcon build --packages-select mowgli_lidar_docking \
      --event-handlers console_cohesion+ 2>&1 | tail -20
  ```

  Expected: exit 0, `mowgli_lidar_docking_lib.a` installed under
  `install/mowgli_lidar_docking/lib/`.

- **Task 2 — colcon build + colcon test of mowgli_lidar_docking**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2 && \
    colcon build --packages-select mowgli_lidar_docking \
      --event-handlers console_cohesion+ 2>&1 | tail -10 && \
    colcon test --packages-select mowgli_lidar_docking \
      --event-handlers console_cohesion+ 2>&1 | tail -15 && \
    colcon test-result --verbose \
      --test-result-base build/mowgli_lidar_docking 2>&1 | tail -25
  ```

  Expected: 5 gtest binaries, 15 cases, 0 failures.

- **PROBE re-confirmation (defensive — already passed at PROBE.md time)**

  ```bash
  # Inside devcontainer after FetchContent has populated:
  find /ros2_ws/build/kinematic_icp/_deps -name VoxelHashMap.hpp 2>/dev/null
  grep -n "AddPoints\|GetClosestNeighbor" \
    /ros2_ws/build/kinematic_icp/_deps/kiss_icp-src/cpp/kiss_icp/core/VoxelHashMap.hpp
  ```

  Expected: header found at `_deps/kiss_icp-src/cpp/kiss_icp/core/VoxelHashMap.hpp`;
  both functions declared as struct members (default public).

## Drift detection

| Check | Expected | Actual |
| ----- | -------- | ------ |
| brace balance across all 12 new .hpp/.cpp + 5 test .cpp | open == close per file | green (see Task 2 verification — all balanced) |
| package depends include `kinematic_icp` + `mowgli_geometry` + `mowgli_interfaces` + `laser_geometry` + `pcl_ros` + `sophus` | all 6 present | green |
| no TF / publisher / subscriber in lib | `! grep -rq "TransformBroadcaster\|create_publisher\|create_subscription" src/` | green |
| 5 ament_add_gtest registrations | exactly 5 | green (5 lines in test/CMakeLists.txt) |
| 15 TEST cases total | exactly 15 across the 5 binaries | green (3+4+3+4+1) |

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 — Blocking] CMake target resolution probe could not be run on host**
- **Found during:** Task 1 Step 0
- **Issue:** Plan instructed running an in-devcontainer probe of
  `mowgli_geometry::mowgli_geometry` and `kinematic_icp::pipeline`
  IMPORTED-target existence before authoring `target_link_libraries`.
  Host (macOS) cannot run colcon; the namespaced kinematic_icp target
  cannot be resolved on-disk because kinematic_icp's CMake exports are
  generated at build time (FetchContent declares kiss_icp; cmake
  config files are written into install/.../cmake/ only after a
  successful colcon build). PROBE.md explicitly notes this in the
  "Submodule layout" section.
- **Fix:** Followed the safer pattern from
  `ros2/src/mowgli_coverage_planner/CMakeLists.txt:65-74` and used
  `ament_target_dependencies(... mowgli_geometry kinematic_icp ...)`
  for both deps instead of explicitly hand-naming
  `mowgli_geometry::mowgli_geometry` or
  `kinematic_icp::pipeline` in `target_link_libraries`. This propagates
  include paths via the standard ROS2 idiom without making
  per-component-target assumptions that may not hold across kinematic_icp
  upstream releases. PCL gets `${PCL_LIBRARIES}` directly because
  PCL is not an ament package; Sophus and Eigen3 use
  `Sophus::Sophus` / `Eigen3::Eigen` because those are stable
  header-only IMPORTED targets shipped by their config files. The
  decision is recorded in the "CMake Target Resolution" section
  above so Plan 02-04's executor inherits the rationale.
- **Files modified:** `ros2/src/mowgli_lidar_docking/CMakeLists.txt`
- **Commit:** `70785508`

**2. [Rule 1 — Bug] Plan snippet's GetClosestNeighbor return-type mismatch**
- **Found during:** Task 2 (writing confidence_metrics.cpp)
- **Issue:** The plan's confidence_metrics.cpp example block (line
  642 of 02-02-PLAN.md) showed
  `const Eigen::Vector3d nn = voxel_map.GetClosestNeighbor(p);` but
  PROBE.md "Verbatim declaration" (line 78) confirms kiss_icp
  v1.2.0's actual signature is
  `std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(...)
  const;`. Following the plan literally would have produced a
  compile error.
- **Fix:** Used the correct signature in confidence_metrics.cpp with
  structured-binding destructure
  `const auto [neighbor, d2] = voxel_map.GetClosestNeighbor(p);` and
  consumed the squared distance directly (the second tuple element)
  instead of recomputing `(p - nn).squaredNorm()`. This matches
  PROBE.md's "Implications for Plan 02-02" code outline (line 169-176)
  rather than the plan body's snippet.
- **Files modified:** `ros2/src/mowgli_lidar_docking/src/confidence_metrics.cpp`
- **Commit:** `0d0ae778`

**3. [Rule 3 — Blocking] Task-1 CMakeLists referenced confidence_metrics.cpp before Task 2 created it**
- **Found during:** Task 1 (committing the package skeleton standalone)
- **Issue:** The plan body snippet listed all 4 source files including
  `src/confidence_metrics.cpp` in the Task-1 `add_library` block,
  AND included `add_subdirectory(test)` — but Task 2 is the one that
  creates both confidence_metrics.cpp and the test/ directory.
  Committing Task 1 verbatim from the plan would leave the package in
  a non-buildable state until Task 2 lands. The phase-end batched
  podman build would catch this, but the per-task commit would still
  be a transient regression.
- **Fix:** In Task 1's CMakeLists.txt, omitted
  `src/confidence_metrics.cpp` from the lib sources and commented out
  `add_subdirectory(test)` with a TODO. Task 2's commit then added
  both back. Each commit is independently buildable in isolation
  (modulo the deferred phase-end colcon run).
- **Files modified:** `ros2/src/mowgli_lidar_docking/CMakeLists.txt`
- **Commits:** `70785508` (Task 1 transient state),
  `0d0ae778` (Task 2 final state)

## Threat Flags

None. The new package's library implements every threat-register
mitigation (T-02-01 source validation; T-02-02 PCD load failure → false;
T-02-03 NaN-propagation natural exclusion via squaredNorm/<=
comparison; T-02-05 atomic_write for both yaml writers; T-02-06
real-day-arithmetic instead of lex compare). No new trust boundaries
are introduced — the loaders read the same disk files the plan's
threat model already covers.

## Self-Check

Verifying claims before proceeding to STATE.md / ROADMAP.md updates.

### Files claimed exist

```
FOUND: ros2/src/mowgli_lidar_docking/package.xml
FOUND: ros2/src/mowgli_lidar_docking/CMakeLists.txt
FOUND: ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/idock_matcher.hpp
FOUND: ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/dock_scan_io.hpp
FOUND: ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/dock_approach_loader.hpp
FOUND: ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/dock_scan_meta_loader.hpp
FOUND: ros2/src/mowgli_lidar_docking/include/mowgli_lidar_docking/confidence_metrics.hpp
FOUND: ros2/src/mowgli_lidar_docking/src/dock_scan_io.cpp
FOUND: ros2/src/mowgli_lidar_docking/src/dock_approach_loader.cpp
FOUND: ros2/src/mowgli_lidar_docking/src/dock_scan_meta_loader.cpp
FOUND: ros2/src/mowgli_lidar_docking/src/confidence_metrics.cpp
FOUND: ros2/src/mowgli_lidar_docking/test/CMakeLists.txt
FOUND: ros2/src/mowgli_lidar_docking/test/test_dock_scan_io.cpp
FOUND: ros2/src/mowgli_lidar_docking/test/test_confidence_metrics.cpp
FOUND: ros2/src/mowgli_lidar_docking/test/test_dock_approach_loader.cpp
FOUND: ros2/src/mowgli_lidar_docking/test/test_dock_scan_meta_loader.cpp
FOUND: ros2/src/mowgli_lidar_docking/test/test_idock_matcher_mock.cpp
```

### Commits claimed exist

```
FOUND: 70785508 — Task 1
FOUND: 0d0ae778 — Task 2
```

## Self-Check: PASSED
