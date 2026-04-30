# Phase 2 — Deferred Items

Out-of-scope discoveries from plan execution that should be tracked outside
this phase.

## From Plan 02-01

- **MapArea.h MD5 drift in firmware rosserial bindings**
  - File: `firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/MapArea.h`
  - Symptom: `python3 firmware/scripts/sync_ros_lib.py` rewrites the MD5 hash
    from `95c4cd71205ce55b6d40a3234589b32a` to `c7d63fd7c5f100e71c60e659a1a99a1e`
    even though `MapArea.msg` was not touched in Plan 02-01.
  - Cause: pre-existing — `MapArea.msg` text changed in a prior plan/commit
    without re-running `sync_ros_lib.py`. The header has been carrying a
    stale MD5 ever since.
  - Risk: rosserial communication mismatch between firmware (computes MD5 from
    its compiled-in header) and ROS2 host (computes MD5 from `MapArea.msg`).
    If the firmware ever subscribes to `MapArea`, messages will be silently
    rejected. Today firmware does not consume `MapArea`, so impact is latent.
  - Action: filed for follow-up. `git checkout -- ...MapArea.h` reverted the
    drift so Plan 02-01's scope remains DockMatchConfidence-only.
  - Suggested fix: re-run `sync_ros_lib.py` and commit the MapArea.h MD5 fix
    as a separate `chore: re-sync firmware MapArea.h` PR outside Phase 2.

## From Plan 02-07

- **`ros.generated.ts` MapAreaConstants enum init expressions are invalid TS**
  - File: `gui/web/src/types/ros.generated.ts` lines 281-283
  - Symptom: `cd gui/web && yarn build` fails with
    ```
    src/types/ros.generated.ts(281,22): error TS2474: const enum member initializers must be constant expressions.
    src/types/ros.generated.ts(281,22): error TS2565: Property 'NARROW_AREA_SKIP' is used before being assigned.
    ```
    The generator emits `NARROW_AREA_SKIP = NARROW_AREA_SKIP` (self-
    referential), which TypeScript correctly rejects — the constants need
    numeric values or a different generator output shape.
  - Cause: pre-existing — bug in `gui/generate_ts_types.sh` introduced before
    Plan 02-07. The file is NOT imported anywhere (the canonical types live
    in `ros.ts`), but `tsconfig.json:include = ["src"]` makes `tsc` compile
    it anyway.
  - Risk: `yarn build` exits 2 on host / CI. Plan 02-07's GUI changes
    (`useDockMatch.ts`, `DockMatchCard.tsx`, `MowerStatus.tsx`) compile
    cleanly in isolation; the failure is entirely in `ros.generated.ts`.
  - Action: NOT fixed in Plan 02-07 (deviation rule SCOPE BOUNDARY:
    out-of-scope, pre-existing). `cd gui && go build ./...` exits 0 — the
    Go side of Plan 02-07's GUI extension is verified.
  - Suggested fix: either (a) regenerate `ros.generated.ts` after fixing
    the `MapAreaConstants` enum emission in `gui/generate_ts_types.sh`,
    OR (b) add `ros.generated.ts` to `tsconfig.json:exclude` until the
    canonical `ros.ts` is regenerated from the same source. Out-of-scope
    for Phase 2.

## From phase-end podman build (post Plan 02-08)

Phase-end build run (`podman build --target build`) on `feat/mag-pipeline-resurrect`
HEAD `0e8b2038` produced:

```
Summary: 9 packages finished [1min 25s]
  1 package failed: mowgli_lidar_docking
  9 packages had stderr output: kinematic_icp mowgli_coverage_planner mowgli_geometry
                                 mowgli_hardware mowgli_lidar_docking mowgli_localization
                                 mowgli_map mowgli_monitoring mowgli_nav2_plugins
  3 packages not processed (mowgli_behavior, mowgli_simulation, mowgli_bringup)
```

### Bug 1: `libsophus-dev` does not exist on Ubuntu Noble arm64 — FIXED

- File: `ros2/Dockerfile`
- Symptom: `E: Unable to locate package libsophus-dev` during apt install.
- Root cause: Plan 02-01 RESEARCH listed `libsophus-dev` as a Phase 2 apt dep,
  but Ubuntu Noble (24.04) does not ship that package on arm64. The ROS2
  distribution provides the equivalent as `ros-kilted-sophus`.
- Fix: commit `0e8b2038` replaced `libsophus-dev` with `ros-kilted-sophus`.
- Status: fixed in this branch.

### Bug 2: `mowgli_lidar_docking` cannot reach kinematic_icp / kiss_icp C++ headers — OPEN

- Files: `ros2/src/mowgli_lidar_docking/CMakeLists.txt`,
  `include/mowgli_lidar_docking/kinematic_icp_dock_matcher.hpp` (Plan 02-04),
  `src/confidence_metrics.cpp` (Plan 02-02).
- Symptom (verbatim from build log):
  ```
  fatal error: kinematic_icp/pipeline/KinematicICP.hpp: No such file or directory
  fatal error: kiss_icp/core/VoxelHashMap.hpp: No such file or directory
  Failed   <<< mowgli_lidar_docking [8.16s, exited with code 2]
  ```
- Root cause: the kinematic_icp ROS package (PRBonn upstream submodule pinned
  via `.gitmodules`) does NOT export its underlying C++ library headers via
  ament. Specifically:
  - `kinematic_icp/pipeline/KinematicICP.hpp` lives in
    `ros2/src/kinematic_icp/cpp/kinematic_icp/pipeline/` and is built as
    target `kinematic_icp_pipeline` via `add_subdirectory(... cpp/kinematic_icp)`
    inside `ros2/src/kinematic_icp/ros/CMakeLists.txt`. Neither the headers
    nor the library target are installed or `ament_export_*`-ed.
  - `kiss_icp/core/VoxelHashMap.hpp` is FetchContent'd at kinematic_icp build
    time into `ros2/build/kinematic_icp/_deps/kiss_icp-src/cpp/kiss_icp/core/`
    and likewise not installed.
  - Plan 02-02 PROBE.md (lines 47-52) flagged this exact integration risk,
    but Plan 02-02 + Plan 02-04 executors used `ament_target_dependencies(
    mowgli_lidar_docking_lib PUBLIC kinematic_icp)` which only propagates
    the ROS-wrapper headers (`include/kinematic_icp_ros/...`), not the
    C++ library headers we actually need.
- Risk: BLOCKS Phase 2 production deployment. dock_scan_match executable
  cannot be compiled. Plan 02-04 + Plan 02-06 + Plan 02-07 + Plan 02-08
  (which all depend transitively on mowgli_lidar_docking) cannot deliver
  end-to-end behavior on Pi5 until this is resolved.
- Affected downstream packages (3 not processed in build):
  `mowgli_behavior` (Plan 02-06 BT nodes consume IDockMatcher contract from
  this package), `mowgli_simulation` (Plan 02-08 sim e2e), `mowgli_bringup`
  (Plan 02-04 launch wiring).
- Suggested fix paths (gap closure required — too large for inline fix):
  - **(A) Self-FetchContent + add_subdirectory in mowgli_lidar_docking**: in
    `mowgli_lidar_docking/CMakeLists.txt`, replicate the kinematic_icp ros
    pattern — `FetchContent_Declare(kiss_icp ...)` for the URL/version
    matched to kinematic_icp's `cpp/kinematic_icp/kiss_icp/kiss-icp.cmake`,
    plus `add_subdirectory(${CMAKE_SOURCE_DIR}/../kinematic_icp/cpp/kinematic_icp
    ${CMAKE_CURRENT_BINARY_DIR}/kinematic_icp_cpp_for_mowgli)` to build the
    cpp lib in-tree. Adds ~3-5 min to the build but keeps mowgli_lidar_docking
    self-contained. CMake target name conflicts must be resolved via
    `EXCLUDE_FROM_ALL` or scoped subdirectory naming.
  - **(B) Patch the kinematic_icp submodule** to install + ament_export the
    cpp headers and library targets. This means carrying a local diff against
    PRBonn upstream, either by pinning a fork (change `.gitmodules`) or by
    keeping a generated patch file applied during build. Cleaner long-term
    but introduces fork-maintenance burden.
  - **(C) Vendor minimum kiss_icp headers + symbols** into mowgli_lidar_docking.
    Hacky; loses upstream fix tracking; not recommended.
- Action item: file as gap-closure plan via `/gsd-plan-phase 2 --gaps`. The
  gap-closure plan author should pick (A) or (B) and add a regression test
  that the resulting `dock_scan_match` executable can be loaded on Pi5.
- Verified-good packages from same build (do not regress):
  mowgli_geometry (Plan 02-01 key_value_parser), mowgli_hardware +
  mowgli_map (Plan 02-01 inline-parser→shared-header migration),
  mowgli_localization (Plan 02-03 dock_scan_capture, Plan 02-05 cascade),
  kinematic_icp (PRBonn upstream submodule), plus all Phase-1 packages.
